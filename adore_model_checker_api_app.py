#!/usr/bin/env python3

import sys
import os
import json
import queue
import threading
import time
from datetime import datetime
from flask import Flask, Response, request, jsonify


def setup_imports():
    try:
        from adore_model_checker.model_checker_api import get_model_check_blueprint, stop_model_check_worker
    except ImportError:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        if parent_dir not in sys.path:
            sys.path.insert(0, parent_dir)
        from adore_model_checker.model_checker_api import get_model_check_blueprint, stop_model_check_worker
    return get_model_check_blueprint, stop_model_check_worker


def _dashboard_html():
    # 1. Next to this file (development: vendor/adore_model_checker/)
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, 'adore_model_checker_dashboard.html'),
        # Inside the adore_model_checker package subdir (installed package)
        os.path.join(here, 'adore_model_checker', 'adore_model_checker_dashboard.html'),
        os.path.join(here, '..', 'adore_model_checker_dashboard.html'),
    ]
    for path in candidates:
        abs_path = os.path.normpath(path)
        if os.path.exists(abs_path):
            with open(abs_path, 'r') as fh:
                return fh.read()

    # 2. importlib.resources — works for installed wheels
    try:
        from importlib.resources import files
        resource = files('adore_model_checker').joinpath('adore_model_checker_dashboard.html')
        if resource.is_file():
            return resource.read_text(encoding='utf-8')
    except Exception:
        pass

    return "<h1>Dashboard not found</h1><p>adore_model_checker_dashboard.html missing.</p>"


# ── Config file storage ──────────────────────────────────────────
def _config_dir():
    """Resolve the directory used to store uploaded/managed config files."""
    candidates = [
        os.environ.get('ADORE_CONFIG_DIR'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config'),
    ]
    for path in candidates:
        if path:
            os.makedirs(path, exist_ok=True)
            _seed_default_config(path)
            return path
    raise RuntimeError("Cannot determine config directory")


def _seed_default_config(config_dir):
    """Copy the canonical package default.yaml into config_dir if not already present."""
    dest = os.path.join(config_dir, 'default.yaml')
    if os.path.exists(dest):
        return

    content = _load_package_default_yaml()
    if content:
        with open(dest, 'w') as f:
            f.write(content)
        return

    raise RuntimeError(
        "Cannot locate adore_model_checker/config/default.yaml — "
        "ensure the package is installed correctly."
    )


def _load_package_default_yaml():
    """Return the canonical default.yaml content from the package, or None on failure."""
    try:
        from importlib.resources import files
        resource = files('adore_model_checker').joinpath('config/default.yaml')
        if resource.is_file():
            return resource.read_text(encoding='utf-8')
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in [
        os.path.join(here, 'adore_model_checker', 'config', 'default.yaml'),
        os.path.join(here, 'config', 'default.yaml'),
    ]:
        if os.path.exists(candidate):
            with open(candidate, 'r') as f:
                return f.read()

    return None



def _safe_config_name(name):
    """Return True if name is a safe relative yaml filename."""
    return (
        name
        and not os.sep in name
        and not name.startswith('.')
        and name.lower().endswith(('.yaml', '.yml'))
    )


# ── Log broadcaster ──────────────────────────────────────────────
class LogBroadcaster:
    """Fan out log lines to all active SSE clients."""

    def __init__(self, maxlen=5000):
        self._lock = threading.Lock()
        self._clients = []
        self._buffer = []
        self._maxlen = maxlen

    def write(self, text, stream='stdout'):
        msg = json.dumps({
            'text': text.rstrip(),
            'stream': stream,
            'time': datetime.now().strftime('%H:%M:%S'),
        })
        with self._lock:
            self._buffer.append(msg)
            if len(self._buffer) > self._maxlen:
                self._buffer.pop(0)
            for q in list(self._clients):
                try:
                    q.put_nowait(msg)
                except Exception:
                    pass

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self._lock:
            # Replay last 200 lines for new subscribers
            for line in self._buffer[-200:]:
                try:
                    q.put_nowait(line)
                except Exception:
                    pass
            self._clients.append(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            try:
                self._clients.remove(q)
            except ValueError:
                pass


log_broadcaster = LogBroadcaster()


class _BroadcastLogHandler(logging.Handler):
    """Routes Python logging records into the log broadcaster."""

    def emit(self, record):
        try:
            msg = self.format(record)
            stream = 'stderr' if record.levelno >= logging.WARNING else 'stdout'
            log_broadcaster.write(msg, stream)
        except Exception:
            pass


def _install_log_handler():
    handler = _BroadcastLogHandler()
    handler.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)s %(message)s',
                                           datefmt='%H:%M:%S'))
    root = logging.getLogger()
    if not any(isinstance(h, _BroadcastLogHandler) for h in root.handlers):
        root.addHandler(handler)


_install_log_handler()


class _TeeStream:
    """Wraps an existing stream, forwarding writes to both the original and the broadcaster."""

    def __init__(self, original, stream_name):
        self._original = original
        self._stream   = stream_name

    def write(self, text):
        self._original.write(text)
        if text.strip():
            log_broadcaster.write(text, self._stream)

    def flush(self):
        self._original.flush()

    def __getattr__(self, name):
        return getattr(self._original, name)


def _install_tee():
    """Replace sys.stdout/stderr with tee wrappers once, at module import time."""
    if not isinstance(sys.stdout, _TeeStream):
        sys.stdout = _TeeStream(sys.stdout, 'stdout')
    if not isinstance(sys.stderr, _TeeStream):
        sys.stderr = _TeeStream(sys.stderr, 'stderr')


_install_tee()


# ── History store ────────────────────────────────────────────────
def _history_dir():
    candidates = [
        os.environ.get('ADORE_HISTORY_DIR'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'history'),
    ]
    for path in candidates:
        if path:
            os.makedirs(path, exist_ok=True)
            return path
    raise RuntimeError("Cannot determine history directory")


def _save_run_history(run_id, result_data, log_text=None):
    hdir = _history_dir()
    with open(os.path.join(hdir, f'run_{run_id}.json'), 'w') as f:
        json.dump(result_data, f, indent=2, default=str)
    if log_text:
        with open(os.path.join(hdir, f'run_{run_id}.log'), 'w') as f:
            f.write(log_text)


def _load_history_index():
    hdir = _history_dir()
    runs = []
    for fname in sorted(os.listdir(hdir), reverse=True):
        if not fname.startswith('run_') or not fname.endswith('.json'):
            continue
        path = os.path.join(hdir, fname)
        try:
            with open(path) as f:
                data = json.load(f)
            runs.append({
                'run_id':         data.get('run_id'),
                'status':         data.get('status'),
                'overall_result': data.get('results', {}).get('SUMMARY', {}).get('overall_result'),
                'config_file':    data.get('config_file'),
                'completed_at':   data.get('completed_at'),
            })
        except Exception:
            pass
    return runs


# ── App factory ──────────────────────────────────────────────────
def _patch_history_dir(writable_path):
    """Replace _history_dir in the installed model_checker module so
    ContinuousMonitorEngine.__init__ never tries to create directories
    inside the (read-only) package install tree."""
    try:
        import adore_model_checker.model_checker as _mc
        os.makedirs(writable_path, exist_ok=True)
        _mc._history_dir = lambda: writable_path
    except Exception:
        pass


def create_app():
    app = Flask(__name__)

    get_model_check_blueprint, _ = setup_imports()
    blueprint = get_model_check_blueprint()
    app.register_blueprint(blueprint)

    # Patch after registration so the api singleton is already initialised.
    try:
        from adore_model_checker.model_checker_api import _get_api
        _patch_history_dir(_get_api().log_directory)
    except Exception:
        pass

    @app.after_request
    def add_cors(response):
        response.headers['Access-Control-Allow-Origin']  = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, DELETE, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        return response

    @app.route('/')
    def index():
        return {
            'message': 'Adore Model Checker API',
            'version': '0.2.0',
            'endpoints': {
                'blueprint':     '/api/model_check/',
                'dashboard':     '/api/model_checker/dashboard',
                'session':       '/api/model_checker/session/start',
                'history':       '/api/model_checker/history',
                'configs':       '/api/model_checker/configs',
                'logs':          '/api/model_checker/logs/stream',
                'continuous_ext': '/api/model_checker/continuous/',
            }
        }

    @app.route('/health')
    def health():
        return {'status': 'healthy'}

    # Dashboard — /api/model_checker/ prefix never collides with the blueprint.
    @app.route('/api/model_checker/dashboard')
    def dashboard():
        return Response(_dashboard_html(), mimetype='text/html')

    # Disabled-proposition store for continuous mode.
    _continuous_state = {'disabled_propositions': set()}

    @app.route('/api/model_checker/continuous/disabled', methods=['POST'])
    def set_disabled_propositions():
        data = request.get_json(silent=True) or {}
        _continuous_state['disabled_propositions'] = set(data.get('disabled_propositions', []))
        return jsonify({'ok': True})

    @app.route('/api/model_checker/continuous/violations/filtered')
    def filtered_violations():
        try:
            from adore_model_checker.model_checker_api import _get_api
            api = _get_api()
        except Exception as e:
            return jsonify({'error': str(e)}), 500

        if not api._continuous_engine:
            return jsonify({'error': 'Continuous monitoring not initialized'}), 404

        disabled = _continuous_state['disabled_propositions']
        limit = request.args.get('limit', type=int)
        proposition = request.args.get('proposition')
        vehicle_id = request.args.get('vehicle_id', type=int)

        violations = api._continuous_engine.get_violations(
            limit=limit, proposition=proposition, vehicle_id=vehicle_id
        )
        if disabled:
            violations = [v for v in violations if v.get('proposition') not in disabled]
        return jsonify({'violations': violations, 'count': len(violations)})

    # Forward /api/model_checker/continuous/* to the blueprint's view functions directly.
    # This avoids duplicating ContinuousMonitorEngine construction and ensures
    # the installed version's initialisation logic (which may differ from source)
    # is always used. Called within the current request context — valid in Flask.
    @app.route('/api/model_checker/continuous/<path:subpath>', methods=['GET', 'POST', 'OPTIONS'])
    def proxy_continuous(subpath):
        from flask import current_app
        # Map subpath to blueprint endpoint name, handling path parameters
        parts = subpath.split('/')
        base = parts[0]
        endpoint = f'model_check_blueprint.continuous_{base}'
        fn = current_app.view_functions.get(endpoint)
        if fn is None:
            return jsonify({'error': f'Unknown endpoint: /continuous/{subpath}'}), 404
        try:
            return fn(*parts[1:]) if len(parts) > 1 else fn()
        except Exception as e:
            return jsonify({'error': str(e)}), 500

    # ── Timed session endpoint ────────────────────────────────────
    @app.route('/api/model_checker/session/start', methods=['POST'])
    def session_start():
        data        = request.get_json(silent=True) or {}
        config_file = data.get('config_file', 'config/default.yaml')
        mode        = data.get('mode', 'online')
        duration    = float(data.get('duration', 60))
        vehicle_id  = int(data.get('vehicle_id', 0))
        bag_file    = data.get('bag_file')
        label       = data.get('label', '')

        try:
            from adore_model_checker.model_checker_api import _get_api
            api_instance = _get_api()
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500

        try:
            if mode == 'offline':
                if not bag_file:
                    return jsonify({'success': False, 'error': 'bag_file required for offline mode'}), 400
                run_id = api_instance.worker.queue_offline_run(
                    config_file=config_file,
                    bag_file=bag_file,
                    label=label,
                )
            else:
                run_id = api_instance.worker.queue_online_run(
                    config_file=config_file,
                    duration=duration,
                    vehicle_id=vehicle_id,
                    label=label,
                )
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500

        def _persist_when_done():
            deadline = time.time() + duration + 120
            while time.time() < deadline:
                try:
                    run = api_instance.cache.get_run(run_id)
                    if run and run.status.value in ('completed', 'failed', 'cancelled', 'error'):
                        result_data = {
                            'run_id':       run_id,
                            'status':       run.status.value,
                            'config_file':  config_file,
                            'completed_at': datetime.utcnow().isoformat(),
                            'results':      getattr(run, 'results', None),
                        }
                        _save_run_history(run_id, result_data)
                        return
                except Exception:
                    pass
                time.sleep(3)

        threading.Thread(target=_persist_when_done, daemon=True).start()

        return jsonify({'success': True, 'run_id': run_id})

    @app.route('/api/model_checker/session/<int:run_id>/cancel', methods=['POST'])
    def session_cancel(run_id):
        try:
            from adore_model_checker.model_checker_api import _get_api
            api_instance = _get_api()
            api_instance.worker.cancel_run(run_id)
            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500

    @app.route('/api/model_checker/result/<int:run_id>')
    def result_proxy(run_id):
        from flask import current_app
        fn = current_app.view_functions.get('model_check_blueprint.get_result')
        if fn is None:
            return jsonify({'error': 'Result endpoint not available'}), 404
        try:
            return fn(run_id)
        except Exception as e:
            return jsonify({'error': str(e)}), 500

    # ── History endpoints ─────────────────────────────────────────
    @app.route('/api/model_checker/history')
    def history_list():
        try:
            return jsonify({'runs': _load_history_index()})
        except Exception as e:
            return jsonify({'error': str(e)}), 500

    @app.route('/api/model_checker/history/<int:run_id>/log')
    def history_log(run_id):
        hdir = _history_dir()
        log_path = os.path.join(hdir, f'run_{run_id}.log')
        if not os.path.exists(log_path):
            return jsonify({'log': None})
        with open(log_path) as f:
            return jsonify({'log': f.read()})

    # ── Config file endpoints ─────────────────────────────────────
    @app.route('/api/model_checker/configs', methods=['GET'])
    def configs_list():
        try:
            cdir    = _config_dir()
            configs = []
            for fname in sorted(os.listdir(cdir)):
                if not fname.lower().endswith(('.yaml', '.yml')):
                    continue
                fpath = os.path.join(cdir, fname)
                stat  = os.stat(fpath)
                configs.append({
                    'name':     fname,
                    'size':     stat.st_size,
                    'modified': datetime.fromtimestamp(stat.st_mtime).isoformat(),
                })
            return jsonify({'configs': configs})
        except Exception as e:
            return jsonify({'error': str(e)}), 500

    @app.route('/api/model_checker/configs/<path:name>', methods=['GET'])
    def config_get(name):
        if not _safe_config_name(name):
            return jsonify({'error': 'Invalid filename'}), 400
        path = os.path.join(_config_dir(), name)
        if not os.path.exists(path):
            return jsonify({'error': 'Not found'}), 404
        with open(path) as f:
            return jsonify({'name': name, 'content': f.read()})

    @app.route('/api/model_checker/configs', methods=['POST'])
    def config_save():
        data    = request.get_json(silent=True) or {}
        name    = data.get('name', '').strip()
        content = data.get('content', '')
        if not _safe_config_name(name):
            return jsonify({'error': 'Invalid filename'}), 400
        path = os.path.join(_config_dir(), name)
        with open(path, 'w') as f:
            f.write(content)
        return jsonify({'success': True, 'name': name})

    @app.route('/api/model_checker/configs/<path:name>', methods=['DELETE'])
    def config_delete(name):
        if not _safe_config_name(name):
            return jsonify({'error': 'Invalid filename'}), 400
        path = os.path.join(_config_dir(), name)
        if not os.path.exists(path):
            return jsonify({'error': 'Not found'}), 404
        os.remove(path)
        return jsonify({'success': True})

    # ── Log stream (SSE) ──────────────────────────────────────────
    @app.route('/api/model_checker/logs/stream')
    def logs_stream():
        def generate():
            q = log_broadcaster.subscribe()
            try:
                yield 'retry: 3000\n\n'
                while True:
                    try:
                        msg = q.get(timeout=20)
                        yield f'data: {msg}\n\n'
                    except queue.Empty:
                        yield ': keepalive\n\n'
            except GeneratorExit:
                pass
            finally:
                log_broadcaster.unsubscribe(q)

        return Response(
            generate(),
            mimetype='text/event-stream',
            headers={
                'Cache-Control': 'no-cache',
                'X-Accel-Buffering': 'no',
            },
        )

    return app


def main():
    app = create_app()

    try:
        print("Starting Adore Model Checker API server...")
        print("Available at:  http://localhost:5000")
        print("API endpoints: http://localhost:5000/api/model_check/")
        print("Dashboard:     http://localhost:5000/api/model_checker/dashboard")
        app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        try:
            _, stop_model_check_worker = setup_imports()
            stop_model_check_worker()
        except Exception:
            pass


if __name__ == '__main__':
    main()
