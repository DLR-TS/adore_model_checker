#!/usr/bin/env python3

import threading
import time
import logging
import sys
import os
import json
from datetime import datetime, timezone
from flask import Blueprint, jsonify, request, send_file
from werkzeug.utils import secure_filename
import tempfile
import subprocess
import io
from contextlib import redirect_stdout, redirect_stderr
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum
import traceback
import pkg_resources
from pathlib import Path
import shutil

from .model_checker import VehicleMonitorAnalyzer, ConfigLoader, MonitoringConfig


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

__all__ = [
    'ModelCheckAPI',
    'ModelCheckCache', 
    'ModelCheckWorker',
    'get_model_check_blueprint',
    'stop_model_check_worker',
    'model_check_api',
    'model_check_blueprint'
]

class ConfigResolver:
    """Resolves config file paths based on installation context"""
    
    @staticmethod
    def is_development_mode():
        """Detect if running from repository (development) vs installed package"""
        try:
            current_dir = Path(__file__).parent.absolute()
            for parent in [current_dir] + list(current_dir.parents):
                if (parent / 'setup.py').exists():
                    return True, parent
            return False, None
        except:
            return False, None
    
    @staticmethod
    def get_config_directory():
        """Get the appropriate config directory based on context"""
        is_dev, repo_root = ConfigResolver.is_development_mode()
        
        if is_dev and repo_root:
            config_dir = repo_root / 'config'
            logger.info(f"Development mode detected. Using config directory: {config_dir}")
            config_dir.mkdir(parents=True, exist_ok=True)
            return str(config_dir)
        else:
            config_locations = ConfigResolver._get_package_config_locations()
            
            for config_dir in config_locations:
                try:
                    config_path = Path(config_dir)
                    
                    if config_path.exists() and os.access(config_path, os.W_OK):
                        logger.info(f"Using existing writable config directory: {config_path}")
                        return str(config_path)
                    
                    elif config_path.exists() and os.access(config_path, os.R_OK):
                        config_files = list(config_path.glob('*.yaml')) + list(config_path.glob('*.yml'))
                        if config_files:
                            logger.info(f"Using read-only config directory with existing configs: {config_path}")
                            return str(config_path)
                    
                    elif not config_path.exists():
                        config_path.mkdir(parents=True, exist_ok=True)
                        logger.info(f"Created new config directory: {config_path}")
                        return str(config_path)
                        
                except (PermissionError, OSError) as e:
                    logger.debug(f"Cannot use config directory {config_dir}: {e}")
                    continue
            
            fallback_dir = Path(tempfile.gettempdir()) / 'adore_model_checker_config'
            fallback_dir.mkdir(parents=True, exist_ok=True)
            logger.warning(f"Using fallback temporary config directory: {fallback_dir}")
            return str(fallback_dir)
    
    @staticmethod
    def _get_package_config_locations():
        """Get list of potential config locations in order of preference"""
        locations = []
        
        # 1. User config directory
        user_config_dir = Path.home() / '.config' / 'adore_model_checker'
        locations.append(str(user_config_dir))
        
        # 2. User local directory
        user_local_dir = Path.home() / '.local' / 'share' / 'adore_model_checker' / 'config'
        locations.append(str(user_local_dir))
        
        # 3. Try to use package data directory (if available and writable)
        try:
            package_dir = Path(pkg_resources.resource_filename('adore_model_checker', ''))
            package_config_dir = package_dir / 'config'
            locations.append(str(package_config_dir))
        except:
            pass
        
        # 4. System config directory (if writable)
        system_config_dir = Path('/etc') / 'adore_model_checker'
        locations.append(str(system_config_dir))
        
        return locations
    
    @staticmethod
    def resolve_config_path(config_filename):
        """Resolve full path to config file"""
        if os.path.isabs(config_filename):
            return config_filename
        
        config_dir = ConfigResolver.get_config_directory()
        config_path = os.path.join(config_dir, config_filename)
        
        if not os.path.exists(config_path):
            package_config = ConfigResolver._find_package_config(config_filename)
            if package_config:
                try:
                    shutil.copy2(package_config, config_path)
                    logger.info(f"Copied package config {package_config} to {config_path}")
                except (PermissionError, OSError) as e:
                    logger.warning(f"Could not copy config file: {e}")
                    return package_config
            else:
                ConfigResolver.create_default_config(config_path)
        
        return config_path
    
    @staticmethod
    def _find_package_config(config_filename):
        """Find config file in package data"""
        try:
            package_configs = [
                ('adore_model_checker', f'config/{config_filename}'),
                ('adore_model_checker', config_filename),
            ]
            
            for package, resource_path in package_configs:
                try:
                    if pkg_resources.resource_exists(package, resource_path):
                        config_path = pkg_resources.resource_filename(package, resource_path)
                        if os.path.exists(config_path):
                            logger.info(f"Found package config: {config_path}")
                            return config_path
                except:
                    continue
                    
        except Exception as e:
            logger.debug(f"Error finding package config: {e}")
        
        return None
    
    @staticmethod
    def create_default_config(config_path):
        """Create a default config file if it doesn't exist"""
        default_config = """monitoring:
  monitoring_frequency: 10.0
  buffer_size: 1000
  log_level: 'INFO'
  debug_mode: false

safety_parameters:
  max_speed: 30.0
  safe_distance_lateral: 1.5
  safe_distance_longitudinal: 2.0
  time_to_collision_threshold: 3.0
  emergency_brake_deceleration: -8.0
  goal_reach_distance: 5.0
  max_acceleration_rate: 2.0
  max_deceleration_rate: -3.0
  max_jerk: 1.0
  speed_limit_tolerance: 2.0
  stop_line_tolerance: 0.3

vehicles:
  - id: 0
    proposition_groups:
      basic_safety:
        enabled: true
        description: "Core safety propositions - speed, collision avoidance, goal reaching"

    propositions:
      EGO_SPEED:
        enabled: true
        atomic_prop: 'speed_safe'
        formula_type: 'always'
        threshold: 13.89
        data_sources:
          vehicle_state:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: 'vx'
            transform_function: 'abs'
            cache_duration: 0.1
        evaluation_function: 'speed_limit_evaluator'

      NEAR_GOAL:
        enabled: true
        atomic_prop: 'goal_reached'
        formula_type: 'eventually'
        threshold: 5.0
        data_sources:
          route:
            topic: '/ego_vehicle/route'
            field_path: ''
            cache_duration: 5.0
          vehicle_state:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: ''
            cache_duration: 0.1
        evaluation_function: 'near_goal_evaluator'

      DECELERATION:
        enabled: true
        atomic_prop: 'deceleration_safe'
        formula_type: 'always'
        threshold: -6.0
        data_sources:
          vehicle_state:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: 'ax'
            cache_duration: 0.1
          brake_status:
            topic: '/ego_vehicle/brake_status'
            field_path: 'active'
            cache_duration: 0.1
        evaluation_function: 'deceleration_evaluator'

      ACCELERATION_COMPLIANCE:
        enabled: true
        atomic_prop: 'acceleration_compliant'
        formula_type: 'always'
        threshold: 1.5
        data_sources:
          measured_acceleration:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: 'ax'
            cache_duration: 0.1
          commanded_acceleration:
            topic: '/ego_vehicle/next_vehicle_command'
            field_path: 'acceleration'
            cache_duration: 0.1
        evaluation_function: 'acceleration_compliance_evaluator'

      DECELERATION_COMPLIANCE:
        enabled: true
        atomic_prop: 'deceleration_compliant'
        formula_type: 'always'
        threshold: 2.0
        data_sources:
          measured_acceleration:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: 'ax'
            cache_duration: 0.1
          commanded_acceleration:
            topic: '/ego_vehicle/next_vehicle_command'
            field_path: 'acceleration'
            cache_duration: 0.1
        evaluation_function: 'deceleration_compliance_evaluator'
"""
        
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w') as f:
                f.write(default_config)
            logger.info(f"Created default config file at: {config_path}")
        except (PermissionError, OSError) as e:
            logger.error(f"Could not create default config at {config_path}: {e}")
            raise

class RunStatus(Enum):
    PENDING = "pending"
    RUNNING = "running" 
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class ModelCheckRun:
    run_id: int
    mode: str  # 'online' or 'offline'
    status: RunStatus
    config_file: str
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    duration: Optional[float] = None
    vehicle_id: Optional[int] = None
    bag_file: Optional[str] = None
    output_file: Optional[str] = None
    log_file: Optional[str] = None
    results: Optional[Dict[str, Any]] = None
    stdout: str = ""
    stderr: str = ""
    error_message: Optional[str] = None

class ModelCheckCache:
    def __init__(self, log_directory="logs"):
        self.runs = {}
        self.next_run_id = 0
        self.lock = threading.RLock()
        self.log_directory = log_directory
        
        os.makedirs(self.log_directory, exist_ok=True)
    
    def create_run(self, mode: str, config_file: str, **kwargs) -> int:
        with self.lock:
            run_id = self.next_run_id
            self.next_run_id += 1
            
            # Generate ISO8601 ZULU timestamp
            timestamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')
            
            output_file = os.path.join(
                self.log_directory, 
                f"model_check_run_{run_id}_{timestamp}_results.json"
            )
            log_file = os.path.join(
                self.log_directory,
                f"model_check_run_{run_id}_{timestamp}.log"
            )
            
            run = ModelCheckRun(
                run_id=run_id,
                mode=mode,
                status=RunStatus.PENDING,
                config_file=config_file,
                output_file=output_file,
                log_file=log_file,
                **kwargs
            )
            
            self.runs[run_id] = run
            return run_id
    
    def update_run(self, run_id: int, **kwargs):
        with self.lock:
            if run_id in self.runs:
                for key, value in kwargs.items():
                    if hasattr(self.runs[run_id], key):
                        setattr(self.runs[run_id], key, value)
    
    def get_run(self, run_id: int) -> Optional[ModelCheckRun]:
        with self.lock:
            return self.runs.get(run_id)
    
    def get_all_runs(self) -> Dict[int, ModelCheckRun]:
        with self.lock:
            return self.runs.copy()
    
    def get_running_runs(self) -> Dict[int, ModelCheckRun]:
        with self.lock:
            return {
                run_id: run for run_id, run in self.runs.items() 
                if run.status == RunStatus.RUNNING
            }

class ModelCheckWorker:
    def __init__(self, cache: ModelCheckCache):
        self.cache = cache
        self.running = False
        self.thread = None
        self.active_processes = {}
        self.process_lock = threading.RLock()
    
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()
            logger.info("ModelCheck worker thread started")
    
    def stop(self):
        self.running = False
        
        # Cancel any running processes
        with self.process_lock:
            for run_id, process in self.active_processes.items():
                try:
                    process.terminate()
                    logger.info(f"Terminated process for run {run_id}")
                except:
                    pass
        
        if self.thread:
            self.thread.join(timeout=5)
    
    def queue_online_run(self, config_file: str, duration: float = 60.0, vehicle_id: int = 0) -> int:
        resolved_config = ConfigResolver.resolve_config_path(config_file)
        
        run_id = self.cache.create_run(
            mode='online',
            config_file=resolved_config,
            duration=duration,
            vehicle_id=vehicle_id
        )
        logger.info(f"Queued online model check run {run_id} with config: {resolved_config}")
        return run_id
    
    def queue_offline_run(self, config_file: str, bag_file: str) -> int:
        resolved_config = ConfigResolver.resolve_config_path(config_file)
        
        run_id = self.cache.create_run(
            mode='offline',
            config_file=resolved_config,
            bag_file=bag_file
        )
        logger.info(f"Queued offline model check run {run_id} with config: {resolved_config}")
        return run_id
    
    def _worker_loop(self):
        while self.running:
            try:
                self._process_pending_runs()
                self._cleanup_completed_processes()
                time.sleep(1)  # Check every second
            except Exception as e:
                logger.error(f"Error in worker loop: {e}")
                time.sleep(5)
    
    def _process_pending_runs(self):
        runs = self.cache.get_all_runs()
        pending_runs = [
            run for run in runs.values() 
            if run.status == RunStatus.PENDING
        ]
        
        running_count = len(self.cache.get_running_runs())
        max_concurrent = 2  # Configurable
        
        for run in pending_runs:
            if running_count >= max_concurrent:
                break
                
            try:
                self._start_run(run)
                running_count += 1
            except Exception as e:
                logger.error(f"Failed to start run {run.run_id}: {e}")
                self.cache.update_run(
                    run.run_id,
                    status=RunStatus.FAILED,
                    error_message=str(e),
                    end_time=datetime.now(timezone.utc).isoformat()
                )
    
    def _start_run(self, run: ModelCheckRun):
        self.cache.update_run(
            run.run_id,
            status=RunStatus.RUNNING,
            start_time=datetime.now(timezone.utc).isoformat()
        )
        
        thread = threading.Thread(
            target=self._execute_run,
            args=(run,),
            daemon=True
        )
        thread.start()
        
        with self.process_lock:
            self.active_processes[run.run_id] = thread
    
    def _execute_run(self, run: ModelCheckRun):
        stdout_capture = io.StringIO()
        stderr_capture = io.StringIO()
        
        try:
            with redirect_stdout(stdout_capture), redirect_stderr(stderr_capture):
                config = ConfigLoader.load_config(run.config_file)
                analyzer = VehicleMonitorAnalyzer(config)
                
                if run.mode == 'online':
                    results = analyzer.analyze_online(
                        run.vehicle_id or 0, 
                        run.duration or 60.0
                    )
                else:  # offline
                    results = analyzer.analyze_offline(run.bag_file)
                
                with open(run.output_file, 'w') as f:
                    json.dump(results, f, indent=2, default=str)
                
                self.cache.update_run(
                    run.run_id,
                    status=RunStatus.COMPLETED,
                    end_time=datetime.now(timezone.utc).isoformat(),
                    results=results,
                    stdout=stdout_capture.getvalue(),
                    stderr=stderr_capture.getvalue()
                )
                
                logger.info(f"Completed model check run {run.run_id}")
                
        except Exception as e:
            error_msg = str(e)
            logger.error(f"Model check run {run.run_id} failed: {error_msg}")
            
            self.cache.update_run(
                run.run_id,
                status=RunStatus.FAILED,
                end_time=datetime.now(timezone.utc).isoformat(),
                error_message=error_msg,
                stdout=stdout_capture.getvalue(),
                stderr=stderr_capture.getvalue() + f"\nException: {error_msg}\n{traceback.format_exc()}"
            )
        
        finally:
            try:
                with open(run.log_file, 'w') as f:
                    f.write(f"Model Check Run {run.run_id}\n")
                    f.write(f"Mode: {run.mode}\n")
                    f.write(f"Config: {run.config_file}\n")
                    f.write(f"Start: {run.start_time}\n")
                    f.write(f"End: {run.end_time}\n")
                    f.write(f"Status: {run.status.value}\n")
                    f.write("\n--- STDOUT ---\n")
                    f.write(stdout_capture.getvalue())
                    f.write("\n--- STDERR ---\n") 
                    f.write(stderr_capture.getvalue())
            except Exception as e:
                logger.error(f"Failed to write log file for run {run.run_id}: {e}")
            
            with self.process_lock:
                if run.run_id in self.active_processes:
                    del self.active_processes[run.run_id]
    
    def _cleanup_completed_processes(self):
        with self.process_lock:
            completed_runs = []
            for run_id, thread in self.active_processes.items():
                if not thread.is_alive():
                    completed_runs.append(run_id)
            
            for run_id in completed_runs:
                del self.active_processes[run_id]

class ModelCheckAPI:
    def __init__(self, log_directory=None, default_config="default.yaml"):
        base_dir = log_directory if log_directory is not None else os.getenv("LOG_DIRECTORY", "logs")
        self.log_directory = os.path.join(base_dir, "model_checker")
        os.makedirs(self.log_directory, exist_ok=True)
        self.cache = ModelCheckCache(self.log_directory)
        self.worker = ModelCheckWorker(self.cache)
        self.default_config = default_config
        
        try:
            self.config_directory = ConfigResolver.get_config_directory()
        except Exception as e:
            logger.error(f"Failed to initialize config directory: {e}")
            fallback_dir = os.path.join(os.path.expanduser("~"), ".adore_model_checker", "config")
            os.makedirs(fallback_dir, exist_ok=True)
            self.config_directory = fallback_dir
            logger.warning(f"Using fallback config directory: {self.config_directory}")
        
        self.blueprint = self._create_blueprint()
        self.worker.start()
        
        logger.info(f"ModelCheck API initialized")
        logger.info(f"Config directory: {self.config_directory}")
        logger.info(f"Log directory: {self.log_directory}")
        logger.info(f"Default config: {self.default_config}")
    
    def _create_blueprint(self):
        bp = Blueprint('model_check_blueprint', __name__, url_prefix='/api/model_check')
        
        @bp.route('/online', methods=['POST'])
        def start_online_check():
            try:
                data = request.get_json() or {}
                
                config_file = data.get('config_file', self.default_config)
                duration = float(data.get('duration', 60.0))
                vehicle_id = int(data.get('vehicle_id', 0))
                
                run_id = self.worker.queue_online_run(
                    config_file=config_file,
                    duration=duration,
                    vehicle_id=vehicle_id
                )
                
                return jsonify({
                    'message': 'Online model check started',
                    'run_id': run_id,
                    'config_file': config_file,
                    'config_directory': self.config_directory,
                    'duration': duration,
                    'vehicle_id': vehicle_id
                })
                
            except Exception as e:
                logger.error(f"Error starting online check: {e}")
                return jsonify({'error': str(e)}), 500
        
        @bp.route('/offline', methods=['POST'])
        def start_offline_check():
            try:
                if request.is_json:
                    data = request.get_json()
                    config_file = data.get('config_file', self.default_config)
                    bag_file = data.get('bag_file')
                    
                    if not bag_file:
                        return jsonify({'error': 'bag_file required'}), 400
                    
                    if not os.path.exists(bag_file):
                        return jsonify({'error': f'Bag file not found: {bag_file}'}), 400
                        
                else:
                    config_file = request.form.get('config_file', self.default_config)
                    
                    if 'bag_file' not in request.files:
                        return jsonify({'error': 'No bag file uploaded'}), 400
                    
                    file = request.files['bag_file']
                    if file.filename == '':
                        return jsonify({'error': 'No file selected'}), 400
                    
                    filename = secure_filename(file.filename)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    bag_file = os.path.join(
                        self.cache.log_directory,
                        f"uploaded_{timestamp}_{filename}"
                    )
                    file.save(bag_file)
                
                run_id = self.worker.queue_offline_run(
                    config_file=config_file,
                    bag_file=bag_file
                )
                
                return jsonify({
                    'message': 'Offline model check started',
                    'run_id': run_id,
                    'config_file': config_file,
                    'config_directory': self.config_directory,
                    'bag_file': bag_file
                })
                
            except Exception as e:
                logger.error(f"Error starting offline check: {e}")
                return jsonify({'error': str(e)}), 500
        
        @bp.route('/config/list', methods=['GET'])
        def list_available_configs():
            """List available config files in the config directory"""
            try:
                config_files = []
                if os.path.exists(self.config_directory):
                    for file in os.listdir(self.config_directory):
                        if file.endswith(('.yaml', '.yml')):
                            config_files.append(file)
                
                return jsonify({
                    'config_directory': self.config_directory,
                    'available_configs': sorted(config_files),
                    'default_config': self.default_config
                })
                
            except Exception as e:
                logger.error(f"Error listing configs: {e}")
                return jsonify({'error': str(e)}), 500
        
        @bp.route('/result/<int:run_id>', methods=['GET'])
        def get_result(run_id):
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            result = {
                'run_id': run.run_id,
                'mode': run.mode,
                'status': run.status.value,
                'config_file': run.config_file,
                'start_time': run.start_time,
                'end_time': run.end_time,
                'stdout': run.stdout,
                'stderr': run.stderr
            }
            
            if run.mode == 'online':
                result['duration'] = run.duration
                result['vehicle_id'] = run.vehicle_id
            else:
                result['bag_file'] = run.bag_file
            
            if run.status == RunStatus.COMPLETED and run.results:
                result['results'] = run.results
            
            if run.status == RunStatus.FAILED and run.error_message:
                result['error_message'] = run.error_message
            
            result['ready'] = run.status in [RunStatus.COMPLETED, RunStatus.FAILED]
            
            return jsonify(result)
        
        @bp.route('/results', methods=['GET'])
        def get_all_results():
            runs = self.cache.get_all_runs()
            
            results = []
            for run in runs.values():
                result_summary = {
                    'run_id': run.run_id,
                    'mode': run.mode,
                    'status': run.status.value,
                    'start_time': run.start_time,
                    'end_time': run.end_time,
                    'config_file': run.config_file,
                    'ready': run.status in [RunStatus.COMPLETED, RunStatus.FAILED]
                }
                
                if run.mode == 'online':
                    result_summary['duration'] = run.duration
                    result_summary['vehicle_id'] = run.vehicle_id
                else:
                    result_summary['bag_file'] = run.bag_file
                
                if run.status == RunStatus.FAILED and run.error_message:
                    result_summary['error_message'] = run.error_message
                
                results.append(result_summary)
            
            return jsonify({
                'results': results,
                'count': len(results),
                'running': len([r for r in results if r['status'] == 'running']),
                'completed': len([r for r in results if r['status'] == 'completed']),
                'failed': len([r for r in results if r['status'] == 'failed'])
            })
        
        @bp.route('/result/<int:run_id>/download', methods=['GET'])
        def download_result(run_id):
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            if run.status != RunStatus.COMPLETED:
                return jsonify({'error': 'Run not completed'}), 400
            
            if not os.path.exists(run.output_file):
                return jsonify({'error': 'Result file not found'}), 404
            
            return send_file(
                run.output_file,
                as_attachment=True,
                download_name=f"model_check_results_{run_id}.json"
            )
        
        @bp.route('/result/<int:run_id>/log', methods=['GET'])
        def download_log(run_id):
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            if not os.path.exists(run.log_file):
                return jsonify({'error': 'Log file not found'}), 404
            
            return send_file(
                run.log_file,
                as_attachment=True,
                download_name=f"model_check_log_{run_id}.log"
            )
        
        @bp.route('/status', methods=['GET'])
        def get_status():
            runs = self.cache.get_all_runs()
            running_runs = self.cache.get_running_runs()
            
            return jsonify({
                'worker_running': self.worker.running,
                'total_runs': len(runs),
                'running_runs': len(running_runs),
                'active_processes': len(self.worker.active_processes),
                'log_directory': self.cache.log_directory,
                'config_directory': self.config_directory,
                'default_config': self.default_config
            })
        
        @bp.route('/cancel/<int:run_id>', methods=['POST'])
        def cancel_run(run_id):
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            if run.status not in [RunStatus.PENDING, RunStatus.RUNNING]:
                return jsonify({'error': 'Run cannot be cancelled'}), 400
            
            self.cache.update_run(
                run_id,
                status=RunStatus.CANCELLED,
                end_time=datetime.now(timezone.utc).isoformat()
            )
            
            with self.worker.process_lock:
                if run_id in self.worker.active_processes:
                    try:
                        thread = self.worker.active_processes[run_id]
                    except:
                        pass
            
            return jsonify({'message': f'Run {run_id} cancelled'})
        
        return bp
    
    def get_blueprint(self):
        return self.blueprint
    
    def stop_worker(self):
        self.worker.stop()

_model_check_api = None
_model_check_blueprint = None

def get_model_check_blueprint():
    global _model_check_api, _model_check_blueprint
    if _model_check_api is None:
        _model_check_api = ModelCheckAPI()
        _model_check_blueprint = _model_check_api.get_blueprint()
    return _model_check_blueprint

def stop_model_check_worker():
    global _model_check_api
    if _model_check_api is not None:
        _model_check_api.stop_worker()

def _get_api():
    global _model_check_api
    if _model_check_api is None:
        _model_check_api = ModelCheckAPI()
    return _model_check_api

@property
def model_check_api():
    return _get_api()

@property  
def model_check_blueprint():
    return get_model_check_blueprint()
