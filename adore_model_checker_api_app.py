#!/usr/bin/env python3

import sys
import os
from flask import Flask


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

def create_app():
    app = Flask(__name__)
    
    get_model_check_blueprint, _ = setup_imports()
    blueprint = get_model_check_blueprint()
    app.register_blueprint(blueprint)
    
    @app.route('/')
    def index():
        return {
            'message': 'Adore Model Checker API',
            'version': '0.1.0',
            'endpoints': {
                'model_check': '/api/model_check/',
                'status': '/api/model_check/status',
                'results': '/api/model_check/results'
            }
        }
    
    @app.route('/health')
    def health():
        return {'status': 'healthy'}
    
    return app

def main():
    app = create_app()
    
    try:
        print("Starting Adore Model Checker API server...")
        print("Available at: http://localhost:5000")
        print("API endpoints: http://localhost:5000/api/model_check/")
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        try:
            _, stop_model_check_worker = setup_imports()
            stop_model_check_worker()
        except:
            pass

if __name__ == '__main__':
    main()
