#!/usr/bin/env python3

import threading
import time
import logging
import sys
import os
import re
import json
import queue
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
import math

def sanitize_infinity(obj):
    """Recursively replace infinity and NaN values with None for JSON serialization"""
    if isinstance(obj, dict):
        return {k: sanitize_infinity(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [sanitize_infinity(item) for item in obj]
    elif isinstance(obj, float):
        if math.isinf(obj) or math.isnan(obj):
            return None
        return obj
    return obj


try:
    from importlib.resources import files
    from importlib.metadata import distribution
except ImportError:
    from importlib_resources import files
    from importlib_metadata import distribution

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


def sanitize_infinity(obj):
    """Recursively replace infinity and NaN values with None for JSON serialization"""
    if isinstance(obj, dict):
        return {k: sanitize_infinity(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [sanitize_infinity(item) for item in obj]
    elif isinstance(obj, float):
        if math.isinf(obj) or math.isnan(obj):
            return None
        return obj
    return obj

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
        
        user_config_dir = Path.home() / '.config' / 'adore_model_checker'
        locations.append(str(user_config_dir))
        
        user_local_dir = Path.home() / '.local' / 'share' / 'adore_model_checker' / 'config'
        locations.append(str(user_local_dir))
        
        try:
            package_files = files('adore_model_checker')
            if package_files:
                package_config_dir = Path(str(package_files)) / 'config'
                locations.append(str(package_config_dir))
        except Exception as e:
            logger.debug(f"Could not access package config directory: {e}")
        
        system_config_dir = Path('/etc') / 'adore_model_checker'
        locations.append(str(system_config_dir))
        
        return locations

    @staticmethod
    def resolve_config_path(config_filename):
        """Resolve full path to config file"""
        if os.path.isabs(config_filename):
            if os.path.exists(config_filename):
                return config_filename
            else:
                # If absolute path doesn't exist, try to find the filename in package locations
                config_filename = os.path.basename(config_filename)
        
        # Try to find config in various locations
        search_locations = []
        
        # 1. Current working directory (for direct script execution)
        search_locations.append(os.path.join(os.getcwd(), config_filename))
        search_locations.append(os.path.join(os.getcwd(), 'config', config_filename))
        
        # 2. Relative to this module's location (development mode)
        module_dir = os.path.dirname(__file__)
        search_locations.append(os.path.join(module_dir, config_filename))
        search_locations.append(os.path.join(module_dir, 'config', config_filename))
        search_locations.append(os.path.join(module_dir, '..', 'config', config_filename))
        search_locations.append(os.path.join(module_dir, '..', '..', 'config', config_filename))
        
        # 3. Package installation directory
        try:
            # Find the package installation directory
            import adore_model_checker
            package_dir = os.path.dirname(adore_model_checker.__file__)
            search_locations.append(os.path.join(package_dir, 'config', config_filename))
            search_locations.append(os.path.join(package_dir, '..', 'config', config_filename))
            
            # Also try the site-packages location
            if 'site-packages' in package_dir:
                # For system installations like /usr/lib/python3/dist-packages/
                site_packages_parent = os.path.dirname(package_dir)
                search_locations.append(os.path.join(site_packages_parent, 'adore_model_checker', 'config', config_filename))
                
        except ImportError:
            pass
        
        # 4. System locations
        search_locations.extend([
            os.path.join('/usr', 'lib', 'python3', 'dist-packages', 'adore_model_checker', 'config', config_filename),
            os.path.join('/usr', 'local', 'lib', 'python3', 'dist-packages', 'adore_model_checker', 'config', config_filename),
            os.path.join('/opt', 'adore_model_checker', 'config', config_filename),
        ])
        
        # 5. User config directories  
        search_locations.extend([
            os.path.join(os.path.expanduser('~'), '.config', 'adore_model_checker', config_filename),
            os.path.join(os.path.expanduser('~'), '.local', 'share', 'adore_model_checker', 'config', config_filename),
        ])
        
        # Search all locations
        for location in search_locations:
            abs_location = os.path.abspath(location)
            if os.path.exists(abs_location):
                logger.info(f"Found config file: {abs_location}")
                return abs_location
        
        # If no config found, raise an error with all searched locations
        searched_locations = '\n  '.join([os.path.abspath(loc) for loc in search_locations])
        raise FileNotFoundError(
            f"Config file '{config_filename}' not found in any of the following locations:\n  {searched_locations}"
        )
    
    @staticmethod
    def _find_package_config(config_filename):
        """Find config file in package data using modern importlib.resources"""
        try:
            config_locations = [
                f'config/{config_filename}',
                config_filename,
            ]
            
            for resource_path in config_locations:
                try:
                    if '/' in resource_path:
                        parts = resource_path.split('/')
                        if len(parts) == 2:
                            subdir, filename = parts
                            package_files = files('adore_model_checker') / subdir
                            if (package_files / filename).is_file():
                                config_path = str(package_files / filename)
                                logger.info(f"Found package config: {config_path}")
                                return config_path
                    else:
                        package_files = files('adore_model_checker')
                        if (package_files / resource_path).is_file():
                            config_path = str(package_files / resource_path)
                            logger.info(f"Found package config: {config_path}")
                            return config_path
                            
                except Exception as e:
                    logger.debug(f"Error checking resource {resource_path}: {e}")
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
    mode: str
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

class ModelCheckLogger:
    """Custom logger for model checker that writes to log file in real-time"""
    
    def __init__(self, log_file: str, run_id: int):
        self.log_file = log_file
        self.run_id = run_id
        self.log_buffer = []
        self.file_handle = None
        self.lock = threading.Lock()
        
        # Initialize log file
        try:
            self.file_handle = open(log_file, 'w')
            self.write_header()
        except Exception as e:
            logger.error(f"Failed to open log file {log_file}: {e}")
    
    def write_header(self):
        """Write log file header"""
        header = f"""Model Check Run {self.run_id}
Started: {datetime.now(timezone.utc).isoformat()}
=====================================

"""
        self._write_to_file(header)
    
    def log(self, message: str, level: str = "INFO"):
        """Log a message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        formatted_message = f"[{timestamp}] [{level}] {message}\n"
        
        with self.lock:
            self.log_buffer.append(formatted_message)
            self._write_to_file(formatted_message)
    
    def log_progress(self, message: str):
        """Log progress updates"""
        self.log(message, "PROGRESS")
    
    def log_error(self, message: str):
        """Log error messages"""
        self.log(message, "ERROR")
    
    def log_result(self, message: str):
        """Log result information"""
        self.log(message, "RESULT")
    
    def _write_to_file(self, content: str):
        """Write content to log file and flush"""
        if self.file_handle:
            try:
                self.file_handle.write(content)
                self.file_handle.flush()
                os.fsync(self.file_handle.fileno())  # Force write to disk
            except Exception as e:
                logger.error(f"Error writing to log file: {e}")
    
    def get_log_content(self) -> str:
        """Get current log content"""
        with self.lock:
            return ''.join(self.log_buffer)
    
    def close(self):
        """Close log file"""
        if self.file_handle:
            try:
                self.file_handle.close()
                self.file_handle = None
            except Exception as e:
                logger.error(f"Error closing log file: {e}")

class ModelCheckCache:
    def __init__(self, log_directory="logs"):
        self.runs = {}
        self.next_run_id = 0
        self.lock = threading.RLock()
        self.log_directory = log_directory
        self.loggers = {}  # Store active loggers
        
        os.makedirs(self.log_directory, exist_ok=True)
    
    def create_run(self, mode: str, config_file: str, **kwargs) -> int:
        with self.lock:
            run_id = self.next_run_id
            self.next_run_id += 1
            
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
    
    def get_logger(self, run_id: int) -> Optional[ModelCheckLogger]:
        """Get logger for a specific run"""
        with self.lock:
            return self.loggers.get(run_id)
    
    def create_logger(self, run_id: int, log_file: str) -> ModelCheckLogger:
        """Create and store logger for a run"""
        with self.lock:
            logger_instance = ModelCheckLogger(log_file, run_id)
            self.loggers[run_id] = logger_instance
            return logger_instance
    
    def close_logger(self, run_id: int):
        """Close and remove logger for a run"""
        with self.lock:
            if run_id in self.loggers:
                self.loggers[run_id].close()
                del self.loggers[run_id]

class ModelCheckWorker:
    def __init__(self, cache: ModelCheckCache):
        self.cache = cache
        self.running = False
        self.thread = None
        self.active_processes = {}
        self.process_lock = threading.RLock()
        self.run_queue = queue.Queue()

    def start(self):
        """Start the worker thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()
            logger.info("ModelCheckWorker started")

    def stop(self):
        """Stop the worker thread"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5.0)
            
        # Clean up any active processes
        with self.process_lock:
            for run_id, process in list(self.active_processes.items()):
                try:
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=5.0)
                except:
                    pass
            self.active_processes.clear()
        logger.info("ModelCheckWorker stopped")

    def queue_online_run(self, config_file: str, duration: float, vehicle_id: int) -> int:
        """Queue an online model checking run"""
        run_id = self.cache.create_run(
            mode='online',
            config_file=config_file,
            duration=duration,
            vehicle_id=vehicle_id
        )
        
        # Put the run in the queue for processing
        self.run_queue.put(run_id)
        logger.info(f"Queued online run {run_id}")
        
        return run_id

    def queue_offline_run(self, config_file: str, bag_file: str) -> int:
        """Queue an offline model checking run"""
        run_id = self.cache.create_run(
            mode='offline',
            config_file=config_file,
            bag_file=bag_file
        )
        
        # Put the run in the queue for processing
        self.run_queue.put(run_id)
        logger.info(f"Queued offline run {run_id}")
        
        return run_id

    def _worker_loop(self):
        """Main worker thread loop that processes queued runs"""
        logger.info("ModelCheckWorker thread started")
        
        while self.running:
            try:
                # Wait for a run to process (with timeout to check self.running)
                try:
                    run_id = self.run_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Get the run details
                run = self.cache.get_run(run_id)
                if not run:
                    logger.error(f"Run {run_id} not found in cache")
                    continue
                
                # Update status to running
                self.cache.update_run(
                    run_id,
                    status=RunStatus.RUNNING,
                    start_time=datetime.now(timezone.utc).isoformat()
                )
                
                logger.info(f"Starting execution of run {run_id} ({run.mode} mode)")
                
                # Execute the run
                self._execute_run(run)
                
                # Mark task as done
                self.run_queue.task_done()
                
            except Exception as e:
                logger.error(f"Error in worker loop: {e}")
                import traceback
                logger.error(traceback.format_exc())
                
                # If we have a run_id, mark it as failed
                if 'run_id' in locals():
                    self.cache.update_run(
                        run_id,
                        status=RunStatus.FAILED,
                        end_time=datetime.now(timezone.utc).isoformat(),
                        error_message=str(e)
                    )
        
        logger.info("ModelCheckWorker thread stopped")

    def _execute_run(self, run: ModelCheckRun):
            """Execute model check run by calling the CLI tool directly"""
            model_logger = self.cache.create_logger(run.run_id, run.log_file)
            
            try:
                model_logger.log(f"Starting {run.mode} model check with run ID {run.run_id}")
                model_logger.log(f"Requested config file: {run.config_file}")
                
                try:
                    resolved_config_path = ConfigResolver.resolve_config_path(run.config_file)
                    model_logger.log(f"Resolved config path: {resolved_config_path}")
                except FileNotFoundError as e:
                    model_logger.log_error(f"Config resolution failed: {e}")
                    raise Exception(f"Could not find config file: {run.config_file}")
                except Exception as e:
                    model_logger.log_error(f"Config resolution error: {e}")
                    raise Exception(f"Config resolution failed: {e}")
                
                if run.mode == 'online':
                    model_logger.log(f"Duration: {run.duration}s")
                    model_logger.log(f"Vehicle ID: {run.vehicle_id}")
                else:
                    model_logger.log(f"Bag file: {run.bag_file}")
                
                with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as temp_output:
                    temp_output_path = temp_output.name
                
                try:
                    cmd_variants = [
                        ['python', '-m', 'adore_model_checker.adore_model_checker_cli'],
                        ['adore-model-checker'],
                        ['python', '-c', 'from adore_model_checker.adore_model_checker_cli import main; main()']
                    ]
                    
                    base_args = [
                        '--config', resolved_config_path,
                        '--mode', run.mode,
                        '--output', temp_output_path
                    ]
                    
                    if run.mode == 'online':
                        base_args.extend(['--duration', str(run.duration or 60.0)])
                        base_args.extend(['--vehicle-id', str(run.vehicle_id or 0)])
                    else:
                        base_args.extend(['--bag-file', run.bag_file])
                    
                    process = None
                    cmd_used = None
                    
                    for cmd_base in cmd_variants:
                        try:
                            full_cmd = cmd_base + base_args
                            model_logger.log(f"Trying command: {' '.join(full_cmd)}")
                            
                            process = subprocess.Popen(
                                full_cmd,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT,
                                universal_newlines=True,
                                bufsize=1
                            )
                            cmd_used = full_cmd
                            break
                        except FileNotFoundError:
                            model_logger.log(f"Command not found: {' '.join(cmd_base)}")
                            continue
                    
                    if process is None:
                        raise Exception("Could not execute model checker CLI - no valid command found")
                    
                    model_logger.log(f"Successfully started CLI with: {' '.join(cmd_used)}")
                        
                    with self.process_lock:
                        self.active_processes[run.run_id] = process
                    
                    full_output = []
                    current_state = "STARTING"
                    
                    self.cache.update_run(run.run_id, status=RunStatus.RUNNING)
                    model_logger.log("Reading CLI output...")
                    
                    for line in iter(process.stdout.readline, ''):
                        if line:
                            line = line.rstrip()
                            full_output.append(line)
                            
                            state_match = re.match(r'\[STATE\]:\s*(.+)', line)
                            progress_match = re.match(r'\[PROGRESS\]:\s*(.+)', line)
                            
                            if state_match:
                                current_state = state_match.group(1)
                                model_logger.log(f"State: {current_state}")
                            elif progress_match:
                                progress_msg = progress_match.group(1)
                                model_logger.log(f"Progress: {progress_msg}")
                            else:
                                model_logger.log(line)
                            
                            current_log = model_logger.get_log_content()
                            self.cache.update_run(run.run_id, stdout=current_log)
                    
                    return_code = process.wait()
                    model_logger.log(f"CLI process completed with return code: {return_code}")
                    
                    if return_code == 0:
                        if os.path.exists(temp_output_path):
                            try:
                                with open(temp_output_path, 'r') as f:
                                    results = json.load(f)
                                
                                model_logger.log("Results loaded successfully from output file")
                                
                                if isinstance(results, dict) and 'SUMMARY' in results:
                                    summary = results['SUMMARY']
                                    model_logger.log_result(f"Total propositions: {summary.get('total_propositions', 0)}")
                                    model_logger.log_result(f"Passed: {summary.get('passed', 0)}")
                                    model_logger.log_result(f"Failed: {summary.get('failed', 0)}")
                                    model_logger.log_result(f"Success rate: {(summary.get('success_rate', 0) * 100):.1f}%")
                                    model_logger.log_result(f"Overall result: {summary.get('overall_result', 'Unknown')}")
                                
                                try:
                                    with open(run.output_file, 'w') as f:
                                        json.dump(results, f, indent=2, default=str)
                                    model_logger.log(f"Results copied to final location: {run.output_file}")
                                except Exception as e:
                                    model_logger.log_error(f"Failed to copy results to final location: {e}")
                                
                                final_log_content = model_logger.get_log_content()
                                
                                self.cache.update_run(
                                    run.run_id,
                                    status=RunStatus.COMPLETED,
                                    end_time=datetime.now(timezone.utc).isoformat(),
                                    results=results,
                                    stdout=final_log_content,
                                    stderr=""
                                )
                                
                                logger.info(f"Completed model check run {run.run_id} with results")
                                
                            except json.JSONDecodeError as e:
                                model_logger.log_error(f"Failed to parse JSON results: {e}")
                                final_log_content = model_logger.get_log_content()
                                
                                self.cache.update_run(
                                    run.run_id,
                                    status=RunStatus.FAILED,
                                    end_time=datetime.now(timezone.utc).isoformat(),
                                    results=None,
                                    stdout=final_log_content,
                                    stderr=f"Failed to parse JSON results: {e}",
                                    error_message=f"Failed to parse JSON results: {e}"
                                )
                                raise
                            except Exception as e:
                                model_logger.log_error(f"Failed to load results: {e}")
                                final_log_content = model_logger.get_log_content()
                                
                                self.cache.update_run(
                                    run.run_id,
                                    status=RunStatus.FAILED,
                                    end_time=datetime.now(timezone.utc).isoformat(),
                                    results=None,
                                    stdout=final_log_content,
                                    stderr=f"Failed to load results: {e}",
                                    error_message=f"Failed to load results: {e}"
                                )
                                raise
                        else:
                            model_logger.log_error("No output file was created")
                            final_log_content = model_logger.get_log_content()
                            
                            self.cache.update_run(
                                run.run_id,
                                status=RunStatus.FAILED,
                                end_time=datetime.now(timezone.utc).isoformat(),
                                results=None,
                                stdout=final_log_content,
                                stderr="No output file was created",
                                error_message="No output file was created"
                            )
                            raise Exception("No output file was created")
                    else:
                        error_msg = f"CLI tool exited with code {return_code}"
                        model_logger.log_error(error_msg)
                        final_log_content = model_logger.get_log_content()
                        
                        self.cache.update_run(
                            run.run_id,
                            status=RunStatus.FAILED,
                            end_time=datetime.now(timezone.utc).isoformat(),
                            results=None,
                            stdout=final_log_content,
                            stderr=error_msg,
                            error_message=error_msg
                        )
                        raise Exception(error_msg)
                
                finally:
                    if 'temp_output_path' in locals() and os.path.exists(temp_output_path):
                        try:
                            os.unlink(temp_output_path)
                            model_logger.log(f"Cleaned up temporary file: {temp_output_path}")
                        except Exception as e:
                            model_logger.log_error(f"Failed to clean up temporary file: {e}")
                
            except Exception as e:
                error_msg = str(e)
                model_logger.log_error(f"Model check failed: {error_msg}")
                model_logger.log_error(f"Traceback: {traceback.format_exc()}")
                
                logger.error(f"Model check run {run.run_id} failed: {error_msg}")
                
                log_content = model_logger.get_log_content()
                
                self.cache.update_run(
                    run.run_id,
                    status=RunStatus.FAILED,
                    end_time=datetime.now(timezone.utc).isoformat(),
                    error_message=error_msg,
                    stdout=log_content,
                    stderr=f"Exception: {error_msg}\n{traceback.format_exc()}"
                )
            
            finally:
                self.cache.close_logger(run.run_id)
                
                with self.process_lock:
                    if run.run_id in self.active_processes:
                        del self.active_processes[run.run_id]

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
                
                # Don't resolve here - let _execute_run handle it with better error handling
                run_id = self.worker.queue_online_run(
                    config_file=config_file,  # Pass original, let execution resolve it
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

        @bp.route('/result/<int:run_id>/log/current', methods=['GET'])
        def get_current_log(run_id):
            """Get current log content for a running model check"""
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            # Try to get log from active logger first
            logger_instance = self.cache.get_logger(run_id)
            if logger_instance:
                log_content = logger_instance.get_log_content()
            else:
                # Fall back to reading from file or cached stdout
                if run.log_file and os.path.exists(run.log_file):
                    try:
                        with open(run.log_file, 'r') as f:
                            log_content = f.read()
                    except Exception as e:
                        log_content = run.stdout or f"Error reading log file: {e}"
                else:
                    log_content = run.stdout or "No log content available"
            
            return jsonify({
                'run_id': run_id,
                'status': run.status.value,
                'log_content': log_content,
                'log_file': os.path.basename(run.log_file) if run.log_file else None,
                'timestamp': datetime.now().isoformat()
            })

        @bp.route('/result/<int:run_id>', methods=['GET'])
        def get_result(run_id):
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            # Get relative paths
            log_file_relative = None
            output_file_relative = None
            
            if run.log_file:
                try:
                    log_file_relative = os.path.relpath(run.log_file, self.log_directory)
                except:
                    log_file_relative = os.path.basename(run.log_file)
            
            if run.output_file:
                try:
                    output_file_relative = os.path.relpath(run.output_file, self.log_directory)
                except:
                    output_file_relative = os.path.basename(run.output_file)
            
            result = {
                'run_id': run.run_id,
                'mode': run.mode,
                'status': run.status.value,
                'config_file': run.config_file,
                'start_time': run.start_time,
                'end_time': run.end_time,
                'stdout': run.stdout or '',
                'stderr': run.stderr or '',
                'log_file': log_file_relative,
                'output_file': output_file_relative,
                'timestamp': datetime.now().isoformat()
            }
            
            if run.mode == 'online':
                result['duration'] = run.duration
                result['vehicle_id'] = run.vehicle_id
            else:
                result['bag_file'] = run.bag_file
            
            if run.status == RunStatus.COMPLETED and run.results:
                # Sanitize results to replace infinity values with None
                result['results'] = sanitize_infinity(run.results)
            
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

        @bp.route('/debug/runs', methods=['GET'])
        def debug_all_runs():
            """Debug endpoint to see all runs"""
            runs = self.cache.get_all_runs()
            
            debug_info = {
                'total_runs': len(runs),
                'runs': {}
            }
            
            for run_id, run in runs.items():
                debug_info['runs'][run_id] = {
                    'run_id': run.run_id,
                    'status': run.status.value,
                    'mode': run.mode,
                    'start_time': run.start_time,
                    'end_time': run.end_time,
                    'has_stdout': bool(run.stdout),
                    'stdout_length': len(run.stdout) if run.stdout else 0,
                    'has_results': bool(run.results),
                    'error_message': run.error_message
                }
            
            return jsonify(debug_info)

        @bp.route('/cancel/<int:run_id>', methods=['POST'])
        def cancel_run(run_id):
            run = self.cache.get_run(run_id)
            
            if not run:
                return jsonify({'error': f'Run {run_id} not found'}), 404
            
            if run.status not in [RunStatus.PENDING, RunStatus.RUNNING]:
                return jsonify({'error': 'Run cannot be cancelled'}), 400
            
            # Try to terminate the process if it's running
            with self.worker.process_lock:
                if run_id in self.worker.active_processes:
                    try:
                        process = self.worker.active_processes[run_id]
                        if process.poll() is None:
                            process.terminate()
                            time.sleep(1)
                            if process.poll() is None:
                                process.kill()
                    except Exception as e:
                        logger.error(f"Error terminating process for run {run_id}: {e}")
            
            self.cache.update_run(
                run_id,
                status=RunStatus.CANCELLED,
                end_time=datetime.now(timezone.utc).isoformat()
            )
            
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
