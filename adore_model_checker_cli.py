#!/usr/bin/env python3

import sys
import argparse
import os
import logging
import json
import math

def setup_imports():
    """Setup imports to work both when run directly and when installed as package"""
    try:
        from adore_model_checker.ros_marshaller import ROSMarshaller
        from adore_model_checker.bag_file_reader import BagFileReader
        from adore_model_checker.ros_message_importer import ROSMessageImporter
        from adore_model_checker.model_checker import ConfigLoader
        from adore_model_checker.model_checker import VehicleMonitorAnalyzer
        return ROSMarshaller, BagFileReader, ROSMessageImporter, ConfigLoader, VehicleMonitorAnalyzer
    except ImportError:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, current_dir)
        from ros_marshaller import ROSMarshaller
        from bag_file_reader import BagFileReader
        from ros_message_importer import ROSMessageImporter
        from model_checker import ConfigLoader, VehicleMonitorAnalyzer
        return ROSMarshaller, BagFileReader, ROSMessageImporter, ConfigLoader, VehicleMonitorAnalyzer

ROSMarshaller, BagFileReader, ROSMessageImporter, ConfigLoader, VehicleMonitorAnalyzer = setup_imports()

def sanitize_for_json(obj):
    """Recursively sanitize data for JSON serialization, replacing infinity and NaN values"""
    if isinstance(obj, dict):
        return {k: sanitize_for_json(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [sanitize_for_json(item) for item in obj]
    elif isinstance(obj, float):
        if math.isinf(obj) or math.isnan(obj):
            return None  # Replace infinity/NaN with null
        return obj
    return obj

def print_state(state_message):
    """Print state message in a format that can be parsed by the API"""
    print(f"[STATE]: {state_message}")
    sys.stdout.flush()

def print_progress(progress_message):
    """Print progress message that can be parsed by the API"""
    print(f"[PROGRESS]: {progress_message}")
    sys.stdout.flush()

def format_position(pos_dict):
    """Format position dictionary for display"""
    if pos_dict and isinstance(pos_dict, dict):
        return f"({pos_dict.get('x', 'N/A'):.2f}, {pos_dict.get('y', 'N/A'):.2f})"
    return "N/A"

def resolve_config_file(config_file_arg):
    """
    Resolve config file path with comprehensive fallback logic
    Returns: (resolved_path, resolution_log)
    """
    config_resolution_log = []
    resolved_config_path = None
    
    config_resolution_log.append(f"Requested config: {config_file_arg}")
    
    # First try to import ConfigResolver from the API module
    try:
        from adore_model_checker.model_checker_api import ConfigResolver
        resolved_config_path = ConfigResolver.resolve_config_path(config_file_arg)
        config_resolution_log.append(f"ConfigResolver succeeded: {resolved_config_path}")
        return resolved_config_path, config_resolution_log
        
    except ImportError as e:
        config_resolution_log.append(f"ConfigResolver import failed: {e}")
        
    except Exception as e:
        config_resolution_log.append(f"ConfigResolver failed: {e}")
    
    # Fallback config resolution logic
    
    # If absolute path provided, check if it exists
    if os.path.isabs(config_file_arg):
        config_resolution_log.append(f"Checking absolute path: {config_file_arg}")
        if os.path.exists(config_file_arg):
            resolved_config_path = config_file_arg
            config_resolution_log.append(f"Found absolute path: {resolved_config_path}")
            return resolved_config_path, config_resolution_log
        else:
            config_resolution_log.append(f"Absolute path not found: {config_file_arg}")
    
    # Extract just the filename for searching
    config_filename = os.path.basename(config_file_arg)
    config_resolution_log.append(f"Searching for filename: {config_filename}")
    
    # Try various locations for the config file
    search_paths = [
        # Original path as provided
        config_file_arg,
        # Relative to current directory
        os.path.join(os.getcwd(), config_filename),
        # Relative to script directory
        os.path.join(os.path.dirname(__file__), config_filename),
        # In config subdirectory relative to script
        os.path.join(os.path.dirname(__file__), 'config', config_filename),
        # In parent config directory
        os.path.join(os.path.dirname(__file__), '..', 'config', config_filename),
        # Two levels up (for package structure)
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', config_filename),
        # User config directory
        os.path.join(os.path.expanduser('~'), '.config', 'adore_model_checker', config_filename),
        # User local share directory
        os.path.join(os.path.expanduser('~'), '.local', 'share', 'adore_model_checker', 'config', config_filename),
        # System config directory
        os.path.join('/etc', 'adore_model_checker', config_filename)
    ]
    
    for i, path in enumerate(search_paths):
        abs_path = os.path.abspath(path)
        config_resolution_log.append(f"[{i+1}/{len(search_paths)}] Checking: {abs_path}")
        
        if os.path.exists(abs_path):
            resolved_config_path = abs_path
            config_resolution_log.append(f"Found at: {abs_path}")
            return resolved_config_path, config_resolution_log
        else:
            config_resolution_log.append(f"Not found: {abs_path}")
    
    # No config found, create a default one
    config_resolution_log.append("Creating default config file")
    
    # Choose a location to create the default config
    default_locations = [
        os.path.join(os.path.expanduser('~'), '.config', 'adore_model_checker'),
        os.path.join(os.getcwd(), 'config'),
        os.getcwd()
    ]
    
    for config_dir in default_locations:
        try:
            os.makedirs(config_dir, exist_ok=True)
            resolved_config_path = os.path.join(config_dir, config_filename)
            
            # Create the default config
            ConfigLoader.create_minimal_config(resolved_config_path)
            config_resolution_log.append(f"Created default config at: {resolved_config_path}")
            
            return resolved_config_path, config_resolution_log
            
        except (PermissionError, OSError) as e:
            config_resolution_log.append(f"Failed to create config in {config_dir}: {e}")
            continue
    
    # If we get here, we couldn't find or create a config file
    error_msg = f"Could not find or create config file '{config_file_arg}'"
    config_resolution_log.append(f"{error_msg}")
    raise FileNotFoundError(f"{error_msg}. Resolution log: {'; '.join(config_resolution_log)}")

def main():
    parser = argparse.ArgumentParser(description='Dynamic ROS Model Checker for Vehicle Monitoring')
    parser.add_argument('--mode', choices=['online', 'offline'], required=True,
                        help='Monitoring mode')
    parser.add_argument('--config', required=True,
                        help='Configuration file path')
    parser.add_argument('--bag-file', 
                        help='ROS bag file or directory for offline mode')
    parser.add_argument('--vehicle-id', type=int,
                        help='Vehicle ID to monitor (online mode)')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Monitoring duration in seconds (online mode)')
    parser.add_argument('--output', 
                        help='Output file for results')
    parser.add_argument('--create-minimal-config',
                        help='Create minimal sample config file')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.create_minimal_config:
        print_state("CREATING_CONFIG")
        ConfigLoader.create_minimal_config(args.create_minimal_config)
        print(f"Minimal config created at: {args.create_minimal_config}")
        return
    
    try:
        print_state("INITIALIZING")
        print(f"Model Checker starting in {args.mode} mode")
        
        print_state("LOADING_CONFIG")
        
        # Resolve config path with comprehensive fallback logic
        try:
            resolved_config_path, config_log = resolve_config_file(args.config)
            
        except Exception as e:
            print_state("ERROR")
            print(f"Config resolution failed: {e}")
            sys.exit(1)
        
        if not resolved_config_path or not os.path.exists(resolved_config_path):
            print_state("ERROR")
            print(f"Resolved config path is invalid: {resolved_config_path}")
            sys.exit(1)
        
        print(f"Loading configuration from: {resolved_config_path}")
        
        # Load the configuration
        try:
            config = ConfigLoader.load_config(resolved_config_path)
            print(f"Configuration loaded successfully from: {resolved_config_path}")
        except Exception as e:
            print_state("ERROR")
            print(f"Failed to load config from {resolved_config_path}: {e}")
            sys.exit(1)
        
        # Set up logging
        log_level = logging.DEBUG if args.debug else getattr(logging, config.log_level.upper())
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Start ROS services for online mode
        if args.mode == 'online':
            print_state("STARTING_ROS_SERVICES")
            ROSMarshaller.start_metrics_reporter()
        
        print_state("CREATING_ANALYZER")
        analyzer = VehicleMonitorAnalyzer(config)
        print(f"Analyzer created with {len(config.vehicles)} vehicle configurations")
        
        # Execute analysis based on mode
        if args.mode == 'offline':
            if not args.bag_file:
                raise ValueError("Bag file required for offline mode")
            
            if not os.path.exists(args.bag_file):
                raise FileNotFoundError(f"Bag file not found: {args.bag_file}")
            
            print_state("STARTING_OFFLINE_ANALYSIS")
            print(f"Starting offline analysis of bag file: {args.bag_file}")
            results = analyzer.analyze_offline(args.bag_file)
        else:
            if args.vehicle_id is None:
                raise ValueError("Vehicle ID required for online mode")
            print_state("STARTING_ONLINE_MONITORING")
            print(f"Starting online monitoring for vehicle {args.vehicle_id} for {args.duration} seconds...")
            results = analyzer.analyze_online(args.vehicle_id, args.duration)
        
        print_state("PROCESSING_RESULTS")
        print("=" * 60)
        print("DYNAMIC VEHICLE MONITORING RESULTS")
        print("=" * 60)
        
        # Display results
        if isinstance(results, dict) and 'SUMMARY' in results:
            summary = results['SUMMARY']
            print(f"\nSUMMARY:")
            print(f"  Total Propositions: {summary['total_propositions']}")
            print(f"  Analyzed: {summary['analyzed']}")
            print(f"  Passed: {summary['passed']}")
            print(f"  Failed: {summary['failed']}")
            print(f"  Success Rate: {summary['success_rate']:.1%}")
            print(f"  Overall Result: {summary['overall_result']}")
            
            print(f"\nDETAILED RESULTS:")
            print("-" * 60)
            for prop_name, prop_result in results.items():
                if prop_name != 'SUMMARY':
                    status = prop_result.get('status', 'UNKNOWN')
                    states = prop_result.get('states_analyzed', 0)
                    kripke_states = prop_result.get('kripke_states', 0)
                    statistics = prop_result.get('statistics', {})
                    
                    print(f"  {prop_name:30} : {status:8} ({states} states, {kripke_states} Kripke states)")
                    
                    if prop_name == 'EGO_SPEED' and statistics:
                        max_vel = statistics.get('max_velocity', 'N/A')
                        avg_vel = statistics.get('average_velocity', 'N/A')
                        threshold = statistics.get('speed_threshold', 'N/A')
                        states_with_data = statistics.get('states_with_data', 0)
                        states_without_data = statistics.get('states_without_data', 0)
                        valid_evals = statistics.get('valid_evaluations', 0)
                        failed_evals = statistics.get('failed_evaluations', 0)
                        
                        if max_vel != 'N/A':
                            print(f"    {'':30}   Max velocity: {max_vel:.2f} m/s, Avg velocity: {avg_vel:.2f} m/s")
                            print(f"    {'':30}   Speed threshold: {threshold:.2f} m/s")
                            print(f"    {'':30}   States with data: {states_with_data}, without data: {states_without_data}")
                            print(f"    {'':30}   Valid evaluations: {valid_evals}, failed evaluations: {failed_evals}")
                        
                    elif prop_name == 'NEAR_GOAL' and statistics:
                        goal_pos = format_position(statistics.get('goal_position'))
                        final_pos = format_position(statistics.get('final_vehicle_position'))
                        min_dist = statistics.get('min_distance_to_goal', float('inf'))
                        final_dist = statistics.get('final_distance_to_goal', 'N/A')
                        threshold = statistics.get('goal_threshold', 'N/A')
                        states_with_data = statistics.get('states_with_data', 0)
                        states_without_data = statistics.get('states_without_data', 0)
                        
                        print(f"    {'':30}   Goal position: {goal_pos}")
                        print(f"    {'':30}   Final vehicle position: {final_pos}")
                        if min_dist != float('inf'):
                            print(f"    {'':30}   Min distance to goal: {min_dist:.2f}m")
                        if final_dist != 'N/A':
                            print(f"    {'':30}   Final distance to goal: {final_dist:.2f}m")
                        if threshold != 'N/A':
                            print(f"    {'':30}   Distance threshold: {threshold:.2f}m")
                        print(f"    {'':30}   States with data: {states_with_data}, without data: {states_without_data}")

                    elif prop_name == 'DECELERATION' and statistics:
                        max_accel = statistics.get('max_acceleration', 'N/A')
                        max_decel = statistics.get('max_deceleration', 'N/A')
                        avg_accel = statistics.get('average_acceleration', 'N/A')
                        emergency_events = statistics.get('emergency_brake_events', 0)
                        brake_inconsistencies = statistics.get('brake_inconsistency_events', 0)
                        
                        if max_accel != 'N/A':
                            print(f"    {'':30}   Max acceleration: {max_accel:.2f} m/s², Max deceleration: {max_decel:.2f} m/s²")
                            print(f"    {'':30}   Average acceleration: {avg_accel:.2f} m/s²")
                            print(f"    {'':30}   Emergency brake events: {emergency_events}")
                            print(f"    {'':30}   Brake inconsistencies: {brake_inconsistencies}")

                    elif prop_name == 'ACCELERATION_COMPLIANCE' and statistics:
                        max_error = statistics.get('max_acceleration_error', 'N/A')
                        avg_error = statistics.get('average_acceleration_error', 'N/A')
                        max_measured = statistics.get('max_measured_acceleration', 'N/A')
                        max_commanded = statistics.get('max_commanded_acceleration', 'N/A')
                        avg_measured = statistics.get('average_measured_acceleration', 'N/A')
                        avg_commanded = statistics.get('average_commanded_acceleration', 'N/A')
                        violations = statistics.get('compliance_violations', 0)
                        events = statistics.get('acceleration_events', 0)
                        compliance_rate = statistics.get('compliance_rate', 'N/A')
                        
                        if max_error != 'N/A':
                            print(f"    {'':30}   Max error: {max_error:.3f} m/s², Avg error: {avg_error:.3f} m/s²")
                            print(f"    {'':30}   Max measured: {max_measured:.2f} m/s², Max commanded: {max_commanded:.2f} m/s²")
                            print(f"    {'':30}   Avg measured: {avg_measured:.2f} m/s², Avg commanded: {avg_commanded:.2f} m/s²")
                            print(f"    {'':30}   Acceleration events: {events}, Violations: {violations}")
                            if compliance_rate != 'N/A':
                                print(f"    {'':30}   Compliance rate: {compliance_rate:.1%}")

                    elif prop_name == 'DECELERATION_COMPLIANCE' and statistics:
                        max_error = statistics.get('max_deceleration_error', 'N/A')
                        avg_error = statistics.get('average_deceleration_error', 'N/A')
                        max_measured = statistics.get('max_measured_deceleration', 'N/A')
                        max_commanded = statistics.get('max_commanded_deceleration', 'N/A')
                        avg_measured = statistics.get('average_measured_deceleration', 'N/A')
                        avg_commanded = statistics.get('average_commanded_deceleration', 'N/A')
                        violations = statistics.get('compliance_violations', 0)
                        events = statistics.get('deceleration_events', 0)
                        compliance_rate = statistics.get('compliance_rate', 'N/A')
                        
                        if max_error != 'N/A':
                            print(f"    {'':30}   Max error: {max_error:.3f} m/s², Avg error: {avg_error:.3f} m/s²")
                            print(f"    {'':30}   Max measured: {max_measured:.2f} m/s², Max commanded: {max_commanded:.2f} m/s²")
                            print(f"    {'':30}   Avg measured: {avg_measured:.2f} m/s², Avg commanded: {avg_commanded:.2f} m/s²")
                            print(f"    {'':30}   Deceleration events: {events}, Violations: {violations}")
                            if compliance_rate != 'N/A':
                                print(f"    {'':30}   Compliance rate: {compliance_rate:.1%}")

                    elif prop_name == 'SMOOTH_STEERING' and statistics:
                        max_steering_rate = statistics.get('max_steering_rate', 'N/A')
                        min_steering_rate = statistics.get('min_steering_rate', 'N/A')
                        violations = statistics.get('compliance_violations', 0)
                        compliance_rate = statistics.get('compliance_rate', 'N/A')
                        
                        if max_steering_rate != 'N/A':
                            print(f"    {'':30}   Max steering rate: {max_steering_rate:.3f} , Min steering rate: {min_steering_rate:.3f} ")
                            print(f"    {'':30}   Violations: {violations}")
                            if compliance_rate != 'N/A':
                                print(f"    {'':30}   Compliance rate: {compliance_rate:.1%}")

                    elif prop_name == 'TIME_TO_COLLISION' and statistics:
                        min_ttc = statistics.get('min_ttc', 'N/A')
                        avg_ttc = statistics.get('avg_ttc', 'N/A')
                        violations = statistics.get('compliance_violations', 0)
                        compliance_rate = statistics.get('compliance_rate', 'N/A')

                        if min_ttc != 'N/A' and min_ttc != None:
                            print(f"    {'':30}   Min TTC: {min_ttc:.3f} s, Avg TTC: {avg_ttc:.3f} s")
                            print(f"    {'':30}   Violations: {violations}")
                            if compliance_rate != 'N/A':
                                print(f"    {'':30}   Compliance rate: {compliance_rate:.1%}")
                        else:
                            print(f"    {'':30}   No TTC calculatable")

                    elif prop_name == 'LANE_KEEPING' and statistics:
                        max_distance = statistics.get('max_distance', float('inf'))
                        avg_distance = statistics.get('distance_error_avg', float('inf'))
                        violations = statistics.get('compliance_violations', 0)
                        compliance_rate = statistics.get('compliance_rate', 'N/A')

                        if max_distance != float('inf'):
                            print(f"    {'':30}   Max distance: {max_distance:.3f} m, Avg distance: {avg_distance:.3f} m")
                            if compliance_rate != 'N/A':
                                print(f"    {'':30}   Violations: {violations}")
        else:
            # Handle multi-vehicle results
            for vehicle_key, vehicle_results in results.items():
                print(f"\n{vehicle_key.upper()}:")
                print("-" * 30)
                
                if 'SUMMARY' in vehicle_results:
                    summary = vehicle_results['SUMMARY']
                    print(f"  Overall: {summary['overall_result']} ({summary['passed']}/{summary['analyzed']})")
                    
                    for prop_name, prop_result in vehicle_results.items():
                        if prop_name != 'SUMMARY':
                            status = prop_result.get('status', 'UNKNOWN')
                            states = prop_result.get('states_analyzed', 0)
                            kripke_states = prop_result.get('kripke_states', 0)
                            statistics = prop_result.get('statistics', {})
                            
                            print(f"    {prop_name:25} : {status:8} ({states}/{kripke_states})")
                            
                            if prop_name == 'EGO_SPEED' and statistics:
                                max_vel = statistics.get('max_velocity', 'N/A')
                                avg_vel = statistics.get('average_velocity', 'N/A')
                                if max_vel != 'N/A':
                                    print(f"      {'':25}   Max: {max_vel:.2f} m/s, Avg: {avg_vel:.2f} m/s")
                                
                            elif prop_name == 'NEAR_GOAL' and statistics:
                                final_dist = statistics.get('final_distance_to_goal', 'N/A')
                                if final_dist != 'N/A':
                                    print(f"      {'':25}   Final distance: {final_dist:.2f}m")

                            elif prop_name == 'ACCELERATION_COMPLIANCE' and statistics:
                                compliance_rate = statistics.get('compliance_rate', 'N/A')
                                violations = statistics.get('compliance_violations', 0)
                                events = statistics.get('acceleration_events', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}, Events: {events}, Violations: {violations}")

                            elif prop_name == 'DECELERATION_COMPLIANCE' and statistics:
                                compliance_rate = statistics.get('compliance_rate', 'N/A')
                                violations = statistics.get('compliance_violations', 0)
                                events = statistics.get('deceleration_events', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}, Events: {events}, Violations: {violations}")

                            elif prop_name == 'TIME_TO_COLLISION' and statistics:
                                compliance_rate = statistics.get('compliance_rate', 'N/A')
                                violations = statistics.get('compliance_violations', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}, Violations: {violations}")

                            elif prop_name == 'SMOOTH_STEERING' and statistics:
                                compliance_rate = statistics.get('compliance_rate', 'N/A')
                                violations = statistics.get('compliance_violations', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}, Violations: {violations}")

                            elif prop_name == 'LANE_KEEPING' and statistics:
                                compliance_rate = statistics.get('compliance_rate', 'N/A')
                                violations = statistics.get('compliance_violations', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}")

                else:
                    print(f"  Error: {vehicle_results.get('error', 'Unknown error')}")
 
        print_state("SAVING_RESULTS")
        if args.output:
            # Sanitize results before saving to ensure valid JSON
            sanitized_results = sanitize_for_json(results)
            with open(args.output, 'w') as f:
                json.dump(sanitized_results, f, indent=2, default=str)
            print(f"\nResults saved to: {args.output}")
         
        print_state("FINISHED")
        print("Analysis completed successfully!")

        
        print_state("FINISHED")
        print("Analysis completed successfully!")
                
    except Exception as e:
        print_state("ERROR")
        error_msg = f"Analysis failed: {e}"
        print(f"ERROR: {error_msg}")
        logging.error(error_msg)
        if args.debug:
            import traceback
            traceback.print_exc()
        sys.exit(1)
    
    finally:
        if args.mode == 'online':
            print("Stopping ROSMarshaller threads...")
            ROSMarshaller.stop()

if __name__ == '__main__':
    main()
