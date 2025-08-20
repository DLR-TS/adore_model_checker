#!/usr/bin/env python3

import sys
import argparse
import os
import logging
import json

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
        from model_checker import ConfigLoader
        return ROSMarshaller, BagFileReader, ROSMessageImporter, ConfigLoader, VehicleMonitorAnalyzer

ROSMarshaller, BagFileReader, ROSMessageImporter, ConfigLoader, VehicleMonitorAnalyzer = setup_imports()

def format_position(pos_dict):
    """Format position dictionary for display"""
    if pos_dict and isinstance(pos_dict, dict):
        return f"({pos_dict.get('x', 'N/A'):.2f}, {pos_dict.get('y', 'N/A'):.2f})"
    return "N/A"

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
        ConfigLoader.create_minimal_config(args.create_minimal_config)
        print(f"Minimal config created at: {args.create_minimal_config}")
        return
    
    try:
        config = ConfigLoader.load_config(args.config)
        
        log_level = logging.DEBUG if args.debug else getattr(logging, config.log_level.upper())
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        if args.mode == 'online':
            ROSMarshaller.start_metrics_reporter()
        
        analyzer = VehicleMonitorAnalyzer(config)
        
        if args.mode == 'offline':
            if not args.bag_file:
                raise ValueError("Bag file required for offline mode")
            
            if not os.path.exists(args.bag_file):
                raise FileNotFoundError(f"Bag file not found: {args.bag_file}")
            
            print(f"Starting offline analysis of bag file: {args.bag_file}")
            results = analyzer.analyze_offline(args.bag_file)
        else:
            if args.vehicle_id is None:
                raise ValueError("Vehicle ID required for online mode")
            print(f"Starting online monitoring for vehicle {args.vehicle_id} for {args.duration} seconds...")
            results = analyzer.analyze_online(args.vehicle_id, args.duration)
        
        print("=" * 60)
        print("DYNAMIC VEHICLE MONITORING RESULTS")
        print("=" * 60)
        
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
                            print(f"    {'':30}   Min TTC: {min_ttc:.3f} s, Avg TTC: {avg_ttc:.3f} m/s²")
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

                        if max_error != 'N/A':
                            print(f"    {'':30}   Max distance: {max_distance:.3f} m, Avg distance: {avg_distance:.3f} m")
                            if compliance_rate != 'N/A':
                                print(f"    {'':30}   Violations: {violations}")

        else:
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
                                events = statistics.get('max_steering_rate', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}, Violations: {violations}")

                            elif prop_name == 'LANE_KEEPING' and statistics:
                                compliance_rate = statistics.get('compliance_rate', 'N/A')
                                violations = statistics.get('compliance_violations', 0)
                                if compliance_rate != 'N/A':
                                    print(f"      {'':25}   Compliance: {compliance_rate:.1%}, Events: {events}, Violations: {violations}")

                else:
                    print(f"  Error: {vehicle_results.get('error', 'Unknown error')}")
        
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(results, f, indent=2, default=str)
            print(f"\nResults saved to: {args.output}")
                
    except Exception as e:
        logging.error(f"Analysis failed: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        sys.exit(1)
    
    finally:
        if args.mode == 'online':
            ROSMarshaller.stop()

if __name__ == '__main__':
    main()
