#!/usr/bin/env python3

import sys
import argparse
import os
import logging
import time
import yaml
import json
import math
import re
from typing import Optional, List, Dict, Any, Union, Tuple, Callable
from dataclasses import dataclass, field
from enum import Enum
import pandas as pd
import numpy as np
from collections import defaultdict, deque
import threading
import queue

from util.ros_marshaller import ROSMarshaller
from util.bag_file_reader import BagFileReader
from pyModelChecking import *
from pyModelChecking.CTLS import *

from util.ros_message_importer import ROSMessageImporter
ROSMessageImporter.import_all_messages()

class MonitoringMode(Enum):
    OFFLINE = "offline"
    ONLINE = "online"

class PropositionGroup(Enum):
    BASIC_SAFETY = "basic_safety"
    LANE_COMPLIANCE = "lane_compliance"
    TRAFFIC_RULES = "traffic_rules"
    DYNAMIC_SAFETY = "dynamic_safety"
    BEHAVIORAL_SMOOTHNESS = "behavioral_smoothness"
    INTERSECTION_BEHAVIOR = "intersection_behavior"
    SYSTEM_HEALTH = "system_health"
    ENVIRONMENTAL_ADAPTATION = "environmental_adaptation"
    MISSION_EFFICIENCY = "mission_efficiency"
    ADVANCED_MANEUVERS = "advanced_maneuvers"

class PropositionType(Enum):
    IN_COLLISION = ('IN_COLLISION', PropositionGroup.BASIC_SAFETY)
    NEAR_GOAL = ('NEAR_GOAL', PropositionGroup.BASIC_SAFETY)
    EGO_SPEED = ('EGO_SPEED', PropositionGroup.BASIC_SAFETY)
    SAFE_DISTANCE_X = ('SAFE_DISTANCE_X', PropositionGroup.BASIC_SAFETY)
    SAFE_DISTANCE_Y = ('SAFE_DISTANCE_Y', PropositionGroup.BASIC_SAFETY)
    DECELERATION = ('DECELERATION', PropositionGroup.BASIC_SAFETY)
    
    LANE_KEEPING = ('LANE_KEEPING', PropositionGroup.LANE_COMPLIANCE)
    LANE_CHANGE_SAFE = ('LANE_CHANGE_SAFE', PropositionGroup.LANE_COMPLIANCE)
    ROAD_BOUNDARY_RESPECT = ('ROAD_BOUNDARY_RESPECT', PropositionGroup.LANE_COMPLIANCE)
    WRONG_WAY_DRIVING = ('WRONG_WAY_DRIVING', PropositionGroup.LANE_COMPLIANCE)
    
    TRAFFIC_LIGHT_COMPLIANCE = ('TRAFFIC_LIGHT_COMPLIANCE', PropositionGroup.TRAFFIC_RULES)
    STOP_SIGN_COMPLIANCE = ('STOP_SIGN_COMPLIANCE', PropositionGroup.TRAFFIC_RULES)
    SPEED_LIMIT_COMPLIANCE = ('SPEED_LIMIT_COMPLIANCE', PropositionGroup.TRAFFIC_RULES)
    YIELD_COMPLIANCE = ('YIELD_COMPLIANCE', PropositionGroup.TRAFFIC_RULES)
    TURN_SIGNAL_USAGE = ('TURN_SIGNAL_USAGE', PropositionGroup.TRAFFIC_RULES)
    
    TIME_TO_COLLISION = ('TIME_TO_COLLISION', PropositionGroup.DYNAMIC_SAFETY)
    EMERGENCY_BRAKING = ('EMERGENCY_BRAKING', PropositionGroup.DYNAMIC_SAFETY)
    OBSTACLE_AVOIDANCE = ('OBSTACLE_AVOIDANCE', PropositionGroup.DYNAMIC_SAFETY)
    PEDESTRIAN_SAFETY = ('PEDESTRIAN_SAFETY', PropositionGroup.DYNAMIC_SAFETY)
    CYCLIST_SAFETY = ('CYCLIST_SAFETY', PropositionGroup.DYNAMIC_SAFETY)
    BLIND_SPOT_MONITORING = ('BLIND_SPOT_MONITORING', PropositionGroup.DYNAMIC_SAFETY)
    
    SMOOTH_ACCELERATION = ('SMOOTH_ACCELERATION', PropositionGroup.BEHAVIORAL_SMOOTHNESS)
    SMOOTH_STEERING = ('SMOOTH_STEERING', PropositionGroup.BEHAVIORAL_SMOOTHNESS)
    SMOOTH_BRAKING = ('SMOOTH_BRAKING', PropositionGroup.BEHAVIORAL_SMOOTHNESS)
    COMFORT_ZONE = ('COMFORT_ZONE', PropositionGroup.BEHAVIORAL_SMOOTHNESS)
    
    INTERSECTION_APPROACH = ('INTERSECTION_APPROACH', PropositionGroup.INTERSECTION_BEHAVIOR)
    RIGHT_OF_WAY = ('RIGHT_OF_WAY', PropositionGroup.INTERSECTION_BEHAVIOR)
    INTERSECTION_CLEARANCE = ('INTERSECTION_CLEARANCE', PropositionGroup.INTERSECTION_BEHAVIOR)
    TURNING_BEHAVIOR = ('TURNING_BEHAVIOR', PropositionGroup.INTERSECTION_BEHAVIOR)
    
    SENSOR_HEALTH = ('SENSOR_HEALTH', PropositionGroup.SYSTEM_HEALTH)
    LOCALIZATION_ACCURACY = ('LOCALIZATION_ACCURACY', PropositionGroup.SYSTEM_HEALTH)
    COMMUNICATION_STATUS = ('COMMUNICATION_STATUS', PropositionGroup.SYSTEM_HEALTH)
    SYSTEM_RESPONSE_TIME = ('SYSTEM_RESPONSE_TIME', PropositionGroup.SYSTEM_HEALTH)
    PATH_PLANNING_VALIDITY = ('PATH_PLANNING_VALIDITY', PropositionGroup.SYSTEM_HEALTH)
    
    WEATHER_ADAPTATION = ('WEATHER_ADAPTATION', PropositionGroup.ENVIRONMENTAL_ADAPTATION)
    VISIBILITY_ADAPTATION = ('VISIBILITY_ADAPTATION', PropositionGroup.ENVIRONMENTAL_ADAPTATION)
    ROAD_CONDITION_ADAPTATION = ('ROAD_CONDITION_ADAPTATION', PropositionGroup.ENVIRONMENTAL_ADAPTATION)
    TRAFFIC_DENSITY_ADAPTATION = ('TRAFFIC_DENSITY_ADAPTATION', PropositionGroup.ENVIRONMENTAL_ADAPTATION)
    
    ROUTE_OPTIMIZATION = ('ROUTE_OPTIMIZATION', PropositionGroup.MISSION_EFFICIENCY)
    FUEL_EFFICIENCY = ('FUEL_EFFICIENCY', PropositionGroup.MISSION_EFFICIENCY)
    TIME_EFFICIENCY = ('TIME_EFFICIENCY', PropositionGroup.MISSION_EFFICIENCY)
    PARKING_BEHAVIOR = ('PARKING_BEHAVIOR', PropositionGroup.MISSION_EFFICIENCY)
    
    OVERTAKING_SAFETY = ('OVERTAKING_SAFETY', PropositionGroup.ADVANCED_MANEUVERS)
    MERGING_BEHAVIOR = ('MERGING_BEHAVIOR', PropositionGroup.ADVANCED_MANEUVERS)
    ROUNDABOUT_BEHAVIOR = ('ROUNDABOUT_BEHAVIOR', PropositionGroup.ADVANCED_MANEUVERS)
    CONSTRUCTION_ZONE_BEHAVIOR = ('CONSTRUCTION_ZONE_BEHAVIOR', PropositionGroup.ADVANCED_MANEUVERS)
    
    def __init__(self, prop_name: str, group: PropositionGroup):
        self.prop_name = prop_name
        self.group = group

@dataclass
class DataSourceConfig:
    topic: str
    field_path: str
    data_type: Optional[str] = None
    transform_function: Optional[str] = None
    cache_duration: float = 1.0

@dataclass
class PropositionConfig:
    enabled: bool = True
    atomic_prop: str = "p"
    formula_type: str = "always"
    threshold: Optional[float] = None
    data_sources: Dict[str, DataSourceConfig] = field(default_factory=dict)
    evaluation_function: Optional[str] = None
    additional_params: Dict[str, Any] = field(default_factory=dict)

@dataclass
class GroupConfig:
    enabled: bool = False
    description: str = ""

@dataclass
class VehicleConfig:
    id: int
    proposition_groups: Dict[str, GroupConfig] = field(default_factory=dict)
    propositions: Dict[str, PropositionConfig] = field(default_factory=dict)

@dataclass
class SafetyParameters:
    max_speed: float = 30.0
    safe_distance_lateral: float = 1.5
    urban_speed_threshold: float = 15.0
    urban_safe_distance_factor: float = 3.33
    highway_safe_distance_factor: float = 2.0
    deceleration_tolerance: float = 0.1
    goal_reach_distance: float = 5.0
    
    lane_deviation_threshold: float = 0.5
    lane_change_duration_min: float = 3.0
    lane_change_signal_advance: float = 2.0
    road_boundary_buffer: float = 1.0
    
    stop_line_tolerance: float = 0.3
    traffic_light_reaction_time: float = 1.0
    yield_detection_distance: float = 50.0
    speed_limit_tolerance: float = 5.0
    
    time_to_collision_threshold: float = 3.0
    emergency_brake_deceleration: float = -8.0
    reaction_time_threshold: float = 0.5
    
    max_acceleration_rate: float = 2.0
    max_deceleration_rate: float = -3.0
    max_steering_rate: float = 45.0
    max_jerk: float = 1.0
    max_lateral_acceleration: float = 4.0
    
    intersection_approach_speed: float = 20.0
    intersection_clearance_time: float = 2.0
    right_of_way_wait_time: float = 10.0
    turning_speed_limit: float = 15.0
    
    sensor_failure_threshold: float = 0.95
    localization_error_threshold: float = 0.1
    communication_timeout: float = 1.0
    max_response_time: float = 100.0
    path_planning_update_rate: float = 10.0
    
    min_visibility_distance: float = 100.0
    wet_road_speed_reduction: float = 0.8
    heavy_traffic_following_distance: float = 3.0
    weather_speed_reduction: Dict[str, float] = field(default_factory=lambda: {
        'rain': 0.85, 'snow': 0.7, 'fog': 0.6, 'ice': 0.5
    })
    
    fuel_efficiency_threshold: float = 8.0
    route_deviation_threshold: float = 0.1
    parking_accuracy_threshold: float = 0.2
    max_detour_ratio: float = 1.2
    
    pedestrian_safety_distance: float = 3.0
    cyclist_safety_distance: float = 2.0
    school_zone_speed_limit: float = 25.0
    
    overtaking_gap_time: float = 5.0
    merging_gap_time: float = 4.0
    roundabout_entry_speed: float = 20.0
    construction_zone_speed_reduction: float = 0.7

@dataclass
class MonitoringConfig:
    monitoring_frequency: float = 10.0
    buffer_size: int = 1000
    log_level: str = "INFO"
    debug_mode: bool = False
    debug_file: str = "data/monster.yaml"
    safety_params: SafetyParameters = field(default_factory=SafetyParameters)
    vehicles: List[VehicleConfig] = field(default_factory=list)

@dataclass
class VehicleState:
    timestamp: float
    vehicle_id: int
    data: Dict[str, Any] = field(default_factory=dict)
    previous_state: Optional['VehicleState'] = None

class DataCache:
    def __init__(self, cache_duration: float = 1.0):
        self.cache_duration = cache_duration
        self._cache = {}
        self._timestamps = {}
        self._lock = threading.RLock()
    
    def get(self, key: str) -> Optional[Any]:
        with self._lock:
            if key in self._cache:
                if time.time() - self._timestamps[key] < self.cache_duration:
                    return self._cache[key]
                else:
                    del self._cache[key]
                    del self._timestamps[key]
            return None
    
    def set(self, key: str, value: Any):
        with self._lock:
            self._cache[key] = value
            self._timestamps[key] = time.time()

class DataTransforms:
    @staticmethod
    def get_nested_value(data: Dict[str, Any], field_path: str, default=None) -> Any:
        try:
            if not field_path:
                return data
            keys = field_path.split('.')
            value = data
            for key in keys:
                if isinstance(value, dict):
                    value = value.get(key, default)
                elif isinstance(value, list) and key.isdigit():
                    idx = int(key)
                    value = value[idx] if 0 <= idx < len(value) else default
                else:
                    return default
            return value if value is not None else default
        except Exception as e:
            logging.debug(f"Error in get_nested_value: {e}")
            return default
    
    @staticmethod
    def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    @staticmethod
    def calculate_speed(vx: float, vy: float) -> float:
        return math.sqrt(vx**2 + vy**2)
    
    @staticmethod
    def apply_transform(data: Any, transform_function: str) -> Any:
        try:
            if transform_function == "abs":
                return abs(float(data))
            elif transform_function == "speed_from_components":
                if isinstance(data, dict) and 'vx' in data and 'vy' in data:
                    return DataTransforms.calculate_speed(data['vx'], data['vy'])
            elif transform_function.startswith("multiply:"):
                factor = float(transform_function.split(":")[1])
                return float(data) * factor
            elif transform_function.startswith("add:"):
                offset = float(transform_function.split(":")[1])
                return float(data) + offset
            return data
        except Exception as e:
            logging.debug(f"Error in apply_transform: {e}")
            return data

class ConfigLoader:
    @staticmethod
    def load_config(config_path: str) -> MonitoringConfig:
        try:
            with open(config_path, 'r') as file:
                config_dict = yaml.safe_load(file)
            return ConfigLoader._dict_to_config(config_dict)
        except Exception as e:
            logging.error(f"Error loading config: {e}")
            raise

    @staticmethod
    def _dict_to_config(config_dict: Dict[str, Any]) -> MonitoringConfig:
        safety_params = SafetyParameters()
        if 'safety_parameters' in config_dict:
            safety_dict = config_dict['safety_parameters']
            for key, value in safety_dict.items():
                if hasattr(safety_params, key):
                    setattr(safety_params, key, value)

        vehicles = []
        if 'vehicles' in config_dict:
            for vehicle_dict in config_dict['vehicles']:
                vehicle_config = ConfigLoader._parse_vehicle_config(vehicle_dict)
                vehicles.append(vehicle_config)

        main_config = MonitoringConfig(
            safety_params=safety_params,
            vehicles=vehicles
        )

        if 'monitoring' in config_dict:
            monitoring_dict = config_dict['monitoring']
            for key, value in monitoring_dict.items():
                if hasattr(main_config, key):
                    setattr(main_config, key, value)

        return main_config

    @staticmethod
    def _parse_vehicle_config(vehicle_dict: Dict[str, Any]) -> VehicleConfig:
        vehicle_id = vehicle_dict['id']
        
        proposition_groups = {}
        if 'proposition_groups' in vehicle_dict:
            for group_name, group_config in vehicle_dict['proposition_groups'].items():
                if isinstance(group_config, bool):
                    proposition_groups[group_name] = GroupConfig(enabled=group_config)
                else:
                    proposition_groups[group_name] = GroupConfig(**group_config)

        propositions = {}
        if 'propositions' in vehicle_dict:
            for prop_name, prop_config in vehicle_dict['propositions'].items():
                data_sources = {}
                if 'data_sources' in prop_config:
                    for ds_name, ds_config in prop_config['data_sources'].items():
                        data_sources[ds_name] = DataSourceConfig(**ds_config)
                    prop_config.pop('data_sources')
                
                prop_config_obj = PropositionConfig(**prop_config)
                prop_config_obj.data_sources = data_sources
                propositions[prop_name] = prop_config_obj

        return VehicleConfig(
            id=vehicle_id,
            proposition_groups=proposition_groups,
            propositions=propositions
        )

    @staticmethod
    def create_minimal_config(output_path: str):
        config = {
            'monitoring': {
                'monitoring_frequency': 10.0,
                'buffer_size': 1000,
                'log_level': 'INFO',
                'debug_mode': False,
                'debug_file': 'data/monster.yaml'
            },
            'safety_parameters': {
                'max_speed': 30.0,
                'safe_distance_lateral': 1.5,
                'time_to_collision_threshold': 3.0,
                'goal_reach_distance': 5.0
            },
            'vehicles': [
                {
                    'id': 0,
                    'proposition_groups': {
                        'basic_safety': {
                            'enabled': True,
                            'description': 'Core safety propositions'
                        }
                    },
                    'propositions': {
                        'NEAR_GOAL': {
                            'enabled': True,
                            'atomic_prop': 'goal',
                            'formula_type': 'eventually',
                            'threshold': 5.0,
                            'data_sources': {
                                'route': {
                                    'topic': '/ego_vehicle/route',
                                    'field_path': 'goal',
                                    'cache_duration': 5.0
                                },
                                'vehicle_state': {
                                    'topic': '/ego_vehicle/vehicle_state/dynamic',
                                    'field_path': '',
                                    'cache_duration': 0.1
                                }
                            },
                            'evaluation_function': 'near_goal_evaluator'
                        },
                        'EGO_SPEED': {
                            'enabled': True,
                            'atomic_prop': 'speed',
                            'formula_type': 'always',
                            'threshold': 30.0,
                            'data_sources': {
                                'vehicle_state': {
                                    'topic': '/ego_vehicle/vehicle_state/dynamic',
                                    'field_path': 'vx',
                                    'transform_function': 'abs',
                                    'cache_duration': 0.1
                                }
                            },
                            'evaluation_function': 'speed_limit_evaluator'
                        },
                        'SAFE_DISTANCE_X': {
                            'enabled': True,
                            'atomic_prop': 'safe_x',
                            'formula_type': 'always',
                            'data_sources': {
                                'ego_state': {
                                    'topic': '/ego_vehicle/vehicle_state/dynamic',
                                    'field_path': '',
                                    'cache_duration': 0.1
                                },
                                'traffic_participants': {
                                    'topic': '/ego_vehicle/traffic_participants',
                                    'field_path': 'data',
                                    'cache_duration': 0.1
                                }
                            },
                            'evaluation_function': 'longitudinal_safety_evaluator'
                        }
                    }
                }
            ]
        }

        with open(output_path, 'w') as file:
            yaml.dump(config, file, default_flow_style=False, indent=2, sort_keys=False)

class OnlineMonitor:
    def __init__(self, config: MonitoringConfig, vehicle_config: VehicleConfig):
        self.config = config
        self.vehicle_config = vehicle_config
        self.data_buffer = queue.Queue(maxsize=config.buffer_size)
        self.state_history = deque(maxlen=100)
        self.data_cache = DataCache()
        
        self.current_state = VehicleState(
            timestamp=time.time(), 
            vehicle_id=vehicle_config.id
        )
        
        self.topic_data = {}
        self.topic_locks = {}
        self.subscription_threads = []
        
        if config.debug_mode:
            ROSMarshaller.set_debug_mode(True, config.debug_file)
        
        self._setup_subscriptions()
        
        self.monitoring_timer = threading.Timer(
            1.0/config.monitoring_frequency, 
            self._monitoring_callback
        )
        self.monitoring_timer.daemon = True
        self.monitoring_timer.start()

    def _setup_subscriptions(self):
        required_topics = self._get_required_topics()
        
        logging.info(f"Setting up subscriptions for topics: {required_topics}")
        
        for topic in required_topics:
            self.topic_data[topic] = None
            self.topic_locks[topic] = threading.RLock()
            
            callback = self._create_topic_callback(topic)
            thread = ROSMarshaller.subscribe(topic, callback)
            self.subscription_threads.append(thread)
            
            logging.info(f"Subscribed to topic: {topic}")

    def _get_required_topics(self) -> set:
        topics = set()
        enabled_propositions = self._get_enabled_propositions()
        
        for prop_config in enabled_propositions.values():
            for data_source in prop_config.data_sources.values():
                topics.add(data_source.topic)
        
        return topics

    def _get_enabled_propositions(self) -> Dict[str, PropositionConfig]:
        enabled_propositions = {}
        
        for prop_name, prop_config in self.vehicle_config.propositions.items():
            if not prop_config.enabled:
                continue
                
            try:
                prop_type = PropositionType[prop_name]
                group_name = prop_type.group.value
                
                group_config = self.vehicle_config.proposition_groups.get(group_name)
                if group_config and not group_config.enabled:
                    continue
                    
                enabled_propositions[prop_name] = prop_config
                
            except KeyError:
                logging.warning(f"Unknown proposition type: {prop_name}")
                continue
        
        return enabled_propositions

    def _create_topic_callback(self, topic: str) -> Callable:
        def callback(json_str: str, topic_name: str, datatype: str):
            try:
                data = json.loads(json_str)
                with self.topic_locks[topic]:
                    self.topic_data[topic] = {
                        'data': data,
                        'timestamp': time.time(),
                        'datatype': datatype
                    }
                    
                logging.debug(f"Received data on topic {topic}: {type(data)}")
                
            except Exception as e:
                logging.error(f"Error processing data from topic {topic}: {e}")
        
        return callback

    def _monitoring_callback(self):
        try:
            self.current_state.timestamp = time.time()
            
            if self.state_history:
                self.current_state.previous_state = self.state_history[-1]
            
            self._aggregate_data()
            
            if not self.data_buffer.full():
                import copy
                state_copy = copy.deepcopy(self.current_state)
                state_copy.previous_state = None
                self.data_buffer.put(state_copy)
                self.state_history.append(state_copy)
        
        except Exception as e:
            logging.warning(f"Error in monitoring callback: {e}")
        
        finally:
            self.monitoring_timer = threading.Timer(
                1.0/self.config.monitoring_frequency, 
                self._monitoring_callback
            )
            self.monitoring_timer.daemon = True
            self.monitoring_timer.start()

    def _aggregate_data(self):
        self.current_state.data = {}
        
        for topic, topic_lock in self.topic_locks.items():
            with topic_lock:
                topic_info = self.topic_data.get(topic)
                if topic_info:
                    self.current_state.data[topic] = topic_info['data']

    def stop(self):
        if hasattr(self, 'monitoring_timer'):
            self.monitoring_timer.cancel()
        
        for thread in self.subscription_threads:
            try:
                thread.join(timeout=1.0)
            except:
                pass

class PropositionEvaluators:
    @staticmethod
    def near_goal_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                            safety_params: SafetyParameters) -> Optional[bool]:
        try:
            route_data = data.get('route')
            vehicle_data = data.get('vehicle_state')
            
            if not route_data or not vehicle_data:
                return None
            
            goal = DataTransforms.get_nested_value(route_data, 'goal')
            ego_x = DataTransforms.get_nested_value(vehicle_data, 'x')
            ego_y = DataTransforms.get_nested_value(vehicle_data, 'y')
            
            if goal is None or ego_x is None or ego_y is None:
                return None
            
            if not isinstance(goal, dict):
                return None
                
            goal_x = goal.get('x')
            goal_y = goal.get('y')
            
            if goal_x is None or goal_y is None:
                return None
            
            distance = DataTransforms.calculate_distance(ego_x, ego_y, goal_x, goal_y)
            threshold = config.threshold or safety_params.goal_reach_distance
            
            return distance <= threshold
            
        except Exception as e:
            logging.error(f"Error in near_goal_evaluator: {e}")
            return None

    @staticmethod
    def speed_limit_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                              safety_params: SafetyParameters) -> Optional[bool]:
        try:
            speed_source = list(config.data_sources.keys())[0]
            speed_data = data.get(speed_source)
            
            if speed_data is None:
                logging.debug(f"Speed evaluator: No data for source '{speed_source}' in data keys: {list(data.keys())}")
                return True
            
            speed = float(speed_data)
            abs_speed = abs(speed)
            max_speed = config.threshold or safety_params.max_speed
            
            result = abs_speed <= max_speed
            
            logging.debug(f"Speed evaluator: speed={speed:.4f}, abs_speed={abs_speed:.4f}, threshold={max_speed:.4f}, result={result}")
            
            return result
            
        except Exception as e:
            logging.error(f"Error in speed_limit_evaluator: {e}")
            return True

    @staticmethod
    def longitudinal_safety_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                                      safety_params: SafetyParameters) -> Optional[bool]:
        try:
            ego_data = data.get('ego_state')
            traffic_data = data.get('traffic_participants')
            
            if not ego_data or not traffic_data:
                return None
            
            ego_x = DataTransforms.get_nested_value(ego_data, 'x')
            ego_vx = DataTransforms.get_nested_value(ego_data, 'vx')
            
            if ego_x is None or ego_vx is None:
                return None
            
            if not isinstance(traffic_data, list):
                return True
            
            for participant in traffic_data:
                if isinstance(participant, dict):
                    motion_state = participant.get('motion_state', {})
                    vehicle_x = motion_state.get('x')
                    
                    if vehicle_x is not None:
                        distance = abs(ego_x - vehicle_x)
                        required_distance = PropositionEvaluators._calculate_safe_distance(
                            abs(ego_vx), safety_params
                        )
                        
                        if distance < required_distance:
                            return False
            
            return True
            
        except Exception as e:
            logging.error(f"Error in longitudinal_safety_evaluator: {e}")
            return None

    @staticmethod
    def _calculate_safe_distance(speed: float, safety_params: SafetyParameters) -> float:
        if speed <= safety_params.urban_speed_threshold:
            return (speed * 3.6) / safety_params.urban_safe_distance_factor
        else:
            return (speed * 3.6) / safety_params.highway_safe_distance_factor

class ModelChecker:
    def __init__(self, config: MonitoringConfig):
        self.config = config
        self.evaluators = {
            'near_goal_evaluator': PropositionEvaluators.near_goal_evaluator,
            'speed_limit_evaluator': PropositionEvaluators.speed_limit_evaluator,
            'longitudinal_safety_evaluator': PropositionEvaluators.longitudinal_safety_evaluator,
        }



    def _collect_goal_statistics(self, state: VehicleState, prop_config: PropositionConfig, 
                                statistics: Dict[str, Any], is_final: bool = False):
        """Collect goal statistics for NEAR_GOAL proposition"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            route_data = aggregated_data.get('route')
            vehicle_data = aggregated_data.get('vehicle_state')
            
            if route_data and vehicle_data:
                goal = DataTransforms.get_nested_value(route_data, 'goal')
                ego_x = DataTransforms.get_nested_value(vehicle_data, 'x')
                ego_y = DataTransforms.get_nested_value(vehicle_data, 'y')
                
                if goal and isinstance(goal, dict) and ego_x is not None and ego_y is not None:
                    goal_x = goal.get('x')
                    goal_y = goal.get('y')
                    
                    if goal_x is not None and goal_y is not None:
                        # Set goal position (should be consistent across states)
                        if statistics['goal_position'] is None:
                            statistics['goal_position'] = {'x': goal_x, 'y': goal_y}
                        
                        distance = DataTransforms.calculate_distance(ego_x, ego_y, goal_x, goal_y)
                        statistics['min_distance_to_goal'] = min(statistics['min_distance_to_goal'], distance)
                        
                        if is_final:
                            statistics['final_vehicle_position'] = {'x': ego_x, 'y': ego_y}
                            statistics['final_distance_to_goal'] = distance
                            
        except Exception as e:
            logging.debug(f"Error collecting goal statistics: {e}")

    def _evaluate_proposition(self, state: VehicleState, 
                              prop_config: PropositionConfig,
                              prop_type: PropositionType) -> Optional[bool]:
        try:
            if prop_config.evaluation_function:
                evaluator = self.evaluators.get(prop_config.evaluation_function)
                if evaluator:
                    aggregated_data = self._aggregate_data_sources(state, prop_config)
                    return evaluator(aggregated_data, prop_config, self.config.safety_params)
            
            return self._default_evaluation(state, prop_config, prop_type)
            
        except Exception as e:
            logging.warning(f"Error evaluating proposition {prop_type}: {e}")
            return None

    def _aggregate_data_sources(self, state: VehicleState, 
                                prop_config: PropositionConfig) -> Dict[str, Any]:
        aggregated = {}
        
        for source_name, source_config in prop_config.data_sources.items():
            topic_data = state.data.get(source_config.topic)
            
            if topic_data is not None:
                if source_config.field_path:
                    value = DataTransforms.get_nested_value(topic_data, source_config.field_path)
                else:
                    value = topic_data
                
                if source_config.transform_function:
                    value = DataTransforms.apply_transform(value, source_config.transform_function)
                
                aggregated[source_name] = value
        
        return aggregated

    def _default_evaluation(self, state: VehicleState, 
                            prop_config: PropositionConfig,
                            prop_type: PropositionType) -> Optional[bool]:
        try:
            if prop_type == PropositionType.NEAR_GOAL:
                return PropositionEvaluators.near_goal_evaluator(
                    state.data, prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.EGO_SPEED:
                return PropositionEvaluators.speed_limit_evaluator(
                    state.data, prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.SAFE_DISTANCE_X:
                return PropositionEvaluators.longitudinal_safety_evaluator(
                    state.data, prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.SAFE_DISTANCE_Y:
                return self._check_lateral_safety(state, prop_config)
            
            elif prop_type == PropositionType.DECELERATION:
                return self._check_deceleration_consistency(state, prop_config)
            
            elif prop_type == PropositionType.TIME_TO_COLLISION:
                return self._check_time_to_collision(state, prop_config)
            
            elif prop_type == PropositionType.LANE_KEEPING:
                return self._check_lane_keeping(state, prop_config)
            
            elif prop_type == PropositionType.TRAFFIC_LIGHT_COMPLIANCE:
                return self._check_traffic_light_compliance(state, prop_config)
            
            elif prop_type == PropositionType.SMOOTH_ACCELERATION:
                return self._check_smooth_acceleration(state, prop_config)
            
            else:
                logging.warning(f"No evaluation method for proposition type: {prop_type}")
                return None
                
        except Exception as e:
            logging.warning(f"Error in default evaluation for {prop_type}: {e}")
            return None

    def _check_lateral_safety(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            ego_data = aggregated_data.get('ego_state')
            traffic_data = aggregated_data.get('traffic_participants')
            
            if not ego_data or not traffic_data:
                return None
            
            ego_y = DataTransforms.get_nested_value(ego_data, 'y')
            if ego_y is None:
                return None
            
            if not isinstance(traffic_data, list):
                return True
            
            for participant in traffic_data:
                if isinstance(participant, dict):
                    motion_state = participant.get('motion_state', {})
                    vehicle_y = motion_state.get('y')
                    
                    if vehicle_y is not None:
                        distance = abs(ego_y - vehicle_y)
                        if distance < self.config.safety_params.safe_distance_lateral:
                            return False
            
            return True
            
        except Exception as e:
            logging.error(f"Error in lateral safety check: {e}")
            return None

    def _check_deceleration_consistency(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            vehicle_data = aggregated_data.get('vehicle_state')
            
            if not vehicle_data:
                return None
            
            acceleration = DataTransforms.get_nested_value(vehicle_data, 'ax')
            brake_active = DataTransforms.get_nested_value(vehicle_data, 'brake_active', False)
            
            if acceleration is None:
                if state.previous_state:
                    current_vx = DataTransforms.get_nested_value(vehicle_data, 'vx')
                    prev_vx = DataTransforms.get_nested_value(state.previous_state.data, 'vehicle_state.vx')
                    
                    if current_vx is not None and prev_vx is not None:
                        dt = state.timestamp - state.previous_state.timestamp
                        if dt > 0:
                            acceleration = (current_vx - prev_vx) / dt
            
            if acceleration is None:
                return None
            
            if brake_active and acceleration > self.config.safety_params.deceleration_tolerance:
                return False
            
            return True
            
        except Exception as e:
            logging.error(f"Error in deceleration consistency check: {e}")
            return None

    def _check_time_to_collision(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            ego_data = aggregated_data.get('ego_state')
            traffic_data = aggregated_data.get('traffic_participants')
            
            if not ego_data or not traffic_data:
                return None
            
            ego_x = DataTransforms.get_nested_value(ego_data, 'x')
            ego_y = DataTransforms.get_nested_value(ego_data, 'y')
            ego_vx = DataTransforms.get_nested_value(ego_data, 'vx')
            ego_vy = DataTransforms.get_nested_value(ego_data, 'vy')
            
            if None in [ego_x, ego_y, ego_vx, ego_vy]:
                return None
            
            threshold = prop_config.threshold or self.config.safety_params.time_to_collision_threshold
            
            if not isinstance(traffic_data, list):
                return True
            
            for participant in traffic_data:
                if isinstance(participant, dict):
                    motion_state = participant.get('motion_state', {})
                    other_x = motion_state.get('x')
                    other_y = motion_state.get('y')
                    other_vx = motion_state.get('vx')
                    other_vy = motion_state.get('vy')
                    
                    if None in [other_x, other_y, other_vx, other_vy]:
                        continue
                    
                    ttc = self._calculate_time_to_collision(
                        ego_x, ego_y, ego_vx, ego_vy,
                        other_x, other_y, other_vx, other_vy
                    )
                    
                    if ttc is not None and 0 < ttc < threshold:
                        return False
            
            return True
            
        except Exception as e:
            logging.error(f"Error in time to collision check: {e}")
            return None

    def _calculate_time_to_collision(self, x1: float, y1: float, vx1: float, vy1: float,
                                     x2: float, y2: float, vx2: float, vy2: float) -> Optional[float]:
        try:
            rel_x = x2 - x1
            rel_y = y2 - y1
            rel_vx = vx2 - vx1
            rel_vy = vy2 - vy1
            
            if rel_vx == 0 and rel_vy == 0:
                return None
            
            a = rel_vx**2 + rel_vy**2
            b = 2 * (rel_x * rel_vx + rel_y * rel_vy)
            c = rel_x**2 + rel_y**2
            
            if a == 0:
                return None
            
            discriminant = b**2 - 4*a*c
            if discriminant < 0:
                return None
            
            t1 = (-b - math.sqrt(discriminant)) / (2*a)
            t2 = (-b + math.sqrt(discriminant)) / (2*a)
            
            valid_times = [t for t in [t1, t2] if t > 0]
            return min(valid_times) if valid_times else None
            
        except Exception as e:
            logging.error(f"Error calculating time to collision: {e}")
            return None

    def _check_lane_keeping(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            lane_data = aggregated_data.get('lane_info')
            vehicle_data = aggregated_data.get('vehicle_state')
            
            if not lane_data or not vehicle_data:
                return None
            
            lateral_deviation = DataTransforms.get_nested_value(lane_data, 'lateral_deviation')
            if lateral_deviation is None:
                ego_y = DataTransforms.get_nested_value(vehicle_data, 'y')
                lane_center_y = DataTransforms.get_nested_value(lane_data, 'center_y')
                
                if ego_y is not None and lane_center_y is not None:
                    lateral_deviation = abs(ego_y - lane_center_y)
            
            if lateral_deviation is None:
                return None
            
            threshold = prop_config.threshold or self.config.safety_params.lane_deviation_threshold
            return lateral_deviation <= threshold
            
        except Exception as e:
            logging.error(f"Error in lane keeping check: {e}")
            return None

    def _check_traffic_light_compliance(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            traffic_light_data = aggregated_data.get('traffic_light')
            vehicle_data = aggregated_data.get('vehicle_state')
            
            if not traffic_light_data or not vehicle_data:
                return None
            
            light_state = DataTransforms.get_nested_value(traffic_light_data, 'state')
            distance_to_light = DataTransforms.get_nested_value(traffic_light_data, 'distance')
            vehicle_speed = DataTransforms.get_nested_value(vehicle_data, 'vx')
            
            if None in [light_state, distance_to_light, vehicle_speed]:
                return None
            
            if light_state == 'red' and distance_to_light < self.config.safety_params.stop_line_tolerance:
                return abs(vehicle_speed) < 0.1
            
            if light_state == 'yellow':
                stopping_distance = (vehicle_speed**2) / (2 * 4.0)
                return distance_to_light > stopping_distance or abs(vehicle_speed) < 0.1
            
            return True
            
        except Exception as e:
            logging.error(f"Error in traffic light compliance check: {e}")
            return None

    def _check_smooth_acceleration(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            vehicle_data = aggregated_data.get('vehicle_state')
            
            if not vehicle_data:
                return None
            
            acceleration = DataTransforms.get_nested_value(vehicle_data, 'ax')
            
            if acceleration is None and state.previous_state:
                current_vx = DataTransforms.get_nested_value(vehicle_data, 'vx')
                prev_data = state.previous_state.data.get('vehicle_state', {})
                prev_vx = DataTransforms.get_nested_value(prev_data, 'vx')
                
                if current_vx is not None and prev_vx is not None:
                    dt = state.timestamp - state.previous_state.timestamp
                    if dt > 0:
                        acceleration = (current_vx - prev_vx) / dt
            
            if acceleration is None:
                return None
            
            max_accel = prop_config.threshold or self.config.safety_params.max_acceleration_rate
            return abs(acceleration) <= max_accel
            
        except Exception as e:
            logging.error(f"Error in smooth acceleration check: {e}")
            return None

    def get_formula(self, prop_config: PropositionConfig):
        atom = prop_config.atomic_prop
        
        formula_types = {
            'always': A(G(atom)),
            'always_not': A(G(Not(atom))),
            'eventually': A(F(atom)),
            'eventually_always': A(E(G(F(atom)))),
            'never': A(G(Not(atom))),
            'next': A(X(atom)),
            'until': lambda atom2: A(U(atom, atom2)),
            'weak_until': lambda atom2: A(W(atom, atom2)),
        }
        
        formula_type = prop_config.formula_type
        if formula_type in formula_types:
            formula_func = formula_types[formula_type]
            if callable(formula_func) and formula_type in ['until', 'weak_until']:
                atom2 = prop_config.additional_params.get('until_atom', 'q')
                return formula_func(atom2)
            return formula_func
        
        return A(G(atom))

    def create_kripke_from_states(self, states: List[VehicleState], 
                                  prop_config: PropositionConfig,
                                  prop_type: PropositionType) -> Tuple[Optional[Kripke], Dict[str, Any]]:
        if not states:
            return None, {}
    
        max_states = min(len(states), 1000)
        limited_states = states[:max_states]
    
        valid_states = []
        if prop_type in [PropositionType.EGO_SPEED, PropositionType.NEAR_GOAL]:
            for i, state in enumerate(limited_states):
                prop_value = self._evaluate_proposition(state, prop_config, prop_type)
                if prop_value is not None:
                    valid_states.append((i, state, prop_value))
            
            logging.info(f"Filtered {len(limited_states)} states down to {len(valid_states)} valid states for {prop_type.name}")
            
            if not valid_states:
                return None, {}
        else:
            valid_states = [(i, state, None) for i, state in enumerate(limited_states)]
    
        R = [(i, i+1) for i in range(len(valid_states)-1)]
        if valid_states:
            R.append((len(valid_states)-1, len(valid_states)-1))
        
        L = {}
    
        statistics = {}
        
        if prop_type == PropositionType.EGO_SPEED:
            statistics = {
                'max_velocity': 0.0,
                'average_velocity': 0.0,
                'speed_threshold': prop_config.threshold or self.config.safety_params.max_speed,
                'states_with_data': 0,
                'states_without_data': len(limited_states) - len(valid_states),
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0,
                'speed_values': []
            }
        elif prop_type == PropositionType.NEAR_GOAL:
            statistics = {
                'goal_position': None,
                'final_vehicle_position': None,
                'min_distance_to_goal': float('inf'),
                'final_distance_to_goal': None,
                'goal_threshold': prop_config.threshold or self.config.safety_params.goal_reach_distance,
                'states_with_data': 0,
                'states_without_data': len(limited_states) - len(valid_states),
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0
            }
    
        for kripke_state_id, (original_state_id, state, prop_value) in enumerate(valid_states):
            try:
                if prop_value is None:
                    prop_value = self._evaluate_proposition(state, prop_config, prop_type)
                
                if prop_type == PropositionType.EGO_SPEED:
                    self._collect_speed_statistics(state, prop_config, statistics)
                elif prop_type == PropositionType.NEAR_GOAL:
                    self._collect_goal_statistics(state, prop_config, statistics, 
                                                is_final=(kripke_state_id == len(valid_states) - 1))
                
                if prop_value is not None:
                    atom = prop_config.atomic_prop
                    L[kripke_state_id] = {atom} if prop_value else {Not(atom)}
                    statistics['valid_evaluations'] += 1
                    if prop_value:
                        statistics['true_evaluations'] += 1
                    else:
                        statistics['false_evaluations'] += 1
                    
                    if kripke_state_id < 5:
                        logging.info(f"  Kripke State {kripke_state_id} (orig {original_state_id}): prop_value={prop_value}, label={atom if prop_value else f'Not({atom})'}")
                        
            except Exception as e:
                logging.warning(f"Error evaluating valid state {kripke_state_id}: {e}")
                continue
    
        if prop_type == PropositionType.EGO_SPEED:
            if statistics['states_with_data'] > 0:
                statistics['average_velocity'] = statistics.get('speed_sum', 0) / statistics['states_with_data']
    
        if not L:
            return None, statistics
    
        logging.info(f"Created Kripke structure for {prop_type.name}:")
        logging.info(f"  Original states: {len(limited_states)}, Valid states: {len(valid_states)}")
        logging.info(f"  True evaluations: {statistics.get('true_evaluations', 0)}")
        logging.info(f"  False evaluations: {statistics.get('false_evaluations', 0)}")
    
        return Kripke(R=R, L=L), statistics
   
    def _collect_speed_statistics(self, state: VehicleState, prop_config: PropositionConfig, statistics: Dict[str, Any]):
        """Collect speed statistics for EGO_SPEED proposition"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            speed_source = list(prop_config.data_sources.keys())[0]
            speed_data = aggregated_data.get(speed_source)
            
            if speed_data is not None:
                speed = abs(float(speed_data))
                statistics['max_velocity'] = max(statistics.get('max_velocity', 0), speed)
                statistics['states_with_data'] += 1
                statistics['speed_values'].append(speed)
                
                if 'speed_sum' not in statistics:
                    statistics['speed_sum'] = 0
                statistics['speed_sum'] += speed
            else:
                statistics['states_without_data'] += 1
                
        except Exception as e:
            logging.debug(f"Error collecting speed statistics: {e}")
            statistics['states_without_data'] += 1
    
    def check_model(self, kripke: Kripke, formula) -> bool:
        """Check if Kripke structure satisfies the formula from initial state"""
        try:
            import sys
            original_limit = sys.getrecursionlimit()
            try:
                sys.setrecursionlimit(5000)
                
                logging.info(f"Checking model with formula: {formula}")
                labeling = kripke.labelling_function()
                logging.info(f"Kripke structure has {len(labeling)} states")
                
                for i in range(min(5, len(labeling))):
                    if i in labeling:
                        logging.info(f"  State {i}: {labeling[i]}")
                
                result = modelcheck(kripke, formula)
                
                initial_state = 0
                satisfies_formula = initial_state in result
                
                logging.info(f"Model check result: {sorted(result) if result else 'empty set'}")
                logging.info(f"Initial state {initial_state} in result: {initial_state in result}")
                logging.info(f"Formula satisfied: {satisfies_formula}")
                
                atom_name = None
                if hasattr(formula, 'formula') and hasattr(formula.formula, 'formula'):
                    atom = formula.formula.formula
                    atom_name = str(atom)
                    atom_satisfying_states = []
                    not_atom_satisfying_states = []
                    for state, labels in labeling.items():
                        if atom in labels:
                            atom_satisfying_states.append(state)
                        elif f"not {atom}" in [str(label) for label in labels]:
                            not_atom_satisfying_states.append(state)
                    logging.info(f"States satisfying '{atom}': {atom_satisfying_states[:10]}...")
                    logging.info(f"States satisfying 'not {atom}': {not_atom_satisfying_states[:10]}...")
                
                return satisfies_formula
                
            finally:
                sys.setrecursionlimit(original_limit)
        except Exception as e:
            logging.error(f"Model checking failed: {e}")
            import traceback
            logging.error(traceback.format_exc())
            return False

class VehicleMonitorAnalyzer:
    def __init__(self, config: MonitoringConfig):
        self.config = config
        self.model_checker = ModelChecker(config)
    
    def analyze_online(self, vehicle_id: int, duration: float) -> Dict[str, Any]:
        vehicle_config = next((v for v in self.config.vehicles if v.id == vehicle_id), None)
        if not vehicle_config:
            raise ValueError(f"Vehicle {vehicle_id} not found in configuration")
        
        monitor = OnlineMonitor(self.config, vehicle_config)
        
        try:
            start_time = time.time()
            logging.info(f"Starting online monitoring for vehicle {vehicle_id} for {duration} seconds...")
            
            while time.time() - start_time < duration:
                time.sleep(0.1)
            
            states = []
            while not monitor.data_buffer.empty():
                state = monitor.data_buffer.get()
                states.append(state)
            
            logging.info(f"Collected {len(states)} states for analysis")
            
            return self._analyze_states(states, vehicle_config)
            
        finally:
            monitor.stop()



    def _analyze_vehicle_offline(self, bag_file: str, vehicle_config: VehicleConfig) -> Dict[str, Any]:
        """Analyze a single vehicle from bag file data"""
        try:
            bag_reader = BagFileReader(bag_file)
            
            enabled_propositions = self._get_enabled_propositions(vehicle_config)
            required_topics = set()
            for prop_config in enabled_propositions.values():
                for data_source in prop_config.data_sources.values():
                    required_topics.add(data_source.topic)
            
            logging.info(f"Required topics: {required_topics}")
            logging.info(f"Available topics: {bag_reader.get_all_topics()}")
            
            states = self._synchronize_bag_data(bag_reader, required_topics, vehicle_config.id)
            
            logging.info(f"Created {len(states)} synchronized states from bag file")
            
            return self._analyze_states(states, vehicle_config)
            
        except Exception as e:
            logging.error(f"Error in offline bag analysis: {e}")
            return {'error': str(e)}
    
    def _synchronize_bag_data(self, bag_reader: BagFileReader, required_topics: set, vehicle_id: int) -> List[VehicleState]:
        """Synchronize data from multiple topics in bag file"""
        states = []
        
        topic_data = {}
        min_time = float('inf')
        max_time = 0
        
        for topic in required_topics:
            messages = bag_reader.get_topic_data(topic)
            if messages:
                topic_data[topic] = messages
                timestamps = [msg['timestamp'] for msg in messages]
                min_time = min(min_time, min(timestamps))
                max_time = max(max_time, max(timestamps))
        
        if not topic_data:
            logging.warning("No data found for required topics")
            return states
        
        logging.info(f"Bag data time range: {min_time:.2f} to {max_time:.2f} seconds")
        
        time_step = 0.1  # 10 Hz
        current_time = min_time
        
        while current_time <= max_time:
            state = VehicleState(timestamp=current_time, vehicle_id=vehicle_id)
            
            for topic, messages in topic_data.items():
                closest_msg = min(messages, 
                                key=lambda msg: abs(msg['timestamp'] - current_time))
                
                if abs(closest_msg['timestamp'] - current_time) < 0.5:  # 500ms tolerance
                    state.data[topic] = closest_msg
            
            if state.data: 
                states.append(state)
            
            current_time += time_step
        
        return states
    
    def _analyze_states(self, states: List[VehicleState], 
                        vehicle_config: VehicleConfig) -> Dict[str, Any]:
        results = {}
        enabled_propositions = self._get_enabled_propositions(vehicle_config)
        
        logging.info(f"Analyzing {len(states)} states against {len(enabled_propositions)} propositions")
        
        for prop_name, prop_config in enabled_propositions.items():
            try:
                logging.info(f"Starting analysis of {prop_name}")
                prop_type = PropositionType[prop_name]
                
                logging.info(f"Creating Kripke structure for {prop_name}")
                kripke, statistics = self.model_checker.create_kripke_from_states(
                    states, prop_config, prop_type
                )
                
                if kripke:
                    logging.info(f"Kripke structure created for {prop_name}, checking model")
                    formula = self.model_checker.get_formula(prop_config)
                    result = self.model_checker.check_model(kripke, formula)
                    results[prop_name] = {
                        'result': result,
                        'status': 'PASS' if result else 'FAIL',
                        'states_analyzed': len(states),
                        'kripke_states': len(kripke.labelling_function()),
                        'statistics': statistics
                    }
                    logging.info(f"{prop_name}: {'PASS' if result else 'FAIL'}")
                else:
                    logging.warning(f"Could not create Kripke structure for {prop_name}")
                    results[prop_name] = {
                        'result': None,
                        'status': 'NO_DATA',
                        'states_analyzed': 0,
                        'kripke_states': 0,
                        'statistics': statistics
                    }
                    logging.warning(f"No data for {prop_name}")
                    
            except KeyError:
                logging.error(f"Unknown proposition type: {prop_name}")
                results[prop_name] = {
                    'result': None,
                    'status': 'ERROR',
                    'error': 'Unknown proposition type',
                    'statistics': {}
                }
            except Exception as e:
                logging.error(f"Error analyzing {prop_name}: {e}")
                import traceback
                logging.error(traceback.format_exc())
                results[prop_name] = {
                    'result': None,
                    'status': 'ERROR',
                    'error': str(e),
                    'statistics': {}
                }
        
        passed = sum(1 for r in results.values() if r.get('result') is True)
        failed = sum(1 for r in results.values() if r.get('result') is False)
        total = passed + failed
        
        results['SUMMARY'] = {
            'total_propositions': len(enabled_propositions),
            'analyzed': total,
            'passed': passed,
            'failed': failed,
            'success_rate': passed / total if total > 0 else 0.0,
            'overall_result': 'PASS' if failed == 0 and passed > 0 else 'FAIL'
        }
        
        return results

    def _get_enabled_propositions(self, vehicle_config: VehicleConfig) -> Dict[str, PropositionConfig]:
        enabled_propositions = {}
        
        for prop_name, prop_config in vehicle_config.propositions.items():
            if not prop_config.enabled:
                continue
                
            try:
                prop_type = PropositionType[prop_name]
                group_name = prop_type.group.value
                
                group_config = vehicle_config.proposition_groups.get(group_name)
                if group_config and not group_config.enabled:
                    continue
                    
                enabled_propositions[prop_name] = prop_config
                
            except KeyError:
                logging.warning(f"Unknown proposition type: {prop_name}")
                continue
        
        return enabled_propositions

    def analyze_offline(self, bag_file: str) -> Dict[str, Dict[str, Any]]:
        """Analyze vehicle behavior from bag file"""
        results = {}
        
        for vehicle_config in self.config.vehicles:
            try:
                vehicle_results = self._analyze_vehicle_offline(bag_file, vehicle_config)
                results[f'vehicle_{vehicle_config.id}'] = vehicle_results
            except Exception as e:
                logging.error(f"Failed to analyze vehicle {vehicle_config.id}: {e}")
                results[f'vehicle_{vehicle_config.id}'] = {'error': str(e)}
        
        return results

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

def format_position(pos_dict):
    """Format position dictionary for display"""
    if pos_dict and isinstance(pos_dict, dict):
        x = pos_dict.get('x', 'N/A')
        y = pos_dict.get('y', 'N/A')
        if x != 'N/A' and y != 'N/A':
            return f"({x:.2f}, {y:.2f})"
    return "N/A"

if __name__ == '__main__':
    main()
