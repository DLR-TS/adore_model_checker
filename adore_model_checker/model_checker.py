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

def setup_imports():
    try:
        from .ros_marshaller import ROSMarshaller
        from .bag_file_reader import BagFileReader
        from .ros_message_importer import ROSMessageImporter
        return ROSMarshaller, BagFileReader, ROSMessageImporter
    except ImportError:
        try:
            from adore_model_checker.ros_marshaller import ROSMarshaller
            from adore_model_checker.bag_file_reader import BagFileReader
            from adore_model_checker.ros_message_importer import ROSMessageImporter
            return ROSMarshaller, BagFileReader, ROSMessageImporter
        except ImportError:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            sys.path.insert(0, current_dir)
            from ros_marshaller import ROSMarshaller
            from bag_file_reader import BagFileReader
            from ros_message_importer import ROSMessageImporter
            return ROSMarshaller, BagFileReader, ROSMessageImporter

ROSMarshaller, BagFileReader, ROSMessageImporter = setup_imports()

from pyModelChecking import *
from pyModelChecking.CTLS import *

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
    SAFETY_SCORE="safety_score"

class PropositionType(Enum):
    IN_COLLISION = ('IN_COLLISION', PropositionGroup.BASIC_SAFETY)
    NEAR_GOAL = ('NEAR_GOAL', PropositionGroup.BASIC_SAFETY)
    EGO_SPEED = ('EGO_SPEED', PropositionGroup.BASIC_SAFETY)
    SAFE_DISTANCE_X = ('SAFE_DISTANCE_X', PropositionGroup.BASIC_SAFETY)
    SAFE_DISTANCE_Y = ('SAFE_DISTANCE_Y', PropositionGroup.BASIC_SAFETY)
    DECELERATION = ('DECELERATION', PropositionGroup.BASIC_SAFETY)
    SAFETY_SCORE = ('SAFETY_SCORE', PropositionGroup.SAFETY_SCORE)
    
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
    ACCELERATION_COMPLIANCE = ('ACCELERATION_COMPLIANCE', PropositionGroup.DYNAMIC_SAFETY)
    DECELERATION_COMPLIANCE = ('DECELERATION_COMPLIANCE', PropositionGroup.DYNAMIC_SAFETY)
    
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

    def get_human_description(self) -> Dict[str, str]:
        """Get human-comprehensible description for proposition"""
        descriptions = {
            # Basic Safety Propositions
            'IN_COLLISION': {
                'title': 'Collision Avoidance',
                'description': 'Ensures the vehicle never collides with other objects, vehicles, or obstacles',
                'safety_rationale': 'Preventing collisions is the most fundamental safety requirement for autonomous vehicles'
            },
            'NEAR_GOAL': {
                'title': 'Goal Achievement',
                'description': 'Verifies that the vehicle successfully reaches its intended destination within acceptable distance',
                'safety_rationale': 'Confirms the vehicle can complete its mission without getting lost or stopping prematurely'
            },
            'EGO_SPEED': {
                'title': 'Speed Limit Compliance',
                'description': 'Ensures the vehicle maintains speed within safe and legal limits at all times',
                'safety_rationale': 'Excessive speed is a primary factor in accident severity and reduces reaction time'
            },
            'SAFE_DISTANCE_X': {
                'title': 'Safe Following Distance',
                'description': 'Maintains adequate longitudinal distance from vehicles ahead to allow safe stopping',
                'safety_rationale': 'Following too closely (tailgating) is a leading cause of rear-end collisions'
            },
            'SAFE_DISTANCE_Y': {
                'title': 'Safe Lateral Clearance',
                'description': 'Ensures sufficient side-to-side spacing from adjacent vehicles and obstacles',
                'safety_rationale': 'Adequate lateral spacing prevents side-swipe accidents and allows emergency maneuvering'
            },
            'DECELERATION': {
                'title': 'Safe Braking Behavior',
                'description': 'Monitors that braking forces remain within safe limits to prevent loss of control',
                'safety_rationale': 'Excessive braking can cause skidding, loss of control, or rear-end collisions from following vehicles'
            },
            'SAFETY_SCORE': {
                'title': 'Safety Score based on Speed Limit',
                'description': 'Quantified Score for Safety based on the speed of the vehicle compared to the speed limit',
                'safety_rationale': 'Exceeding the speed limit is tolerated to a certain threshold, as this exceeded value increases, the safety should decrease within a tolerable level'
            },

            # Lane Compliance Propositions
            'LANE_KEEPING': {
                'title': 'Lane Maintenance',
                'description': 'Ensures the vehicle stays within its designated lane boundaries',
                'safety_rationale': 'Lane departures can lead to head-on collisions or run-off-road accidents'
            },
            'LANE_CHANGE_SAFE': {
                'title': 'Safe Lane Changes',
                'description': 'Verifies lane changes are performed safely with adequate gaps and signaling',
                'safety_rationale': 'Improper lane changes are a major cause of side-swipe and multi-vehicle accidents'
            },
            'ROAD_BOUNDARY_RESPECT': {
                'title': 'Road Boundary Adherence',
                'description': 'Ensures the vehicle remains within the roadway and doesnt cross into shoulders or off-road areas',
                'safety_rationale': 'Leaving the roadway can result in rollovers, collisions with fixed objects, or getting stuck'
            },
            'WRONG_WAY_DRIVING': {
                'title': 'Correct Direction Travel',
                'description': 'Prevents the vehicle from traveling in the wrong direction on roadways',
                'safety_rationale': 'Wrong-way driving creates extremely high risk of fatal head-on collisions'
            },

            # Traffic Rules Propositions
            'TRAFFIC_LIGHT_COMPLIANCE': {
                'title': 'Traffic Signal Obedience',
                'description': 'Ensures the vehicle properly responds to traffic lights (stop on red, proceed on green)',
                'safety_rationale': 'Running red lights causes high-speed intersection crashes with severe injuries'
            },
            'STOP_SIGN_COMPLIANCE': {
                'title': 'Stop Sign Adherence',
                'description': 'Verifies the vehicle comes to complete stops at stop signs and yields right-of-way',
                'safety_rationale': 'Failure to stop at intersections leads to T-bone and angle crashes'
            },
            'SPEED_LIMIT_COMPLIANCE': {
                'title': 'Legal Speed Limits',
                'description': 'Ensures the vehicle operates within posted speed limits and speed zones',
                'safety_rationale': 'Speeding reduces reaction time and increases accident severity and likelihood'
            },
            'YIELD_COMPLIANCE': {
                'title': 'Right-of-Way Yielding',
                'description': 'Confirms the vehicle properly yields to other vehicles, pedestrians, and cyclists when required',
                'safety_rationale': 'Failure to yield causes intersection crashes and pedestrian/cyclist injuries'
            },
            'TURN_SIGNAL_USAGE': {
                'title': 'Turn Signal Communication',
                'description': 'Ensures the vehicle uses turn signals to communicate intended maneuvers to other drivers',
                'safety_rationale': 'Proper signaling prevents confusion and allows other drivers to react appropriately'
            },

            # Dynamic Safety Propositions
            'TIME_TO_COLLISION': {
                'title': 'Collision Time Prediction',
                'description': 'Monitors predicted time until collision and ensures it remains above safe thresholds',
                'safety_rationale': 'Early collision detection allows time for evasive action or emergency braking'
            },
            'EMERGENCY_BRAKING': {
                'title': 'Emergency Braking Capability',
                'description': 'Verifies the vehicle can perform emergency stops when collision threats are detected',
                'safety_rationale': 'Emergency braking is the last line of defense to prevent or mitigate collisions'
            },
            'OBSTACLE_AVOIDANCE': {
                'title': 'Obstacle Detection and Avoidance',
                'description': 'Ensures the vehicle can detect and safely navigate around unexpected obstacles',
                'safety_rationale': 'Roads contain unpredictable obstacles like debris, animals, or stopped vehicles'
            },
            'PEDESTRIAN_SAFETY': {
                'title': 'Pedestrian Protection',
                'description': 'Maintains safe distances and speeds around pedestrians, especially in crosswalks',
                'safety_rationale': 'Pedestrians are vulnerable road users who can suffer fatal injuries in vehicle collisions'
            },
            'CYCLIST_SAFETY': {
                'title': 'Cyclist Protection',
                'description': 'Ensures safe interactions with bicycles, including adequate passing clearance',
                'safety_rationale': 'Cyclists are vulnerable and difficult to see, requiring special safety considerations'
            },
            'BLIND_SPOT_MONITORING': {
                'title': 'Blind Spot Awareness',
                'description': 'Monitors areas around the vehicle that may not be visible to sensors or cameras',
                'safety_rationale': 'Blind spots hide other vehicles and can cause accidents during lane changes or turns'
            },
            'ACCELERATION_COMPLIANCE': {
                'title': 'Acceleration Command Following',
                'description': 'Verifies the vehicle accurately follows commanded acceleration inputs from the planning system',
                'safety_rationale': 'Inconsistent acceleration response can lead to unpredictable behavior and loss of control'
            },
            'DECELERATION_COMPLIANCE': {
                'title': 'Deceleration Command Following',
                'description': 'Ensures the vehicle accurately follows commanded deceleration inputs from the planning system',
                'safety_rationale': 'Inconsistent braking response compromises stopping ability and collision avoidance'
            },

            # Behavioral Smoothness Propositions
            'SMOOTH_ACCELERATION': {
                'title': 'Comfortable Acceleration',
                'description': 'Ensures acceleration changes are gradual and comfortable for passengers',
                'safety_rationale': 'Sudden acceleration can cause passenger injury and reduce vehicle stability'
            },
            'SMOOTH_STEERING': {
                'title': 'Stable Steering Control',
                'description': 'Monitors steering inputs to ensure smooth, stable vehicle control without erratic movements',
                'safety_rationale': 'Erratic steering can cause loss of control, especially at higher speeds'
            },
            'SMOOTH_BRAKING': {
                'title': 'Progressive Braking',
                'description': 'Ensures braking is applied progressively rather than abruptly when not in emergency situations',
                'safety_rationale': 'Smooth braking prevents passenger injury and maintains vehicle stability'
            },
            'COMFORT_ZONE': {
                'title': 'Passenger Comfort Limits',
                'description': 'Maintains vehicle dynamics within comfortable limits for passengers (acceleration, turning forces)',
                'safety_rationale': 'Excessive g-forces can cause passenger discomfort, motion sickness, or injury'
            },

            # System Health Propositions
            'SENSOR_HEALTH': {
                'title': 'Sensor System Integrity',
                'description': 'Monitors the health and functionality of critical sensors (cameras, lidar, radar)',
                'safety_rationale': 'Sensor failures can blind the vehicle to hazards and prevent safe operation'
            },
            'LOCALIZATION_ACCURACY': {
                'title': 'Position Accuracy',
                'description': 'Ensures the vehicle accurately knows its position on the road and relative to other objects',
                'safety_rationale': 'Poor localization can cause the vehicle to drift into other lanes or miss navigation points'
            },
            'COMMUNICATION_STATUS': {
                'title': 'Vehicle Communication Health',
                'description': 'Monitors communication links with other vehicles, infrastructure, and remote operators',
                'safety_rationale': 'Communication failures can prevent coordination with other vehicles and infrastructure'
            },
            'SYSTEM_RESPONSE_TIME': {
                'title': 'System Reaction Speed',
                'description': 'Ensures the vehicle processes information and responds to hazards within acceptable time limits',
                'safety_rationale': 'Slow system response reduces the ability to avoid accidents and react to emergencies'
            },
            'PATH_PLANNING_VALIDITY': {
                'title': 'Path Planning Integrity',
                'description': 'Verifies that planned vehicle paths are safe, feasible, and collision-free',
                'safety_rationale': 'Invalid path plans can lead the vehicle into dangerous situations or collisions'
            }
        }

        return descriptions.get(self.prop_name, {
            'title': self.prop_name.replace('_', ' ').title(),
            'description': f'Safety monitoring for {self.prop_name.lower().replace("_", " ")}',
            'safety_rationale': 'Critical for safe autonomous vehicle operation'
        })

    def get_formula_description(self, formula_type: str) -> str:
        """Get human-readable description of the temporal logic formula"""
        formula_descriptions = {
            'always': 'This condition must hold true throughout the entire journey',
            'eventually': 'This condition must become true at some point during the journey',
            'never': 'This condition must never become true at any point',
            'next': 'This condition must be true in the next time step',
            'until': 'The first condition must hold until the second condition becomes true',
            'weak_until': 'The first condition must hold until the second condition becomes true, or forever if the second never occurs'
        }
        return formula_descriptions.get(formula_type, f'Custom temporal logic formula: {formula_type}')

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

    higher_speed_limit_tolerance: float = 7.0
    speed_limit_tolerance_safety_score: float = 0.3
    higher_speed_limit_tolerance_safety_score: float = 0.15
    speed_limit_tolerance_threshold: float = 0.5
    higher_speed_limit_tolerance_threshold: int = 5
    speed_safety_score_threshold:  float = 0.4
    speed_safety_score_grade_format: str = 'american'

    lower_lane_tolerance: float = 0.3
    upper_lane_tolerance: float = 0.7
    lower_lane_tolerance_safety_score: float = 0.7
    lane_tolerance_safety_score: float = 0.3
    upper_lane_tolerance_safety_score: float = 0.2
    lane_tolerance_threshold: float = 0.5
    upper_lane_tolerance_threshold: int = 3
    lane_safety_score_threshold: float = 0.4
    lane_safety_score_grade_format: str = 'american'
    
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
    def closest_point_on_segment_2d(x: float, y: float, a: Dict[str, Any], b: Dict[str, Any]) -> Dict[str, Any]:
        ab_x = b['x'] - a['x']
        ab_y = b['y'] - a['y']

        ap_x = x - a['x']
        ap_y = y - a['y']

        ab_len_sq = ab_x**2 + ab_y**2
        if ab_len_sq == 0:
            return a  # A and B are the same point

        t = (ap_x * ab_x + ap_y * ab_y) / ab_len_sq

        if t < 0.0:
            return a
        elif t > 1.0:
            return b
        else:
            closest: Dict[str, Any] = {}
            closest['x'] = a['x'] + t * ab_x
            closest['y'] = a['y'] + t * ab_y
            return closest

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
    def deceleration_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                              safety_params: SafetyParameters) -> Optional[bool]:
        try:
            vehicle_data = data.get('vehicle_state')
            brake_data = data.get('brake_status')
            
            if vehicle_data is None:
                return None
            
            acceleration = float(vehicle_data)
            brake_active = brake_data if brake_data is not None else False
            
            max_safe_deceleration = config.threshold or safety_params.emergency_brake_deceleration
            
            if acceleration < 0:
                is_safe_deceleration = acceleration >= max_safe_deceleration
            else:
                is_safe_deceleration = True
            
            if brake_active and acceleration > -0.5:
                return False
            
            return is_safe_deceleration
            
        except Exception as e:
            logging.error(f"Error in deceleration_evaluator: {e}")
            return None

    @staticmethod
    def acceleration_compliance_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                                        safety_params: SafetyParameters) -> Optional[bool]:
        try:
            measured_data = data.get('measured_acceleration')
            commanded_data = data.get('commanded_acceleration')
            
            if measured_data is None or commanded_data is None:
                return None
            
            measured_accel = float(measured_data)
            commanded_accel = float(commanded_data)
            
            if commanded_accel <= 0.1:
                return True
            
            difference = abs(measured_accel - commanded_accel)
            threshold = config.threshold or 1.5
            
            return difference <= threshold
            
        except Exception as e:
            logging.error(f"Error in acceleration_compliance_evaluator: {e}")
            return None

    @staticmethod
    def deceleration_compliance_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                                        safety_params: SafetyParameters) -> Optional[bool]:
        try:
            measured_data = data.get('measured_acceleration')
            commanded_data = data.get('commanded_acceleration')
            
            if measured_data is None or commanded_data is None:
                return None
            
            measured_accel = float(measured_data)
            commanded_accel = float(commanded_data)
            
            if commanded_accel >= -0.1:
                return True
            
            difference = abs(measured_accel - commanded_accel)
            threshold = config.threshold or 2.0
            
            return difference <= threshold
            
        except Exception as e:
            logging.error(f"Error in deceleration_compliance_evaluator: {e}")
            return None

#    @staticmethod
#    def steering_rate_compliance_evaluator(data: Dict[str, Any], config: PropositionConfig, 
#                                        safety_params: SafetyParameters) -> Optional[bool]:
#        try:
#            measured_data = data.get('steering_rate')
#            
#            if measured_data is None:
#                return None
#            
#            measured_steering_rate = float(measured_data)
#            threshold = config.threshold
#            
#            return measured_steering_rate <= threshold
#            
#        except Exception as e:
#            logging.error(f"Error in steering_rate_compliance_evaluator: {e}")
#            return None

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
            'deceleration_evaluator': PropositionEvaluators.deceleration_evaluator,
            'acceleration_compliance_evaluator': PropositionEvaluators.acceleration_compliance_evaluator,
            'deceleration_compliance_evaluator': PropositionEvaluators.deceleration_compliance_evaluator,
#            'smooth_steering_evaluator': PropositionEvaluators.steering_rate_compliance_evaluator,
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
                        if statistics['goal_position'] is None:
                            statistics['goal_position'] = {'x': goal_x, 'y': goal_y}
                        
                        distance = DataTransforms.calculate_distance(ego_x, ego_y, goal_x, goal_y)
                        statistics['min_distance_to_goal'] = min(statistics['min_distance_to_goal'], distance)
                        
                        if is_final:
                            statistics['final_vehicle_position'] = {'x': ego_x, 'y': ego_y}
                            statistics['final_distance_to_goal'] = distance
                            
        except Exception as e:
            logging.debug(f"Error collecting goal statistics: {e}")

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

                speed_offset = float(speed_data) - self.config.safety_params.max_speed
                safety_score = self._get_speed_safety_score(speed_offset)
                
                statistics['safety_scores'].append(safety_score)
                statistics['average_safety_score'] = self._get_guarded_speed_safety_score(statistics['safety_scores'], statistics['speed_values']) 
                statistics['safety_grade'], statistics['safety_grade_format'] = self.get_grade_from_safety_score(statistics['average_safety_score'],
                                                                                        self.config.safety_params.speed_safety_score_grade_format)
            else:
                statistics['states_without_data'] += 1
                
        except Exception as e:
            logging.debug(f"Error collecting speed statistics: {e}")
            statistics['states_without_data'] += 1

    def _collect_deceleration_statistics(self, state: VehicleState, prop_config: PropositionConfig, 
                                       statistics: Dict[str, Any]):
        """Collect deceleration statistics for DECELERATION proposition"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            accel_data = aggregated_data.get('vehicle_state')
            brake_data = aggregated_data.get('brake_status')
            
            if accel_data is not None:
                acceleration = float(accel_data)
                
                statistics['max_acceleration'] = max(statistics.get('max_acceleration', float('-inf')), acceleration)
                statistics['min_acceleration'] = min(statistics.get('min_acceleration', float('inf')), acceleration)
                
                if acceleration >= 0:
                    statistics['max_positive_acceleration'] = max(
                        statistics.get('max_positive_acceleration', 0), acceleration
                    )
                    statistics['acceleration_events'] = statistics.get('acceleration_events', 0) + 1
                else:
                    statistics['max_deceleration'] = min(
                        statistics.get('max_deceleration', 0), acceleration
                    )
                    statistics['deceleration_events'] = statistics.get('deceleration_events', 0) + 1
                
                emergency_threshold = statistics.get('emergency_threshold', -6.0)
                if acceleration < emergency_threshold:
                    statistics['emergency_brake_events'] = statistics.get('emergency_brake_events', 0) + 1
                
                if brake_data:
                    if brake_data and acceleration > -0.5:
                        statistics['brake_inconsistency_events'] = statistics.get('brake_inconsistency_events', 0) + 1
                
                statistics['acceleration_values'].append(acceleration)
                statistics['states_with_data'] += 1
                
                if 'accel_sum' not in statistics:
                    statistics['accel_sum'] = 0
                statistics['accel_sum'] += acceleration
                
        except Exception as e:
            logging.debug(f"Error collecting deceleration statistics: {e}")
            statistics['states_without_data'] += 1

    def _collect_acceleration_compliance_statistics(self, state: VehicleState, prop_config: PropositionConfig, 
                                                  statistics: Dict[str, Any]):
        """Collect acceleration compliance statistics"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            measured_data = aggregated_data.get('measured_acceleration')
            commanded_data = aggregated_data.get('commanded_acceleration')
            
            if measured_data is not None and commanded_data is not None:
                measured_val = float(measured_data)
                commanded_val = float(commanded_data)
                
                if commanded_val > 0.1:
                    difference = abs(measured_val - commanded_val)
                    
                    statistics['max_acceleration_error'] = max(statistics.get('max_acceleration_error', 0), difference)
                    statistics['min_acceleration_error'] = min(statistics.get('min_acceleration_error', float('inf')), difference)
                    
                    statistics['max_measured_acceleration'] = max(statistics.get('max_measured_acceleration', 0), measured_val)
                    statistics['max_commanded_acceleration'] = max(statistics.get('max_commanded_acceleration', 0), commanded_val)
                    
                    statistics['acceleration_errors'].append(difference)
                    statistics['measured_accelerations'].append(measured_val)
                    statistics['commanded_accelerations'].append(commanded_val)
                    
                    statistics['acceleration_events'] += 1
                    
                    if 'accel_error_sum' not in statistics:
                        statistics['accel_error_sum'] = 0
                        statistics['measured_accel_sum'] = 0
                        statistics['commanded_accel_sum'] = 0
                    
                    statistics['accel_error_sum'] += difference
                    statistics['measured_accel_sum'] += measured_val
                    statistics['commanded_accel_sum'] += commanded_val
                    
                    threshold = prop_config.threshold or 1.5
                    if difference > threshold:
                        statistics['compliance_violations'] += 1
                    
                statistics['states_with_data'] += 1
            else:
                statistics['states_without_data'] += 1
                
        except Exception as e:
            logging.debug(f"Error collecting acceleration compliance statistics: {e}")
            statistics['states_without_data'] += 1

    def _collect_deceleration_compliance_statistics(self, state: VehicleState, prop_config: PropositionConfig, 
                                                  statistics: Dict[str, Any]):
        """Collect deceleration compliance statistics"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            measured_data = aggregated_data.get('measured_acceleration')
            commanded_data = aggregated_data.get('commanded_acceleration')
            
            if measured_data is not None and commanded_data is not None:
                measured_val = float(measured_data)
                commanded_val = float(commanded_data)
                
                if commanded_val < -0.1:
                    difference = abs(measured_val - commanded_val)
                    
                    statistics['max_deceleration_error'] = max(statistics.get('max_deceleration_error', 0), difference)
                    statistics['min_deceleration_error'] = min(statistics.get('min_deceleration_error', float('inf')), difference)
                    
                    statistics['max_measured_deceleration'] = min(statistics.get('max_measured_deceleration', 0), measured_val)
                    statistics['max_commanded_deceleration'] = min(statistics.get('max_commanded_deceleration', 0), commanded_val)
                    
                    statistics['deceleration_errors'].append(difference)
                    statistics['measured_decelerations'].append(measured_val)
                    statistics['commanded_decelerations'].append(commanded_val)
                    
                    statistics['deceleration_events'] += 1
                    
                    if 'decel_error_sum' not in statistics:
                        statistics['decel_error_sum'] = 0
                        statistics['measured_decel_sum'] = 0
                        statistics['commanded_decel_sum'] = 0
                    
                    statistics['decel_error_sum'] += difference
                    statistics['measured_decel_sum'] += measured_val
                    statistics['commanded_decel_sum'] += commanded_val
                    
                    threshold = prop_config.threshold or 2.0
                    if difference > threshold:
                        statistics['compliance_violations'] += 1
                    
                statistics['states_with_data'] += 1
            else:
                statistics['states_without_data'] += 1
                
        except Exception as e:
            logging.debug(f"Error collecting deceleration compliance statistics: {e}")
            statistics['states_without_data'] += 1

    def _collect_ttc_compliance_statistics(self, state: VehicleState, prop_config: PropositionConfig, 
                                                  statistics: Dict[str, Any]):
        """Collect ttc compliance statistics"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)

            ego_data = aggregated_data.get('ego_state')
            traffic_data = aggregated_data.get('traffic_participants')

            ego_x = DataTransforms.get_nested_value(ego_data, 'x')
            ego_y = DataTransforms.get_nested_value(ego_data, 'y')
            ego_vx = DataTransforms.get_nested_value(ego_data, 'vx')
            ego_vy = DataTransforms.get_nested_value(ego_data, 'vy')

            threshold = prop_config.threshold or self.config.safety_params.time_to_collision_threshold

            for participant in traffic_data:
                if isinstance(participant, dict):

                    participant_data = participant.get('participant_data', {})
                    motion_state = participant_data.get('motion_state', {})
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
                    if ttc is not None:
                        if statistics['min_ttc'] is None:
                            statistics['min_ttc'] = ttc
                        else:
                            statistics['min_ttc'] = min(ttc, statistics['min_ttc'])
                            statistics['sun_ttc'] += ttc;

                    threshold = prop_config.threshold
                    if ttc < threshold:
                        statistics['compliance_violations'] += 1
                statistics['states_with_data'] += 1
            else:
                statistics['states_without_data'] += 1

        except Exception as e:
            logging.debug(f"Error collecting ttc compliance statistics: {e}")
            statistics['states_without_data'] += 1

    def _collect_smooth_steering_compliance_statistics(self, state: VehicleState, prop_config: PropositionConfig, 
                                                  statistics: Dict[str, Any]):
        """Collect smooth steering compliance statistics"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            measured_data = aggregated_data.get('steering_rate')
            
            if measured_data is not None:
                measured_val = float(measured_data)

                if statistics['max_steering_rate'] is None:
                    statistics['max_steering_rate'] = 0.0
                if statistics['min_steering_rate'] is None:
                    statistics['min_steering_rate'] = float('inf')

                statistics['max_steering_rate'] = max(statistics['max_steering_rate'], measured_val)
                statistics['min_steering_rate'] = min(statistics['min_steering_rate'], measured_val)

                threshold = prop_config.threshold
                if measured_val > threshold:
                    statistics['compliance_violations'] += 1

                statistics['states_with_data'] += 1
        except Exception as e:
            logging.debug(f"Error collecting smooth steering compliance statistics: {e}")
            statistics['states_without_data'] += 1

    def _collect_lanekeeping_compliance_statistics(self, state: VehicleState, prop_config: PropositionConfig,
                                                  statistics: Dict[str, Any]):
        """Collect deceleration compliance statistics"""
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            lane_data = aggregated_data.get('lane_info')
            vehicle_data = aggregated_data.get('vehicle_state')
            min_distance_on_map = float('inf')
            x = DataTransforms.get_nested_value(vehicle_data, 'x')
            y = DataTransforms.get_nested_value(vehicle_data, 'y')

            if lane_data is not None and vehicle_data is not None:
                road_data = DataTransforms.get_nested_value(lane_data, 'roads')
                for road in road_data:
                    lanes_data = DataTransforms.get_nested_value(road, 'lanes')
                    for lanes in lanes_data:
                        center_points = DataTransforms.get_nested_value(lanes, 'center_points')
                        if len(center_points) < 2:
                            raise ValueError("At least two points are required to define a line")
                        for i in range(len(center_points) - 2):
                            a = center_points[i]
                            b = center_points[i + 1]
                            closest = DataTransforms.closest_point_on_segment_2d(x, y, a, b)
                            dist = DataTransforms.calculate_distance(x, y, closest['x'], closest['y'])
                            if dist < min_distance_on_map:
                                min_distance_on_map = dist

                #the maximum of all minimum distances to centerline of each frame
                statistics['max_distance'] = max(statistics.get('max_distance', 0), min_distance_on_map)

                if 'distance_error_sum' not in statistics:
                    statistics['distance_error_sum'] = 0
                statistics['distance_error_sum'] += min_distance_on_map

                threshold = prop_config.threshold
                if min_distance_on_map > threshold:
                    statistics['compliance_violations'] += 1
                statistics['states_with_data'] += 1
                
                statistics['lane_values'].append(min_distance_on_map)

                safety_score = self._get_lane_safety_score(min_distance_on_map)

                statistics['safety_scores'].append(safety_score)
                statistics['average_safety_score'] = self._get_guarded_lane_safety_score(statistics['safety_scores'], statistics['lane_values']) 
                statistics['safety_grade'], statistics['safety_grade_format'] = self.get_grade_from_safety_score(statistics['average_safety_score'], 
                                                                                                self.config.safety_params.lane_safety_score_grade_format)
            else:
                statistics['states_without_data'] += 1

        except Exception as e:
            logging.debug(f"Error collecting lanekeeping compliance statistics: {e}")
            statistics['states_without_data'] += 1

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
                    self._aggregate_data_sources(state, prop_config), prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.EGO_SPEED:
                return PropositionEvaluators.speed_limit_evaluator(
                    self._aggregate_data_sources(state, prop_config), prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.SAFE_DISTANCE_X:
                return PropositionEvaluators.longitudinal_safety_evaluator(
                    self._aggregate_data_sources(state, prop_config), prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.SAFE_DISTANCE_Y:
                return self._check_lateral_safety(state, prop_config)
            
            elif prop_type == PropositionType.DECELERATION:
                return PropositionEvaluators.deceleration_evaluator(
                    self._aggregate_data_sources(state, prop_config), prop_config, self.config.safety_params
                )

            elif prop_type == PropositionType.ACCELERATION_COMPLIANCE:
                return PropositionEvaluators.acceleration_compliance_evaluator(
                    self._aggregate_data_sources(state, prop_config), prop_config, self.config.safety_params
                )

            elif prop_type == PropositionType.DECELERATION_COMPLIANCE:
                return PropositionEvaluators.deceleration_compliance_evaluator(
                    self._aggregate_data_sources(state, prop_config), prop_config, self.config.safety_params
                )
            
            elif prop_type == PropositionType.TIME_TO_COLLISION:
                return self._check_time_to_collision(state, prop_config)
            
            elif prop_type == PropositionType.LANE_KEEPING:
                return self._check_lane_keeping(state, prop_config)
            
            elif prop_type == PropositionType.TRAFFIC_LIGHT_COMPLIANCE:
                return self._check_traffic_light_compliance(state, prop_config)
            
            elif prop_type == PropositionType.SMOOTH_ACCELERATION:
                return self._check_smooth_acceleration(state, prop_config)

            elif prop_type == PropositionType.SMOOTH_STEERING:
                return self._check_smooth_steering(state, prop_config)

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
            
            min_distance_on_map = float('inf')
            x = DataTransforms.get_nested_value(vehicle_data, 'x')
            y = DataTransforms.get_nested_value(vehicle_data, 'y')
            road_data = DataTransforms.get_nested_value(lane_data, 'roads')
            for road in road_data:
                lanes_data = DataTransforms.get_nested_value(road, 'lanes')
                for lanes in lanes_data:
                    center_points = DataTransforms.get_nested_value(lanes, 'center_points')
                    if len(center_points) < 2:
                        raise ValueError("At least two points are required to define a line")
                    for i in range(len(center_points) - 2):
                        a = center_points[i]
                        b = center_points[i + 1]
                        closest = DataTransforms.closest_point_on_segment_2d(x, y, a, b)
                        dist = DataTransforms.calculate_distance(x, y, closest['x'], closest['y'])
                        if dist < min_distance_on_map:
                            min_distance_on_map = dist

            threshold = prop_config.threshold or self.config.safety_params.lane_deviation_threshold
            return min_distance_on_map <= threshold
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

    def _check_smooth_steering(self, state: VehicleState, prop_config: PropositionConfig) -> Optional[bool]:
        try:
            aggregated_data = self._aggregate_data_sources(state, prop_config)
            steering_rate = aggregated_data.get('steering_rate')
            if steering_rate is None:
                return None

            max_steering_rate = prop_config.threshold or self.config.safety_params.max_steering_rate
            abs(steering_rate) <= max_steering_rate
            return abs(steering_rate) <= max_steering_rate
            
        except Exception as e:
            logging.error(f"Error in smooth acceleration check: {e}")
            return None
    
    def _get_speed_safety_score(self, vel_val: float) -> float:
        # self.config.safety_params.max_speed
        safe_val = 1.0
        if (vel_val >= self.config.safety_params.higher_speed_limit_tolerance):
            safe_val = self.config.safety_params.higher_speed_limit_tolerance_safety_score
        elif (vel_val >= self.config.safety_params.speed_limit_tolerance):
            sign_norm = self._safety_normalization_function(vel_val, self.config.safety_params.speed_limit_tolerance, 
                                                            self.config.safety_params.higher_speed_limit_tolerance)
            safe_val = self._scale_function(sign_norm, self.config.safety_params.speed_limit_tolerance_safety_score, 
                                            self.config.safety_params.higher_speed_limit_tolerance_safety_score)
        elif (vel_val > 0):
            sign_norm = self._safety_normalization_function(vel_val, 0, self.config.safety_params.speed_limit_tolerance)
            safe_val = self._scale_function(sign_norm, 1, self.config.safety_params.speed_limit_tolerance_safety_score)
        return safe_val

    def _get_guarded_speed_safety_score(self, safety_array: List[float], speed_array: List[float]) -> float:
        safe_val = 1.0

        # average
        safe_val = sum(safety_array) / float(len(safety_array))
    
        # guard conditions
        tolerance_violations = 0
        higher_tolerance_violations = 0
        for speed in speed_array:
            if speed > self.config.safety_params.higher_speed_limit_tolerance:
                higher_tolerance_violations += 1
            if speed > self.config.safety_params.speed_limit_tolerance:
                tolerance_violations += 1

        if higher_tolerance_violations >= self.config.safety_params.higher_speed_limit_tolerance_threshold:
            if safe_val > self.config.safety_params.higher_speed_limit_tolerance_safety_score:
                safe_val = self.config.safety_params.higher_speed_limit_tolerance_safety_score
        elif (float(tolerance_violations)/float(len(speed_array))) >= self.config.safety_params.speed_limit_tolerance_threshold:
            if safe_val > self.config.safety_params.speed_limit_tolerance_safety_score:
                safe_val = self.config.safety_params.speed_limit_tolerance_safety_score
        return safe_val
    
    def _get_lane_safety_score(self, infringement: float) -> float:
        safe_val = 1.0
        if (infringement >= self.config.safety_params.upper_lane_tolerance):
            safe_val = self.config.safety_params.upper_lane_tolerance_safety_score
        elif (infringement >= self.config.safety_params.lane_deviation_threshold):
            sign_norm = self._safety_normalization_function(infringement, self.config.safety_params.lane_deviation_threshold, 
                                                            self.config.safety_params.upper_lane_tolerance)
            safe_val = self._scale_function(sign_norm, self.config.safety_params.lane_tolerance_safety_score, 
                                            self.config.safety_params.upper_lane_tolerance_safety_score_safety_score)
        elif (infringement >= self.config.safety_params.lower_lane_tolerance):
            sign_norm = self._safety_normalization_function(infringement, self.config.safety_params.lower_lane_tolerance, 
                                                            self.config.safety_params.lane_deviation_threshold)
            safe_val = self._scale_function(sign_norm, self.config.safety_params.lower_lane_tolerance_safety_score, 
                                            self.config.safety_params.lane_tolerance_safety_score)
        elif (infringement > 0):
            sign_norm = self._safety_normalization_function(infringement, 0, 
                                                            self.config.safety_params.lower_lane_tolerance)
            safe_val = self._scale_function(sign_norm, 1, 
                                            self.config.safety_params.lower_lane_tolerance_safety_score)
        return safe_val
    
    def _get_guarded_lane_safety_score(self, safety_array: List[float], lane_array: List[float]) -> float:
        safe_val = 1.0

        # average
        safe_val = sum(safety_array) / float(len(safety_array))
    
        # guard conditions
        tolerance_violations = 0
        upper_tolerance_violations = 0
        for infringement in lane_array:
            if infringement > self.config.safety_params.upper_lane_tolerance:
                upper_tolerance_violations += 1
            if infringement > self.config.safety_params.lane_deviation_threshold:
                tolerance_violations += 1

        if upper_tolerance_violations >= self.config.safety_params.upper_lane_tolerance_threshold:
            if safe_val > self.config.safety_params.upper_lane_tolerance_safety_score:
                safe_val = self.config.safety_params.upper_lane_tolerance_safety_score
        elif (float(tolerance_violations)/float(len(lane_array))) >= self.config.safety_params.lane_tolerance_threshold:
            if safe_val > self.config.safety_params.lane_tolerance_safety_score:
                safe_val = self.config.safety_params.lane_tolerance_safety_score
        return safe_val

    def _safety_normalization_function(self, value: float, upper: float, lower: float) -> float:
        """Upper Bound is where system is more safe
            Lower Bound is where system is less safe
        """
        norm = (value - lower) / (upper - lower)
        # if lower > upper:
        #     norm = 1 -norm
        return norm
    
    def _scale_function(self, value: float, upper: float, lower: float) -> float:
        """Scales a normalized value to the range [lower, upper]
        """
        scale = (upper - lower)*value + lower
        return scale

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
        if prop_type in [PropositionType.EGO_SPEED, PropositionType.NEAR_GOAL, PropositionType.SMOOTH_STEERING,
                        PropositionType.ACCELERATION_COMPLIANCE, PropositionType.DECELERATION_COMPLIANCE,
                        PropositionType.LANE_KEEPING]: #BAB: I think this must always run and the else never?
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
                'average_safety_score': 0.0,
                'safety_grade': "",
                'safety_grade_format': "",
                'speed_values': [],
                'safety_scores': []
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
        elif prop_type == PropositionType.TIME_TO_COLLISION:
            statistics = {
                'min_ttc': None,
                'avg_ttc': None,
                'sum_ttc': 0,
                'goal_threshold': prop_config.threshold or self.config.safety_params.time_to_collision_threshold,
                'states_with_data': 0,
                'states_without_data': 0,
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0
            }
        elif prop_type == PropositionType.SMOOTH_STEERING:
            statistics = {
                'max_steering_rate': None,
                'min_steering_rate': None,
                'states_with_data': 0,
                'states_without_data': 0,
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0
            }
        elif prop_type == PropositionType.DECELERATION:
            statistics = {
                'max_acceleration': float('-inf'),
                'min_acceleration': float('inf'),
                'max_positive_acceleration': 0.0,
                'max_deceleration': 0.0,
                'average_acceleration': 0.0,
                'emergency_threshold': prop_config.threshold or self.config.safety_params.emergency_brake_deceleration,
                'acceleration_events': 0,
                'deceleration_events': 0,
                'emergency_brake_events': 0,
                'brake_inconsistency_events': 0,
                'states_with_data': 0,
                'states_without_data': 0,
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0,
                'acceleration_values': []
            }
        elif prop_type == PropositionType.ACCELERATION_COMPLIANCE:
            statistics = {
                'max_acceleration_error': 0.0,
                'min_acceleration_error': float('inf'),
                'max_measured_acceleration': 0.0,
                'max_commanded_acceleration': 0.0,
                'average_acceleration_error': 0.0,
                'average_measured_acceleration': 0.0,
                'average_commanded_acceleration': 0.0,
                'acceleration_threshold': prop_config.threshold or 1.5,
                'acceleration_events': 0,
                'compliance_violations': 0,
                'states_with_data': 0,
                'states_without_data': 0,
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0,
                'acceleration_errors': [],
                'measured_accelerations': [],
                'commanded_accelerations': []
            }
        elif prop_type == PropositionType.DECELERATION_COMPLIANCE:
            statistics = {
                'max_deceleration_error': 0.0,
                'min_deceleration_error': float('inf'),
                'max_measured_deceleration': 0.0,
                'max_commanded_deceleration': 0.0,
                'average_deceleration_error': 0.0,
                'average_measured_deceleration': 0.0,
                'average_commanded_deceleration': 0.0,
                'deceleration_threshold': prop_config.threshold or 2.0,
                'deceleration_events': 0,
                'compliance_violations': 0,
                'states_with_data': 0,
                'states_without_data': 0,
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0,
                'deceleration_errors': [],
                'measured_decelerations': [],
                'commanded_decelerations': []
            }
        elif prop_type == PropositionType.LANE_KEEPING:
            statistics = {
                'max_distance' : 0,
                'avg_distance' : float('inf'),
                'compliance_violations' : 0,
                'states_with_data' : 0,
                'states_without_data' : 0,
                'valid_evaluations': 0,
                'failed_evaluations': 0,
                'true_evaluations': 0,
                'false_evaluations': 0,
                'average_safety_score': 0.0,
                'safety_grade': "",
                'safety_grade_format': "",
                'lane_values': [],
                'safety_scores': []
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
                elif prop_type == PropositionType.DECELERATION:
                    self._collect_deceleration_statistics(state, prop_config, statistics)
                elif prop_type == PropositionType.ACCELERATION_COMPLIANCE:
                    self._collect_acceleration_compliance_statistics(state, prop_config, statistics)
                elif prop_type == PropositionType.DECELERATION_COMPLIANCE:
                    self._collect_deceleration_compliance_statistics(state, prop_config, statistics)
                elif prop_type == PropositionType.TIME_TO_COLLISION:
                    self._collect_ttc_compliance_statistics(state, prop_config, statistics)
                elif prop_type == PropositionType.SMOOTH_STEERING:
                    self._collect_smooth_steering_compliance_statistics(state, prop_config, statistics)
                elif prop_type == PropositionType.LANE_KEEPING:
                    self._collect_lanekeeping_compliance_statistics(state, prop_config, statistics)

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
        if prop_type == PropositionType.TIME_TO_COLLISION:
            if statistics['states_with_data'] > 0:
                statistics['avg_ttc'] = statistics.get('sum_ttc', 0) / statistics['states_with_data']
        elif prop_type == PropositionType.DECELERATION:
            if statistics['states_with_data'] > 0:
                statistics['average_acceleration'] = statistics.get('accel_sum', 0) / statistics['states_with_data']
        elif prop_type == PropositionType.ACCELERATION_COMPLIANCE:
            if statistics['acceleration_events'] > 0:
                statistics['average_acceleration_error'] = statistics.get('accel_error_sum', 0) / statistics['acceleration_events']
                statistics['average_measured_acceleration'] = statistics.get('measured_accel_sum', 0) / statistics['acceleration_events']
                statistics['average_commanded_acceleration'] = statistics.get('commanded_accel_sum', 0) / statistics['acceleration_events']
                statistics['compliance_rate'] = (statistics['acceleration_events'] - statistics['compliance_violations']) / statistics['acceleration_events']
        elif prop_type == PropositionType.DECELERATION_COMPLIANCE:
            if statistics['deceleration_events'] > 0:
                statistics['average_deceleration_error'] = statistics.get('decel_error_sum', 0) / statistics['deceleration_events']
                statistics['average_measured_deceleration'] = statistics.get('measured_decel_sum', 0) / statistics['deceleration_events']
                statistics['average_commanded_deceleration'] = statistics.get('commanded_decel_sum', 0) / statistics['deceleration_events']
                statistics['compliance_rate'] = (statistics['deceleration_events'] - statistics['compliance_violations']) / statistics['deceleration_events']
        elif prop_type ==PropositionType.LANE_KEEPING:
            statistics['distance_error_avg'] = statistics.get('distance_error_sum', 0) / statistics['states_with_data']

        if not L:
            return None, statistics

        logging.info(f"Created Kripke structure for {prop_type.name}:")
        logging.info(f"  Original states: {len(limited_states)}, Valid states: {len(valid_states)}")
        logging.info(f"  True evaluations: {statistics.get('true_evaluations', 0)}")
        logging.info(f"  False evaluations: {statistics.get('false_evaluations', 0)}")

        return Kripke(R=R, L=L), statistics
    
    def get_grade_from_safety_score(self, safety_score: float, grade_format: str) -> str | str:
        
        if grade_format == 'american':
            grade_value = 'A'
            if safety_score <= 0.4:
                grade_value = 'F'
            elif safety_score <= 0.45:
                grade_value = 'D-'
            elif safety_score <= 0.5:
                grade_value = 'D'
            elif safety_score <= 0.55:
                grade_value = 'D+'
            elif safety_score <= 0.6:
                grade_value = 'C-'
            elif safety_score <= 0.65:
                grade_value = 'C'
            elif safety_score <= 0.7:
                grade_value = 'C+'
            elif safety_score <= 0.8:
                grade_value = 'B-'
            elif safety_score <= 0.85:
                grade_value = 'B'
            elif safety_score <= 0.9:
                grade_value = 'B+'
            elif safety_score <= 0.95:
                grade_value = 'A-'
            
            return grade_value, grade_format

        elif grade_format == 'german':
            grade_value = '1.0'
            if safety_score <= 0.4:
                grade_value = '5.0'
            elif safety_score <= 0.45:
                grade_value = '4.0'
            elif safety_score <= 0.5:
                grade_value = '4.0'
            elif safety_score <= 0.55:
                grade_value = '3.7'
            elif safety_score <= 0.6:
                grade_value = '3.3'
            elif safety_score <= 0.65:
                grade_value = '3.0'
            elif safety_score <= 0.7:
                grade_value = '2.7'
            elif safety_score <= 0.8:
                grade_value = '2.3'
            elif safety_score <= 0.85:
                grade_value = '2.0'
            elif safety_score <= 0.9:
                grade_value = '1.7'
            elif safety_score <= 0.95:
                grade_value = '1.3'
            
            return grade_value, grade_format
        
        elif grade_format == 'both':
            grade_value = 'A, 1.0'
            if safety_score <= 0.4:
                grade_value = 'F, 5.0'
            elif safety_score <= 0.45:
                grade_value = 'D-, 4.0'
            elif safety_score <= 0.5:
                grade_value = 'D, 4.0'
            elif safety_score <= 0.55:
                grade_value = 'D+, 3.7'
            elif safety_score <= 0.6:
                grade_value = 'C-, 3.3'
            elif safety_score <= 0.65:
                grade_value = 'C, 3.0'
            elif safety_score <= 0.7:
                grade_value = 'C+, 2.7'
            elif safety_score <= 0.8:
                grade_value = 'B-, 2.3'
            elif safety_score <= 0.85:
                grade_value = 'B, 2.0'
            elif safety_score <= 0.9:
                grade_value = 'B+, 1.7'
            elif safety_score <= 0.95:
                grade_value = 'A-, 1.3'
            
            return grade_value, grade_format

        else:
            return 'None', 'None'
            # No Output
    
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

    def _print_progress(self, message):
        """Print progress message that can be captured by API"""
        print(f"[PROGRESS]: {message}", flush=True)
    
    def analyze_online(self, vehicle_id: int, duration: float) -> Dict[str, Any]:
        """Analyze vehicle behavior online"""
        self._print_progress("Setting up online monitoring")
        
        vehicle_config = next((v for v in self.config.vehicles if v.id == vehicle_id), None)
        if not vehicle_config:
            raise ValueError(f"Vehicle {vehicle_id} not found in configuration")
        
        self._print_progress("Creating online monitor")
        monitor = OnlineMonitor(self.config, vehicle_config)
        
        try:
            start_time = time.time()
            self._print_progress(f"Starting monitoring for {duration} seconds")
            logging.info(f"Starting online monitoring for vehicle {vehicle_id} for {duration} seconds...")
            
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                if int(elapsed) % 10 == 0 and elapsed > 0:  # Progress every 10 seconds
                    self._print_progress(f"Monitoring progress: {elapsed:.0f}/{duration:.0f} seconds")
                time.sleep(0.1)
            
            self._print_progress("Collecting monitored data")
            states = []
            while not monitor.data_buffer.empty():
                state = monitor.data_buffer.get()
                states.append(state)
            
            self._print_progress(f"Collected {len(states)} states for analysis")
            logging.info(f"Collected {len(states)} states for analysis")
            
            self._print_progress("Starting state analysis")
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
        
        time_step = 0.1
        current_time = min_time
        
        while current_time <= max_time:
            state = VehicleState(timestamp=current_time, vehicle_id=vehicle_id)
            
            for topic, messages in topic_data.items():
                closest_msg = min(messages, 
                                key=lambda msg: abs(msg['timestamp'] - current_time))
                
                if abs(closest_msg['timestamp'] - current_time) < 0.5:
                    state.data[topic] = closest_msg
            
            if state.data: 
                states.append(state)
            
            current_time += time_step
        
        return states
    
    def _analyze_states(self, states: List[VehicleState], 
                        vehicle_config: VehicleConfig) -> Dict[str, Any]:
        results = {}
        enabled_propositions = self._get_enabled_propositions(vehicle_config)
        
        self._print_progress(f"Analyzing {len(states)} states against {len(enabled_propositions)} propositions")
        logging.info(f"Analyzing {len(states)} states against {len(enabled_propositions)} propositions")
        
        total_props = len(enabled_propositions)
        analyzed_props = 0
        
        for prop_name, prop_config in enabled_propositions.items():
            try:
                analyzed_props += 1
                self._print_progress(f"Analyzing proposition {prop_name} ({analyzed_props}/{total_props})")
                logging.info(f"Starting analysis of {prop_name}")
                
                prop_type = PropositionType[prop_name]
                
                # Get human-readable description
                description_info = prop_type.get_human_description()
                formula_description = prop_type.get_formula_description(prop_config.formula_type)
                
                self._print_progress(f"Creating Kripke structure for {prop_name}")
                logging.info(f"Creating Kripke structure for {prop_name}")
                kripke, statistics = self.model_checker.create_kripke_from_states(
                    states, prop_config, prop_type
                )
                
                if kripke:
                    self._print_progress(f"Checking model for {prop_name}")
                    logging.info(f"Kripke structure created for {prop_name}, checking model")
                    formula = self.model_checker.get_formula(prop_config)
                    result = self.model_checker.check_model(kripke, formula)
                    results[prop_name] = {
                        'result': result,
                        'status': 'PASS' if result else 'FAIL',
                        'states_analyzed': len(states),
                        'kripke_states': len(kripke.labelling_function()),
                        'statistics': statistics,
                        # Human-comprehensible descriptions
                        'description': description_info,
                        'formula_description': formula_description,
                        'formula_type': prop_config.formula_type,
                        'threshold': prop_config.threshold,
                        'group': prop_type.group.value
                    }
                    logging.info(f"{prop_name}: {'PASS' if result else 'FAIL'}")
                else:
                    logging.warning(f"Could not create Kripke structure for {prop_name}")
                    results[prop_name] = {
                        'result': None,
                        'status': 'NO_DATA',
                        'states_analyzed': 0,
                        'kripke_states': 0,
                        'statistics': statistics,
                        'description': description_info,
                        'formula_description': formula_description,
                        'formula_type': prop_config.formula_type,
                        'threshold': prop_config.threshold,
                        'group': prop_type.group.value
                    }
                    logging.warning(f"No data for {prop_name}")
                    
            except KeyError:
                logging.error(f"Unknown proposition type: {prop_name}")
                results[prop_name] = {
                    'result': None,
                    'status': 'ERROR',
                    'error': 'Unknown proposition type',
                    'statistics': {},
                    'description': {
                        'title': prop_name.replace('_', ' ').title(),
                        'description': f'Unknown proposition: {prop_name}',
                        'safety_rationale': 'Not available for unknown proposition types'
                    },
                    'formula_description': 'Not available',
                    'formula_type': 'unknown',
                    'threshold': None,
                    'group': 'unknown'
                }
            except Exception as e:
                logging.error(f"Error analyzing {prop_name}: {e}")
                import traceback
                logging.error(traceback.format_exc())
                results[prop_name] = {
                    'result': None,
                    'status': 'ERROR',
                    'error': str(e),
                    'statistics': {},
                    'description': {
                        'title': prop_name.replace('_', ' ').title(),
                        'description': f'Error analyzing proposition: {prop_name}',
                        'safety_rationale': 'Analysis failed due to technical error'
                    },
                    'formula_description': 'Not available due to error',
                    'formula_type': prop_config.formula_type if 'prop_config' in locals() else 'unknown',
                    'threshold': prop_config.threshold if 'prop_config' in locals() else None,
                    'group': 'unknown'
                }
        
        self._print_progress("Calculating summary statistics")
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
    
        self._print_progress("State analysis completed")
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
