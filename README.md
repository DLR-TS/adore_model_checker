# ADORe ROS2 Model Checker for Vehicle Safety Monitoring in ADORe

A tool for monitoring and verifying safety properties of autonomous vehicles 
using Computation Tree Logic (CTL) model checking. This tool can analyze both 
live ROS2 data streams and offline bag data files to ensure vehicles comply with 
safety requirements.

This tool is a WORK IN PROGRESS and likely has many bugs.

## Features

- **Dual Mode Operation**: Online monitoring of live ROS2 topics and offline analysis of bag files
- **Configurable Safety Properties**: 50+ built-in safety propositions organized into 10 categories
- **Multi-Vehicle Support**: Monitor multiple vehicles simultaneously with individual configurations
- **Flexible Data Sources**: Map any ROS2 topics to safety propositions with field path extraction
- **CTL Model Checking**: Uses formal verification techniques to prove safety properties
- **Extensible Framework**: Easy to add custom safety propositions and evaluation logic
- **Comprehensive Reporting**: Detailed analysis results with statistics and pass/fail status
- **Auto-importing ROS messages**: All defined ROS messages are auto-imported with the `ROSMessageImporter` class 
- **Dictionary Based Topic Subscription**: All subscribed ROS topic messages are converted to dictionaries with the `ROSMarshaller` class 


## Proposition Categories

## Safety Proposition Categories

| Category | Proposition | Implementation Status |
|----------|-------------|------------------------|
| **Basic Safety** | EGO_SPEED | IMPLEMENTED |
|  | NEAR_GOAL | IMPLEMENTED |
|  | SAFE_DISTANCE_X | WORK IN PROGRESS |
|  | SAFE_DISTANCE_Y | WORK IN PROGRESS |
|  | DECELERATION | WORK IN PROGRESS |
| **Lane Compliance** | LANE_KEEPING | WORK IN PROGRESS |
|  | LANE_CHANGE_SAFE | WORK IN PROGRESS |
|  | ROAD_BOUNDARY_RESPECT | WORK IN PROGRESS |
|  | WRONG_WAY_DRIVING | WORK IN PROGRESS |
| **Traffic Rules** | TRAFFIC_LIGHT_COMPLIANCE | WORK IN PROGRESS |
|  | STOP_SIGN_COMPLIANCE | WORK IN PROGRESS |
|  | SPEED_LIMIT_COMPLIANCE | WORK IN PROGRESS |
|  | YIELD_COMPLIANCE | WORK IN PROGRESS |
|  | TURN_SIGNAL_USAGE | WORK IN PROGRESS |
| **Dynamic Safety** | TIME_TO_COLLISION | WORK IN PROGRESS |
|  | EMERGENCY_BRAKING | WORK IN PROGRESS |
|  | OBSTACLE_AVOIDANCE | WORK IN PROGRESS |
|  | PEDESTRIAN_SAFETY | WORK IN PROGRESS |
|  | CYCLIST_SAFETY | WORK IN PROGRESS |
|  | BLIND_SPOT_MONITORING | WORK IN PROGRESS |
| **Behavioral Smoothness** | SMOOTH_ACCELERATION | WORK IN PROGRESS |
|  | SMOOTH_STEERING | WORK IN PROGRESS |
|  | SMOOTH_BRAKING | WORK IN PROGRESS |
|  | COMFORT_ZONE | WORK IN PROGRESS |
| **Intersection Behavior** | INTERSECTION_APPROACH | WORK IN PROGRESS |
|  | RIGHT_OF_WAY | WORK IN PROGRESS |
|  | INTERSECTION_CLEARANCE | WORK IN PROGRESS |
|  | TURNING_BEHAVIOR | WORK IN PROGRESS |
| **System Health** | SENSOR_HEALTH | WORK IN PROGRESS |
|  | LOCALIZATION_ACCURACY | WORK IN PROGRESS |
|  | COMMUNICATION_STATUS | WORK IN PROGRESS |
|  | SYSTEM_RESPONSE_TIME | WORK IN PROGRESS |
|  | PATH_PLANNING_VALIDITY | WORK IN PROGRESS |
| **Environmental Adaptation** | WEATHER_ADAPTATION | WORK IN PROGRESS |
|  | VISIBILITY_ADAPTATION | WORK IN PROGRESS |
|  | ROAD_CONDITION_ADAPTATION | WORK IN PROGRESS |
|  | TRAFFIC_DENSITY_ADAPTATION | WORK IN PROGRESS |
| **Mission Efficiency** | ROUTE_OPTIMIZATION | WORK IN PROGRESS |
|  | FUEL_EFFICIENCY | WORK IN PROGRESS |
|  | TIME_EFFICIENCY | WORK IN PROGRESS |
|  | PARKING_BEHAVIOR | WORK IN PROGRESS |
| **Advanced Maneuvers** | OVERTAKING_SAFETY | WORK IN PROGRESS |
|  | MERGING_BEHAVIOR | WORK IN PROGRESS |
|  | ROUNDABOUT_BEHAVIOR | WORK IN PROGRESS |
|  | CONSTRUCTION_ZONE_BEHAVIOR | WORK IN PROGRESS |


## Installation

### Prerequisites

- ROS2 (Foxy, Galactic, Humble, or Iron)
- Python 3.8+

## Installing System Requirements
System-level dependencies are listed in the `requirements.system` file, 
with one package name per line. Lines starting with `#` are treated as comments.
For inline comments, everything after the first `#` on a line will be ignored 
by the installation command.

You can install all necessary `apt` packages by running the following command in your terminal:
```bash
sudo apt-get update
grep -v '^#' requirements.system | sed 's/#.*//' | xargs sudo apt-get install -y

Installing required Python packages:
```bash
pip3 install -r requirements.pip3
```

### ROS2 Setup
Make sure your ROS2 environment is sourced:
```bash
source /opt/ros/<your-ros-distro>/setup.bash
```

## Quick Start

### 1. Generate Configuration

Create a minimal configuration file:
```bash
python3 ModelChecker.py --create-minimal-config minimal_config.yaml
```

### 2. Configure Data Sources

Edit the configuration file to map your ROS2 topics:

```yaml
vehicles:
  - id: 0
    proposition_groups:
      basic_safety:
        enabled: true
        description: "Core safety propositions"
    
    propositions:
      EGO_SPEED:
        enabled: true
        atomic_prop: 'speed_safe'
        formula_type: 'always'
        threshold: 30.0
        data_sources:
          vehicle_state:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: 'vx'
            transform_function: 'abs'
            cache_duration: 0.1
        evaluation_function: 'speed_limit_evaluator'
```

### 3a. Online Monitoring
Monitor a live vehicle for 60 seconds:
```bash
python3 ModelChecker.py --mode online --config your_config.yaml --vehicle-id 0 --duration 60
```

### 3b. Offline Analysis 
Analyze offline bag data:
```bash
python3 ModelChecker.py --mode offline --config your_config.yaml --bag-file data.bag
```

## Configuration

The tool uses YAML configuration files to specify:

- Vehicle data source mappings
- Enabled safety proposition groups
- Individual proposition settings with data sources
- Safety parameters and thresholds
- Monitoring frequency and buffer settings

### Example Configuration

```yaml
monitoring:
  monitoring_frequency: 10.0
  buffer_size: 1000
  log_level: INFO
  debug_mode: false

safety_parameters:
  max_speed: 30.0
  safe_distance_lateral: 1.5
  time_to_collision_threshold: 3.0
  goal_reach_distance: 5.0

vehicles:
  - id: 0
    proposition_groups:
      basic_safety:
        enabled: true
        description: "Core safety propositions"
      dynamic_safety:
        enabled: true
        description: "Dynamic collision avoidance"
      traffic_rules:
        enabled: false
        description: "Traffic law compliance"
    
    propositions:
      EGO_SPEED:
        enabled: true
        atomic_prop: 'speed_safe'
        formula_type: 'always'
        threshold: 30.0
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

      SAFE_DISTANCE_X:
        enabled: false
        atomic_prop: 'longitudinal_safe'
        formula_type: 'always'
        data_sources:
          ego_state:
            topic: '/ego_vehicle/vehicle_state/dynamic'
            field_path: ''
            cache_duration: 0.1
          traffic_participants:
            topic: '/ego_vehicle/traffic_participants'
            field_path: 'data'
            cache_duration: 0.1
        evaluation_function: 'longitudinal_safety_evaluator'
```

### Data Source Configuration

Each proposition can specify multiple data sources:

- **topic**: ROS2 topic name
- **field_path**: Dot-notation path to extract specific fields (empty for full message)
- **data_type**: Optional data type specification
- **transform_function**: Optional data transformation ('abs', 'speed_from_components', etc.)
- **cache_duration**: How long to cache data in seconds

### CTL Formula Types

- **always**: A(G(p)) - Property must always hold
- **eventually**: A(F(p)) - Property must eventually hold
- **never**: A(G(Not(p))) - Property must never hold
- **next**: A(X(p)) - Property holds in next state
- **until**: A(U(p,q)) - Property p holds until q holds

## Usage Examples

### Monitor with Debug Output

```bash
python3 ModelChecker.py --mode online --config config.yaml --vehicle-id 0 --duration 30 --debug
```

### Monitor and Log 
Monitor and log results to a json file:
```bash
python3 ModelChecker.py --mode online --config config.yaml --vehicle-id 0 --output results.json
```

### Debug Mode with Data Recording
```bash
python3 ModelChecker.py --mode online --config config.yaml --vehicle-id 0 --debug-mode --data-file recorded_data.yaml
```

### Analyze Recorded Data
```bash
python3 ModelChecker.py --mode offline --config config.yaml --data-file recorded_data.yaml
```

## Command Line Options

```
usage: ModelChecker.py [-h] --mode {online,offline} --config CONFIG
                       [--bag-file BAG_FILE] [--vehicle-id VEHICLE_ID]
                       [--duration DURATION] [--output OUTPUT]
                       [--create-minimal-config CREATE_MINIMAL_CONFIG]
                       [--debug]

options:
  -h, --help            show this help message and exit
  --mode {online,offline}
                        Monitoring mode
  --config CONFIG       Configuration file path
  --bag-file BAG_FILE   ROS bag file for offline mode
  --vehicle-id VEHICLE_ID
                        Vehicle ID to monitor (online mode)
  --duration DURATION   Monitoring duration in seconds (online mode)
  --output OUTPUT       Output file for results
  --create-minimal-config CREATE_MINIMAL_CONFIG
                        Create minimal sample config file
  --debug               Enable debug logging
```

## Output Format

The tool provides detailed results with statistics:

```json
{
  "SUMMARY": {
    "total_propositions": 2,
    "analyzed": 2,
    "passed": 1,
    "failed": 1,
    "success_rate": 0.5,
    "overall_result": "FAIL"
  },
  "EGO_SPEED": {
    "result": true,
    "status": "PASS",
    "states_analyzed": 49,
    "kripke_states": 49,
    "statistics": {
      "max_velocity": 3.34,
      "average_velocity": 3.03,
      "speed_threshold": 13.89,
      "states_with_data": 34,
      "states_without_data": 15
    }
  },
  "NEAR_GOAL": {
    "result": false,
    "status": "FAIL",
    "states_analyzed": 49,
    "kripke_states": 49,
    "statistics": {
      "goal_position": {"x": 606447.62, "y": 5797244.84},
      "final_vehicle_position": {"x": 606481.58, "y": 5797319.95},
      "min_distance_to_goal": 82.43,
      "final_distance_to_goal": 82.43,
      "goal_threshold": 5.0
    }
  }
}
```

## Extending the Tool

### Adding Custom Propositions

1. Add new proposition types to `PropositionType` enum with appropriate group
2. Implement evaluation logic in `PropositionEvaluators` class
3. Add to default configuration generation
4. Configure data sources in YAML

### Custom Evaluation Functions

Create new static methods in `PropositionEvaluators`:

```python
@staticmethod
def custom_evaluator(data: Dict[str, Any], config: PropositionConfig, 
                     safety_params: SafetyParameters) -> Optional[bool]:
    # Custom evaluation logic
    return True  # or False based on your logic
```

### Data Transformations

Add new transform functions in `DataTransforms.apply_transform()`:

```python
elif transform_function == "custom_transform":
    return custom_transformation(data)
```

## Architecture

- **ConfigLoader**: Handles YAML configuration parsing and validation
- **OnlineMonitor**: ROS2 data collection with topic subscriptions
- **ModelChecker**: Core CTL model checking logic with Kripke structure creation
- **PropositionEvaluators**: Safety proposition evaluation functions
- **VehicleMonitorAnalyzer**: Orchestrates analysis and reporting
- **DataTransforms**: Data extraction and transformation utilities

## Dependencies

- **pyModelChecking**: CTL model checking library
- **ROSMarshaller**: ROS2 interface for topic subscription and data handling
- **pandas/numpy**: Data processing and analysis
- **PyYAML**: Configuration file parsing
- **threading/queue**: Concurrent data collection

## Troubleshooting

### Common Issues

1. **ROS2 not found**: Ensure ROS2 is installed and sourced
2. **Topic not found**: Check topic names with `ros2 topic list`
3. **No data for proposition**: Check topic names and field paths in configuration
4. **Missing evaluation function**: Ensure evaluation_function matches available functions
5. **CTL formula errors**: Verify formula_type is supported

### Debug Tips

- Use `--debug` flag for detailed logging
- Check data source configuration with first few states
- Verify topic publication with `ros2 topic echo`
- Use `--debug-mode` to record data for offline analysis

### Performance Considerations

- Adjust `monitoring_frequency` based on system capabilities
- Set appropriate `cache_duration` for different data sources
- Limit `buffer_size` for memory-constrained systems
- Use field_path extraction to reduce data processing overhead
