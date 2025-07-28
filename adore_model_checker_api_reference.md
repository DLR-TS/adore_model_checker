# ADORe Model Checker API Reference

## Overview

The ADORe Model Checker API provides comprehensive model checking capabilities for autonomous vehicle scenarios. It supports both online (real-time ROS2 data) and offline (bag file) analysis modes with configurable safety propositions and monitoring parameters.

## Getting Started

### Installation & Setup

The Model Checker API can be run as a standalone service or integrated into the main ADORe API.

### Running the Standalone API

```bash
# Run the standalone model checker API server
python3 adore_model_checker_api_app.py
```

The API will be available at:
- **Base URL**: `http://localhost:5000`
- **API Endpoints**: `http://localhost:5000/api/model_check/`
- **Health Check**: `http://localhost:5000/health`

### Integration with Main ADORe API

The model checker is automatically integrated when running the main ADORe API:

```bash
# The model checker endpoints are available at:
# http://localhost:8888/api/model_check/
python3 adore_api.py
```

### Configuration

The API automatically manages configuration files and directories:

- **Config Directory**: `~/.config/adore_model_checker/` (or system-appropriate location)
- **Default Config**: `default.yaml` (created automatically if missing)
- **Log Directory**: `logs/model_checker/` (or specified LOG_DIRECTORY)

## API Endpoints

### Start Online Model Checking

**POST** `/api/model_check/online`

Start real-time model checking session monitoring live ROS2 data.

**Request Body:**
```json
{
  "config_file": "default.yaml",
  "duration": 60.0,
  "vehicle_id": 0
}
```

**Parameters:**
- `config_file`: Configuration file name (optional, default: "default.yaml")
- `duration`: Monitoring duration in seconds (optional, default: 60.0)
- `vehicle_id`: Vehicle ID to monitor (optional, default: 0)

**Response:**
```json
{
  "message": "Online model check started",
  "run_id": 12345,
  "config_file": "default.yaml",
  "config_directory": "/home/user/.config/adore_model_checker",
  "duration": 60.0,
  "vehicle_id": 0
}
```

### Start Offline Model Checking

**POST** `/api/model_check/offline`

Start model checking analysis on recorded bag file data.

**Request Body (JSON):**
```json
{
  "config_file": "default.yaml",
  "bag_file": "/path/to/recording.bag"
}
```

**Request Body (Form Upload):**
```
Content-Type: multipart/form-data

config_file: default.yaml
bag_file: [uploaded bag file]
```

**Response:**
```json
{
  "message": "Offline model check started",
  "run_id": 12346,
  "config_file": "default.yaml",
  "config_directory": "/home/user/.config/adore_model_checker",
  "bag_file": "/path/to/recording.bag"
}
```

### Get Model Check Result

**GET** `/api/model_check/result/<run_id>`

Get detailed results and status of a specific model checking run.

**Response:**
```json
{
  "run_id": 12345,
  "mode": "online",
  "status": "completed",
  "config_file": "/path/to/config.yaml",
  "start_time": "2025-07-28T10:30:00Z",
  "end_time": "2025-07-28T10:31:00Z",
  "duration": 60.0,
  "vehicle_id": 0,
  "ready": true,
  "results": {
    "SUMMARY": {
      "total_propositions": 5,
      "passed": 4,
      "failed": 1,
      "success_rate": 0.8,
      "overall_result": "PASS"
    },
    "EGO_SPEED": {
      "status": "pass",
      "description": {
        "title": "Speed Limit Compliance",
        "description": "Vehicle maintains safe speed limits",
        "safety_rationale": "Prevents accidents due to excessive speed"
      },
      "formula_description": "Always speed < threshold",
      "result": 0.95,
      "threshold": 13.89,
      "states_analyzed": 600,
      "kripke_states": 150
    }
  },
  "stdout": "Model checking output...",
  "stderr": ""
}
```

**Status Values:**
- `pending`: Run queued but not started
- `running`: Currently executing
- `completed`: Successfully finished
- `failed`: Execution failed
- `cancelled`: Manually cancelled

### Get All Results

**GET** `/api/model_check/results`

Get summary of all model checking runs.

**Response:**
```json
{
  "results": [
    {
      "run_id": 12345,
      "mode": "online",
      "status": "completed",
      "start_time": "2025-07-28T10:30:00Z",
      "end_time": "2025-07-28T10:31:00Z",
      "config_file": "/path/to/config.yaml",
      "duration": 60.0,
      "vehicle_id": 0,
      "ready": true
    }
  ],
  "count": 5,
  "running": 1,
  "completed": 3,
  "failed": 1
}
```

### Download Result File

**GET** `/api/model_check/result/<run_id>/download`

Download the complete model checking results as a JSON file.

**Response:**
File download with filename: `model_check_results_{run_id}.json`

### Download Log File

**GET** `/api/model_check/result/<run_id>/log`

Download the execution log file for a specific run.

**Response:**
File download with filename: `model_check_log_{run_id}.log`

### Cancel Model Check Run

**POST** `/api/model_check/cancel/<run_id>`

Cancel a pending or running model check job.

**Response:**
```json
{
  "message": "Run 12345 cancelled"
}
```

### Get API Status

**GET** `/api/model_check/status`

Get current status of the model checker API and worker threads.

**Response:**
```json
{
  "worker_running": true,
  "total_runs": 10,
  "running_runs": 2,
  "active_processes": 2,
  "log_directory": "/path/to/logs/model_checker",
  "config_directory": "/home/user/.config/adore_model_checker",
  "default_config": "default.yaml"
}
```

### List Available Configurations

**GET** `/api/model_check/config/list`

List all available configuration files in the config directory.

**Response:**
```json
{
  "config_directory": "/home/user/.config/adore_model_checker",
  "available_configs": [
    "default.yaml",
    "safety_critical.yaml",
    "performance_test.yaml"
  ],
  "default_config": "default.yaml"
}
```

## Configuration Files

### Config File Structure

Configuration files are written in YAML format and define monitoring parameters and safety propositions:

```yaml
monitoring:
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
        description: "Core safety propositions"

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
```

### Supported Propositions

Built-in safety propositions include:

#### EGO_SPEED
- **Purpose**: Speed limit compliance
- **Formula**: Always (speed < threshold)
- **Data Source**: `/ego_vehicle/vehicle_state/dynamic.vx`

#### NEAR_GOAL  
- **Purpose**: Goal reaching verification
- **Formula**: Eventually (distance_to_goal < threshold)
- **Data Sources**: Route and vehicle state

#### DECELERATION
- **Purpose**: Safe deceleration limits
- **Formula**: Always (deceleration > threshold)
- **Data Source**: `/ego_vehicle/vehicle_state/dynamic.ax`

#### ACCELERATION_COMPLIANCE
- **Purpose**: Acceleration command following
- **Formula**: Always (|measured - commanded| < threshold)
- **Data Sources**: Measured and commanded acceleration

#### DECELERATION_COMPLIANCE
- **Purpose**: Deceleration command following  
- **Formula**: Always (|measured - commanded| < threshold)
- **Data Sources**: Measured and commanded acceleration

### Formula Types

- `always`: Property must hold at all times (□p)
- `eventually`: Property must eventually become true (◊p)
- `until`: Property must hold until another becomes true (p U q)
- `next`: Property must hold in the next state (○p)

## ROS2 Topic Integration

### Required Topics for Online Mode

The model checker subscribes to these ROS2 topics:

#### Vehicle State
- **Topic**: `/ego_vehicle/vehicle_state/dynamic`
- **Type**: `adore_if_ros_msg/msg/VehicleExtendedState`
- **Fields**: `vx` (velocity), `ax` (acceleration), position data

#### Route Information
- **Topic**: `/ego_vehicle/route`
- **Type**: Route message type
- **Purpose**: Goal position and path planning

#### Vehicle Commands
- **Topic**: `/ego_vehicle/next_vehicle_command`
- **Type**: Command message type
- **Fields**: `acceleration` (commanded acceleration)

#### Brake Status
- **Topic**: `/ego_vehicle/brake_status`
- **Type**: Brake status message type
- **Fields**: `active` (brake activation state)

## Error Handling

All endpoints return appropriate HTTP status codes:

- **200 OK**: Successful operation
- **400 Bad Request**: Invalid request parameters or missing required data
- **404 Not Found**: Run ID or resource not found
- **500 Internal Server Error**: Internal processing error

### Common Error Responses

```json
{
  "error": "Run 12345 not found"
}
```

```json
{
  "error": "bag_file required"
}
```

```json
{
  "error": "Bag file not found: /path/to/file.bag"
}
```

## Usage Examples

### Example 1: Start Online Model Checking

```bash
curl -X POST http://localhost:5000/api/model_check/online \
  -H "Content-Type: application/json" \
  -d '{
    "config_file": "default.yaml",
    "duration": 120.0,
    "vehicle_id": 0
  }'
```

### Example 2: Check Run Status

```bash
curl http://localhost:5000/api/model_check/result/12345
```

### Example 3: Start Offline Analysis

```bash
curl -X POST http://localhost:5000/api/model_check/offline \
  -H "Content-Type: application/json" \
  -d '{
    "config_file": "safety_critical.yaml",
    "bag_file": "/path/to/scenario_recording.bag"
  }'
```

### Example 4: Upload and Analyze Bag File

```bash
curl -X POST http://localhost:5000/api/model_check/offline \
  -F "config_file=default.yaml" \
  -F "bag_file=@/local/path/to/recording.bag"
```

### Example 5: Download Results

```bash
curl http://localhost:5000/api/model_check/result/12345/download \
  -o model_check_results.json
```

## Integration Notes

### Directory Structure

```
~/.config/adore_model_checker/
├── default.yaml
├── safety_critical.yaml
└── custom_config.yaml

logs/model_checker/
├── model_check_run_12345_20250728T103000Z_results.json
├── model_check_run_12345_20250728T103000Z.log
└── uploaded_20250728_120000_recording.bag
```

### Concurrency

- Maximum 2 concurrent model checking runs (configurable)
- Runs are queued when capacity is exceeded
- Thread-safe run management and result caching

### Logging

- All runs generate detailed log files
- Stdout/stderr captured during execution
- Timestamped file naming with ISO8601 format
- Automatic log directory creation and management

## Health Check

**GET** `/health`

Simple health check endpoint for monitoring API availability.

**Response:**
```json
{
  "status": "healthy"
}
```

## Root Endpoint

**GET** `/`

Get API information and available endpoints.

**Response:**
```json
{
  "message": "Adore Model Checker API",
  "version": "0.1.0",
  "endpoints": {
    "model_check": "/api/model_check/",
    "status": "/api/model_check/status",
    "results": "/api/model_check/results"
  }
}
```
