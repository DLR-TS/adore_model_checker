# Adore Model Checker API Reference

## Running the AORe API app
To run the ADORe API use the following command:
```
python3 adore_model_checker_api_app.py
```

## Base URL
```
http://localhost:5000/api/model_check
```

## Authentication
Currently no authentication required.

> ⚠️ **Danger!**
> 
> The ADORe API is intended for software recherche only. Do not run it on a 
> publicly facing network or system. The API **HAS NOT** been evaluated for 
> security.


## Endpoints

### Start Online Model Checking

Start a new online model checking session that monitors live ROS2 data.

**Endpoint:** `POST /api/model_check/online`


**Example:**
```
curl -X POST http://localhost:5000/api/model_check/online \
     -H "Content-Type: application/json" \
     -d '{}'
```
**Request Body:**
```json
{
  "config_file": "config/default.yaml",
  "duration": 60.0,
  "vehicle_id": 0
}
```

**Parameters:**
| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| config_file | string | No | "config/default.yaml" | Path to YAML configuration file |
| duration | float | No | 60.0 | Monitoring duration in seconds |
| vehicle_id | integer | No | 0 | ID of vehicle to monitor |

**Response:**
```json
{
  "message": "Online model check started",
  "run_id": 0,
  "config_file": "config/default.yaml",
  "duration": 60.0,
  "vehicle_id": 0
}
```

**Status Codes:**
- `200` - Success
- `400` - Bad request (invalid parameters or config file not found)
- `500` - Internal server error

---

### Start Offline Model Checking

Start a new offline model checking session using a ROS bag file.

**Endpoint:** `POST /api/model_check/offline`

#### Option 1: JSON with file path

**Request Body:**
```json
{
  "config_file": "config/default.yaml",
  "bag_file": "/path/to/bagfile.bag"
}
```

#### Option 2: File upload

**Request:** `multipart/form-data`

**Form Fields:**
| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| config_file | string | No | "config/default.yaml" | Path to YAML configuration file |
| bag_file | file | Yes | - | ROS bag file to analyze |

**Response:**
```json
{
  "message": "Offline model check started",
  "run_id": 1,
  "config_file": "config/default.yaml",
  "bag_file": "/path/to/bagfile.bag"
}
```

**Status Codes:**
- `200` - Success
- `400` - Bad request (missing bag file, config file not found)
- `500` - Internal server error

---

### Get Specific Result

Retrieve the result of a specific model checking run.

**Endpoint:** `GET /api/model_check/result/<run_id>`

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| run_id | integer | Yes | ID of the model checking run |

**Response:**
```json
{
  "run_id": 0,
  "mode": "online",
  "status": "completed",
  "config_file": "config/default.yaml",
  "start_time": "2023-12-07T10:30:00.000000+00:00",
  "end_time": "2023-12-07T10:31:00.000000+00:00",
  "duration": 60.0,
  "vehicle_id": 0,
  "stdout": "Starting online monitoring...\n",
  "stderr": "",
  "ready": true,
  "results": {
    "NEAR_GOAL": {
      "result": true,
      "status": "PASS",
      "states_analyzed": 600,
      "kripke_states": 600,
      "statistics": {
        "goal_position": {"x": 100.0, "y": 50.0},
        "final_distance_to_goal": 2.5,
        "min_distance_to_goal": 2.5
      }
    },
    "SUMMARY": {
      "total_propositions": 3,
      "analyzed": 3,
      "passed": 2,
      "failed": 1,
      "success_rate": 0.67,
      "overall_result": "FAIL"
    }
  }
}
```

**Response Fields:**
| Field | Type | Description |
|-------|------|-------------|
| run_id | integer | Unique identifier for the run |
| mode | string | "online" or "offline" |
| status | string | "pending", "running", "completed", "failed", "cancelled" |
| config_file | string | Path to configuration file used |
| start_time | string | ISO8601 timestamp when run started |
| end_time | string | ISO8601 timestamp when run ended |
| stdout | string | Standard output from the model checker |
| stderr | string | Standard error output from the model checker |
| ready | boolean | Whether results are ready for consumption |
| results | object | Model checking results (only present when status is "completed") |
| error_message | string | Error message (only present when status is "failed") |

**Status Codes:**
- `200` - Success
- `404` - Run not found

---

### Get All Results

Retrieve a summary of all model checking runs.

**Endpoint:** `GET /api/model_check/results`

**Response:**
```json
{
  "results": [
    {
      "run_id": 0,
      "mode": "online",
      "status": "completed",
      "start_time": "2023-12-07T10:30:00.000000+00:00",
      "end_time": "2023-12-07T10:31:00.000000+00:00",
      "config_file": "config/default.yaml",
      "duration": 60.0,
      "vehicle_id": 0,
      "ready": true
    },
    {
      "run_id": 1,
      "mode": "offline",
      "status": "running",
      "start_time": "2023-12-07T10:35:00.000000+00:00",
      "end_time": null,
      "config_file": "config/default.yaml",
      "bag_file": "/path/to/bagfile.bag",
      "ready": false
    }
  ],
  "count": 2,
  "running": 1,
  "completed": 1,
  "failed": 0
}
```

**Status Codes:**
- `200` - Success

---

### Download Result File

Download the complete results as a JSON file.

**Endpoint:** `GET /api/model_check/result/<run_id>/download`

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| run_id | integer | Yes | ID of the model checking run |

**Response:**
- File download with `Content-Type: application/json`
- Filename: `model_check_results_<run_id>.json`

**Status Codes:**
- `200` - Success (file download)
- `400` - Run not completed
- `404` - Run not found or result file not found

---

### Download Log File

Download the log file for a specific run.

**Endpoint:** `GET /api/model_check/result/<run_id>/log`

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| run_id | integer | Yes | ID of the model checking run |

**Response:**
- File download with `Content-Type: text/plain`
- Filename: `model_check_log_<run_id>.log`

**Status Codes:**
- `200` - Success (file download)
- `404` - Run not found or log file not found

---

### Get API Status

Get the current status of the model checker API service.

**Endpoint:** `GET /api/model_check/status`

**Response:**
```json
{
  "worker_running": true,
  "total_runs": 5,
  "running_runs": 1,
  "active_processes": 1,
  "log_directory": "logs",
  "default_config": "config/default.yaml"
}
```

**Status Codes:**
- `200` - Success

---

### Cancel Run

Cancel a pending or running model checking job.

**Endpoint:** `POST /api/model_check/cancel/<run_id>`

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| run_id | integer | Yes | ID of the model checking run to cancel |

**Response:**
```json
{
  "message": "Run 0 cancelled"
}
```

**Status Codes:**
- `200` - Success
- `400` - Run cannot be cancelled (already completed/failed)
- `404` - Run not found

---

## Status Values

### Run Status
- `pending` - Run is queued but not yet started
- `running` - Run is currently executing
- `completed` - Run finished successfully
- `failed` - Run encountered an error and failed
- `cancelled` - Run was cancelled by user request

### Proposition Status
- `PASS` - Proposition was satisfied
- `FAIL` - Proposition was violated  
- `NO_DATA` - Insufficient data to evaluate proposition
- `ERROR` - Error occurred during evaluation

---

## Example Usage

### Start Online Model Checking
```bash
curl -X POST http://localhost:5000/api/model_check/online \
  -H "Content-Type: application/json" \
  -d '{
    "duration": 120,
    "vehicle_id": 1,
    "config_file": "config/vehicle1.yaml"
  }'
```

### Upload Bag File for Offline Analysis
```bash
curl -X POST http://localhost:5000/api/model_check/offline \
  -F "bag_file=@/path/to/bagfile.bag" \
  -F "config_file=config/default.yaml"
```

### Check Result Status
```bash
curl http://localhost:5000/api/model_check/result/0
```

### Get All Results
```bash
curl http://localhost:5000/api/model_check/results
```

### Download Results
```bash
curl -O http://localhost:5000/api/model_check/result/0/download
```

### Cancel Running Job
```bash
curl -X POST http://localhost:5000/api/model_check/cancel/0
```

---

## Error Handling

All API endpoints return appropriate HTTP status codes and error messages in JSON format:

```json
{
  "error": "Description of the error that occurred"
}
```

Common error scenarios:
- Configuration file not found
- Bag file not found or inaccessible  
- Invalid parameters
- Run not found
- Internal processing errors
- File system errors

---

## File Management

### Output Files
- Results are saved as JSON files in the configured log directory
- Filenames follow the pattern: `model_check_run_<id>_<timestamp>_results.json`
- Files use ISO8601 timestamps in UTC timezone

### Log Files  
- Complete execution logs are saved for each run
- Filenames follow the pattern: `model_check_run_<id>_<timestamp>.log`
- Include stdout, stderr, and execution metadata

### Cleanup
- Files are not automatically cleaned up
- Manual cleanup of old files may be necessary for long-running deployments
