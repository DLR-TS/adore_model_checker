# Model Checker API Quick Start

## Overview
REST API interface for invoking the ADORe model checker programmatically.

## Starting the API
```bash
python3 adore_model_checker_api_app.py
```
## API Endpoints
- **Standalone**: `http://localhost:5000/api/model_check/`

> **Note:** Port numbers may differ based on your configuration. Check the console output when starting the API for the actual port.


## Basic Usage

### Start Online Monitoring
```bash
curl -X POST http://localhost:5000/api/model_check/online \
  -H "Content-Type: application/json" \
  -d '{
    "config_file": "default.yaml",
    "duration": 60.0,
    "vehicle_id": 0
  }'
```

**Response:**
```json
{
  "message": "Online model check started",
  "run_id": 12345,
  "config_file": "default.yaml",
  "duration": 60.0,
  "vehicle_id": 0
}
```

### Check Results
```bash
curl http://localhost:5000/api/model_check/result/12345
```

**Response:**
```json
{
  "run_id": 12345,
  "status": "completed",
  "ready": true,
  "results": {
    "SUMMARY": {
      "success_rate": 1.0,
      "overall_result": "PASS"
    },
    "EGO_SPEED": {
      "status": "pass",
      "result": 0.95
    }
  }
}
```

### Start Offline Analysis
```bash
curl -X POST http://localhost:5000/api/model_check/offline \
  -H "Content-Type: application/json" \
  -d '{
    "config_file": "default.yaml",
    "bag_file": "/path/to/recording.bag"
  }'
```

### Upload Bag File
```bash
curl -X POST http://localhost:5000/api/model_check/offline \
  -F "config_file=default.yaml" \
  -F "bag_file=@recording.bag"
```

### Download Results
```bash
curl http://localhost:5000/api/model_check/result/12345/download \
  -o results.json
```

### Cancel Run
```bash
curl -X POST http://localhost:5000/api/model_check/cancel/12345
```

## Run Status
- `pending`: Queued but not started
- `running`: Currently executing  
- `completed`: Successfully finished
- `failed`: Execution failed

## API Reference
For complete endpoint documentation see: **[Model Checker API Reference](adore_model_checker_api_reference.md)**
