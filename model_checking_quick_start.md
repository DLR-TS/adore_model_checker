# Model Checking Quick Start Guide

## Overview
The ADORe Model Checker monitors vehicle safety using Computation Tree Logic 
(CTL) to verify safety properties in real-time or from recorded data.

## Quick Start

### 1. Run Live Vehicle Monitoring
Monitor a live vehicle for 60 seconds using the default configuration:
```bash
python3 adore_model_checker_cli.py --mode online --config default.yaml --vehicle-id 0 --duration 60 --output result.json
```
or if the ADORe model checker package is installed in your system:
```bash
adore-model-checker --mode online --config default.yaml --vehicle-id 0 --duration 60
```

### Debug Mode
Run with detailed logging:
```bash
python3 adore_model_checker_cli.py --mode online --config default.yaml --vehicle-id 0 --duration 30 --debug
```

## Understanding Results

### Console Output
The tool displays real-time results showing:
- **PASS/FAIL** status for each safety proposition
- **Success rate** as percentage of passed checks
- **Statistics** like max velocity, distances, compliance rates

Example output:
```
============================================================
DYNAMIC VEHICLE MONITORING RESULTS
============================================================

SUMMARY:
 Total Propositions: 2
 Analyzed: 2
 Passed: 2
 Failed: 0
 Success Rate: 100.0%
 Overall Result: PASS

DETAILED RESULTS:
 EGO_SPEED                      : PASS     (49 states)
                                    Max velocity: 3.34 m/s
                                    Speed threshold: 13.89 m/s
 NEAR_GOAL                      : PASS     (49 states)
                                    Final distance to goal: 2.85m
                                    Distance threshold: 5.00m
```

### JSON Output
When using `--output results.json`, you get detailed statistics:
```json
{
 "SUMMARY": {
   "total_propositions": 2,
   "analyzed": 2,
   "passed": 2,
   "failed": 0,
   "success_rate": 1.0,
   "overall_result": "PASS"
 },
 "EGO_SPEED": {
   "result": true,
   "status": "PASS",
   "states_analyzed": 49,
   "statistics": {
     "max_velocity": 3.34,
     "average_velocity": 3.03,
     "speed_threshold": 13.89
   }
 }
}
```

## Default Safety Checks

The default configuration monitors:
- **EGO_SPEED**: Vehicle stays under speed limit
- **NEAR_GOAL**: Vehicle eventually reaches goal
- **SAFE_DISTANCE_X/Y**: Maintains safe distances from obstacles
- **TIME_TO_COLLISION**: Avoids imminent collisions
- **LANE_KEEPING**: Stays within lane boundaries

## Common Commands

### Using Python Script Directly
```bash
# Quick 30-second check with results saved
python3 adore_model_checker_cli.py --mode online --config default.yaml --vehicle-id 0 --duration 30 --output check.json

# Debug mode for troubleshooting
python3 adore_model_checker_cli.py --mode online --config default.yaml --vehicle-id 0 --duration 10 --debug
```

### Using Installed Package
If the ADORe model checker package is installed in your system:
```bash
# Quick 30-second check with results saved
adore-model-checker --mode online --config default.yaml --vehicle-id 0 --duration 30 --output check.json

# Debug mode for troubleshooting
adore-model-checker --mode online --config default.yaml --vehicle-id 0 --duration 10 --debug
```

## Result Interpretation

- **PASS**: Safety property was satisfied throughout monitoring
- **FAIL**: Safety property was violated at least once
- **States analyzed**: Number of time steps checked
- **Success rate**: Percentage of propositions that passed
- **Overall result**: PASS if all enabled propositions pass, FAIL otherwise

## For More Technical Information

This quick start covers the basics of running model checks and viewing results. For comprehensive technical documentation, including:

- **Advanced Configuration**: Custom safety propositions, data source mapping, formula types
- **Architecture Details**: Internal components, evaluation functions, data transforms
- **Extending the Tool**: Adding custom propositions and evaluation logic
- **Installation Guide**: System requirements, dependencies, package building
- **Troubleshooting**: Common issues, debug tips, performance considerations
- **Complete API Reference**: All configuration options and command-line parameters

See the full **[README.md](README.md)** documentation.

The README includes detailed examples for:
- Custom YAML configurations
- All available safety proposition categories
- CTL formula types and their use cases
- Data source field path extraction
- Performance tuning and optimization
