"""
Adore Model Checker - Dynamic ROS Model Checker for Vehicle Safety Monitoring
"""

__version__ = "0.1.0"
__author__ = "Your Name"

# Import main classes for easy access
try:
    from .model_checker import (
        VehicleMonitorAnalyzer,
        ConfigLoader,
        MonitoringConfig,
        MonitoringMode,
        ModelChecker,
        PropositionType,
        PropositionGroup
    )
    from .model_checker_api import (
        ModelCheckAPI,
        get_model_check_blueprint,
        stop_model_check_worker
    )
except ImportError:
    # Handle case where dependencies are not available
    pass

__all__ = [
    'VehicleMonitorAnalyzer',
    'ConfigLoader', 
    'MonitoringConfig',
    'MonitoringMode',
    'ModelChecker',
    'PropositionType',
    'PropositionGroup',
    'ModelCheckAPI',
    'get_model_check_blueprint',
    'stop_model_check_worker'
]
