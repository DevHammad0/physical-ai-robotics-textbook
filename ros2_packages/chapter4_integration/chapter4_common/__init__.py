"""
Chapter 4 common utilities and configuration module

This package provides shared utilities and configuration for the VLA pipeline:
- Message conversion utilities
- Logging and error handling
- Isaac Sim integration helpers
- Configuration management
"""

from .utils import (
    get_logger,
    detection_to_dict,
    dict_to_detection_dict,
    voice_command_to_dict,
    task_plan_to_dict,
    validate_detection_list,
    validate_plan,
    validate_robot_state,
    ERROR_CODES,
    get_error_message,
    log_voice_command,
    log_detection,
    log_execution_status,
    bbox_to_center,
    pixel_to_3d,
)

from .config import (
    VoiceConfig,
    VisionConfig,
    PlanningConfig,
    ManipulationConfig,
    SafetyConfig,
    Config,
    get_config,
    reset_config,
    get_default_workspace_bounds,
    get_default_joint_limits,
)

__all__ = [
    # Utils
    'get_logger',
    'detection_to_dict',
    'dict_to_detection_dict',
    'voice_command_to_dict',
    'task_plan_to_dict',
    'validate_detection_list',
    'validate_plan',
    'validate_robot_state',
    'ERROR_CODES',
    'get_error_message',
    'log_voice_command',
    'log_detection',
    'log_execution_status',
    'bbox_to_center',
    'pixel_to_3d',
    # Config
    'VoiceConfig',
    'VisionConfig',
    'PlanningConfig',
    'ManipulationConfig',
    'SafetyConfig',
    'Config',
    'get_config',
    'reset_config',
    'get_default_workspace_bounds',
    'get_default_joint_limits',
]
