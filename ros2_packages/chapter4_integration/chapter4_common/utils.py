#!/usr/bin/env python3
"""
Shared utility functions for Chapter 4 VLA pipeline
"""

import json
import logging
from datetime import datetime
from typing import Dict, List, Any, Tuple
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def get_logger(name: str) -> logging.Logger:
    """Get a configured logger instance"""
    return logging.getLogger(name)


# ============================================================================
# Message Conversion Utilities
# ============================================================================

def detection_to_dict(detection) -> Dict[str, Any]:
    """Convert ROS 2 DetectionResult message to dict"""
    return {
        'class_name': detection.class_name,
        'confidence': float(detection.confidence),
        'bbox': {
            'x1': detection.x1,
            'y1': detection.y1,
            'x2': detection.x2,
            'y2': detection.y2,
            'width': detection.width,
            'height': detection.height
        }
    }


def dict_to_detection_dict(data: Dict[str, Any]) -> Dict[str, Any]:
    """Validate and normalize detection dictionary format"""
    required_fields = ['class_name', 'confidence', 'bbox']
    bbox_fields = ['x1', 'y1', 'x2', 'y2', 'width', 'height']

    if not all(field in data for field in required_fields):
        raise ValueError(f"Missing required fields: {required_fields}")

    bbox = data['bbox']
    if not all(field in bbox for field in bbox_fields):
        raise ValueError(f"Missing bbox fields: {bbox_fields}")

    return {
        'class_name': str(data['class_name']),
        'confidence': float(data['confidence']),
        'bbox': {
            'x1': int(bbox['x1']),
            'y1': int(bbox['y1']),
            'x2': int(bbox['x2']),
            'y2': int(bbox['y2']),
            'width': int(bbox['width']),
            'height': int(bbox['height'])
        }
    }


def voice_command_to_dict(voice_cmd) -> Dict[str, Any]:
    """Convert ROS 2 VoiceCommand message to dict"""
    return {
        'text': voice_cmd.text,
        'confidence': float(voice_cmd.confidence),
        'timestamp': voice_cmd.timestamp.sec + voice_cmd.timestamp.nanosec / 1e9
    }


def task_plan_to_dict(plan) -> Dict[str, Any]:
    """Convert ROS 2 TaskPlan message to dict"""
    return {
        'task_id': plan.task_id,
        'task_description': plan.task_description,
        'subtasks': list(plan.subtasks),
        'subtask_descriptions': list(plan.subtask_descriptions),
        'estimated_duration': float(plan.estimated_duration),
        'is_feasible': plan.is_feasible,
        'feasibility_reason': plan.feasibility_reason,
        'generated_timestamp': plan.generated_timestamp.sec + plan.generated_timestamp.nanosec / 1e9
    }


# ============================================================================
# Isaac Sim Utilities
# ============================================================================

def load_isaac_sim_scene(scene_path: str) -> bool:
    """Load Isaac Sim scene from YAML path"""
    logger = get_logger(__name__)
    try:
        import yaml
        with open(scene_path, 'r') as f:
            scene_config = yaml.safe_load(f)
        logger.info(f"Loaded Isaac Sim scene: {scene_path}")
        return True
    except Exception as e:
        logger.error(f"Failed to load Isaac Sim scene: {e}")
        return False


def validate_robot_state(state) -> Tuple[bool, str]:
    """Validate robot state for safety"""
    logger = get_logger(__name__)

    try:
        # Check for NaN values
        joint_positions = np.array(state.joint_positions)
        if np.any(np.isnan(joint_positions)):
            return False, "NaN in joint positions"

        # Check gripper force limits
        if state.gripper_force > 100.0:  # Arbitrary max force
            return False, f"Gripper force {state.gripper_force} exceeds limit"

        # Check if in safe zone
        if not state.is_in_safe_zone:
            return False, "Robot outside safe workspace"

        # Check for error status
        if state.error_status:
            return False, f"Robot error: {state.error_status}"

        logger.debug("Robot state validated successfully")
        return True, "OK"

    except Exception as e:
        logger.error(f"Error validating robot state: {e}")
        return False, str(e)


# ============================================================================
# Error Code Utilities
# ============================================================================

ERROR_CODES = {
    'SUCCESS': 0,
    'VOICE_TRANSCRIPTION_FAILED': 1001,
    'VISION_DETECTION_FAILED': 1002,
    'VISION_SEGMENTATION_FAILED': 1003,
    'PLAN_INVALID': 2001,
    'PLAN_UNREACHABLE': 2002,
    'GRASP_GENERATION_FAILED': 3001,
    'MOTION_PLANNING_FAILED': 3002,
    'EXECUTION_FAILED': 4001,
    'SAFETY_VIOLATION': 5001,
    'TIMEOUT': 6001,
    'UNKNOWN_ERROR': 9999
}


def get_error_message(error_code: int) -> str:
    """Get human-readable error message from code"""
    for name, code in ERROR_CODES.items():
        if code == error_code:
            return name
    return "UNKNOWN_ERROR"


# ============================================================================
# Logging Utilities
# ============================================================================

def log_voice_command(text: str, confidence: float) -> None:
    """Log voice command with timestamp"""
    logger = get_logger(__name__)
    logger.info(f"Voice command: '{text}' (confidence: {confidence:.2f})")


def log_detection(detections: List[Dict[str, Any]]) -> None:
    """Log detected objects"""
    logger = get_logger(__name__)
    for det in detections:
        logger.info(f"Detected: {det['class_name']} (conf: {det['confidence']:.2f})")


def log_execution_status(task_id: str, status: str, progress: float) -> None:
    """Log task execution status"""
    logger = get_logger(__name__)
    logger.info(f"Task {task_id}: {status} ({progress:.1f}%)")


# ============================================================================
# Data Validation Utilities
# ============================================================================

def validate_detection_list(detections: List[Dict[str, Any]]) -> bool:
    """Validate detection list format"""
    try:
        for det in detections:
            dict_to_detection_dict(det)
        return True
    except ValueError:
        return False


def validate_plan(plan_dict: Dict[str, Any]) -> bool:
    """Validate task plan format"""
    required_fields = ['task_id', 'task_description', 'subtasks', 'is_feasible']
    return all(field in plan_dict for field in required_fields)


# ============================================================================
# Geometry Utilities
# ============================================================================

def bbox_to_center(bbox: Dict[str, float]) -> Tuple[float, float]:
    """Convert bounding box to center coordinates"""
    center_x = (bbox['x1'] + bbox['x2']) / 2.0
    center_y = (bbox['y1'] + bbox['y2']) / 2.0
    return center_x, center_y


def pixel_to_3d(pixel_x: float, pixel_y: float, depth: float,
                fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float, float]:
    """Project 2D pixel + depth to 3D position using camera intrinsics"""
    x_3d = (pixel_x - cx) * depth / fx
    y_3d = (pixel_y - cy) * depth / fy
    z_3d = depth
    return x_3d, y_3d, z_3d


def pose_to_list(pose) -> List[float]:
    """Convert pose message to [x, y, z, qx, qy, qz, qw] list"""
    return [
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    ]


if __name__ == '__main__':
    logger = get_logger(__name__)
    logger.info("Chapter 4 common utilities module loaded")
