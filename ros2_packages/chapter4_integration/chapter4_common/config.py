#!/usr/bin/env python3
"""
Shared configuration for Chapter 4 VLA pipeline
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class VoiceConfig:
    """Voice recognition configuration"""
    # Audio capture parameters
    sample_rate: int = 16000  # Hz (Whisper standard)
    channels: int = 1
    dtype: str = 'float32'
    duration: float = 5.0  # Max recording duration in seconds

    # Whisper model selection
    model_name: str = 'base'  # Options: tiny, base, small, medium, large
    language: str = 'en'
    use_fp16: bool = True  # Use half-precision for GPU acceleration

    # Confidence thresholding
    confidence_threshold: float = 0.7  # Minimum confidence to publish

    # Timeout configuration
    recording_timeout: float = 5.0  # Seconds
    transcription_timeout: float = 10.0  # Seconds

    # ROS 2 topic configuration
    voice_command_topic: str = '/robot/voice_command'
    voice_confidence_topic: str = '/robot/voice_confidence'


@dataclass
class VisionConfig:
    """Vision system configuration"""
    # Object detection (YOLO)
    detection_model: str = 'yolov8n.pt'  # Options: yolov8n, yolov8s, yolov8m, yolov8l
    detection_confidence_threshold: float = 0.5
    detection_iou_threshold: float = 0.45
    max_detections: int = 100

    # Segmentation model
    segmentation_model: str = 'fcn_resnet50'  # Options: fcn_resnet50, deeplabv3
    segmentation_threshold: float = 0.5

    # Camera parameters
    camera_topic: str = '/robot/camera/image_raw'
    camera_intrinsics: Tuple[float, float, float, float] = (640.0, 640.0, 320.0, 240.0)  # fx, fy, cx, cy
    image_width: int = 640
    image_height: int = 480

    # ROS 2 topic configuration
    detection_topic: str = '/robot/detections'
    segmentation_topic: str = '/robot/segmentation'
    annotated_image_topic: str = '/robot/camera/annotated'

    # Processing
    use_gpu: bool = True
    batch_size: int = 1
    max_inference_time: float = 0.1  # 100ms max


@dataclass
class PlanningConfig:
    """LLM-based planning configuration"""
    # LLM selection
    llm_backend: str = 'openai'  # Options: openai, ollama
    model_name: str = 'gpt-4'  # For OpenAI, or model name for Ollama

    # API configuration
    api_timeout: float = 30.0  # Seconds
    max_retries: int = 3
    temperature: float = 0.3  # Lower = more deterministic
    max_tokens: int = 500

    # Planning constraints
    max_subtasks: int = 10
    max_plan_duration: float = 60.0  # Seconds
    replanning_enabled: bool = True

    # ROS 2 service/topic configuration
    planning_service: str = '/robot/plan_task'
    validation_service: str = '/robot/validate_plan'

    # Safety validation
    check_workspace_constraints: bool = True
    check_joint_limits: bool = True
    check_gripper_feasibility: bool = True


@dataclass
class ManipulationConfig:
    """Manipulation and grasping configuration"""
    # Grasp planning
    grasp_model: str = 'dexnet'  # Options: dexnet, ggcnn, gq-cnn
    grasp_quality_threshold: float = 0.3
    max_grasp_candidates: int = 5

    # Motion planning
    motion_planner: str = 'rrt'  # Options: rrt, rrt*, prm, trajectory_optimization
    planning_time_limit: float = 5.0  # Seconds
    collision_checking: bool = True

    # Gripper parameters
    gripper_type: str = 'parallel'  # Options: parallel, soft, anthropomorphic
    gripper_max_force: float = 100.0  # Newtons
    gripper_max_width: float = 0.1  # Meters

    # Robot parameters
    robot_base_name: str = 'humanoid'
    arm_dof: int = 7  # Degrees of freedom

    # ROS 2 service/topic configuration
    grasp_generation_service: str = '/robot/generate_grasps'
    motion_planning_service: str = '/robot/plan_motion'
    arm_movement_action: str = '/robot/move_arm'

    # Execution parameters
    cartesian_velocity_limit: float = 0.5  # m/s
    joint_velocity_limit: float = 1.0  # rad/s


@dataclass
class SafetyConfig:
    """Safety monitoring configuration"""
    # Kill switch / E-stop
    emergency_stop_enabled: bool = True
    emergency_stop_timeout: float = 0.1  # Seconds (sub-100ms response)

    # Workspace boundaries
    workspace_min: List[float] = None
    workspace_max: List[float] = None

    # Force limits (Newtons)
    max_gripper_force: float = 100.0
    max_joint_torque: float = 50.0  # Nm

    # Joint angle limits (radians)
    joint_angle_limits_min: List[float] = None
    joint_angle_limits_max: List[float] = None

    # Velocity limits (safe speeds)
    max_cartesian_velocity: float = 0.5  # m/s
    max_joint_velocity: float = 1.0  # rad/s

    # Collision detection
    collision_detection_enabled: bool = True
    collision_check_frequency: float = 100.0  # Hz

    # ROS 2 topic configuration
    safety_event_topic: str = '/robot/safety_events'
    robot_state_topic: str = '/robot/state'
    emergency_stop_topic: str = '/robot/emergency_stop'

    # State machine
    safe_idle_state: Dict = None  # Neutral position to return to


def get_default_workspace_bounds() -> Tuple[List[float], List[float]]:
    """Get default workspace boundaries for humanoid robot"""
    # Min and max [x, y, z] in meters
    min_bounds = [-0.5, -0.5, 0.3]  # Don't go too far from body
    max_bounds = [1.0, 0.5, 2.0]    # Reasonable reach envelope
    return min_bounds, max_bounds


def get_default_joint_limits() -> Tuple[List[float], List[float]]:
    """Get default joint limits for 7-DOF arm (radians)"""
    # Typical industrial robot limits
    min_limits = [-3.14, -1.57, -3.14, -1.57, -3.14, -1.57, -3.14]
    max_limits = [3.14, 1.57, 3.14, 1.57, 3.14, 1.57, 3.14]
    return min_limits, max_limits


class Config:
    """Central configuration object"""
    def __init__(self):
        self.voice = VoiceConfig()
        self.vision = VisionConfig()
        self.planning = PlanningConfig()
        self.manipulation = ManipulationConfig()
        self.safety = SafetyConfig()

        # Set default workspace and joint limits
        ws_min, ws_max = get_default_workspace_bounds()
        self.safety.workspace_min = ws_min
        self.safety.workspace_max = ws_max

        j_min, j_max = get_default_joint_limits()
        self.safety.joint_angle_limits_min = j_min
        self.safety.joint_angle_limits_max = j_max

    def to_dict(self) -> dict:
        """Convert config to dictionary for serialization"""
        return {
            'voice': self.voice.__dict__,
            'vision': self.vision.__dict__,
            'planning': self.planning.__dict__,
            'manipulation': self.manipulation.__dict__,
            'safety': self.safety.__dict__,
        }

    def __repr__(self) -> str:
        return f"Config(voice={self.voice}, vision={self.vision}, planning={self.planning}, " \
               f"manipulation={self.manipulation}, safety={self.safety})"


# Global config instance
_config = None


def get_config() -> Config:
    """Get or create global config instance"""
    global _config
    if _config is None:
        _config = Config()
    return _config


def reset_config() -> None:
    """Reset config to defaults (useful for testing)"""
    global _config
    _config = Config()


if __name__ == '__main__':
    cfg = get_config()
    print(cfg)
    import json
    print(json.dumps(cfg.to_dict(), indent=2, default=str))
