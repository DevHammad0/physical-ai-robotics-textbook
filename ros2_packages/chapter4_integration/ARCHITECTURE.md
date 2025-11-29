# Chapter 4 VLA System Architecture

## Overview

The Vision-Language-Action (VLA) system integrates multiple AI/ML components into a cohesive robot control pipeline. This document describes the ROS 2 node interfaces, topic/service contracts, and message flow.

## System Architecture Diagram

```
┌────────────────────────────────────────────────────────────────────┐
│                   Vision-Language-Action Pipeline                  │
└────────────────────────────────────────────────────────────────────┘

┌──────────────┐
│  Microphone  │
└──────┬───────┘
       │ Audio Stream
       ↓
┌─────────────────────────────┐
│  whisper_ros2_node          │ [Lesson 2: Voice Input]
├─────────────────────────────┤
│ • Subscribes to: audio      │
│ • Publishes to:             │
│   - /robot/voice_command    │
│   - /robot/voice_confidence │
└──────┬──────────────────────┘
       │ VoiceCommand
       ↓
┌──────────────┐              ┌──────────────┐
│   Camera     │              │ Depth Sensor │
└──────┬───────┘              └──────┬───────┘
       │ RGB Image                   │ Depth Data
       │                             │
       ├─────────┬───────────────────┤
       │         │                   │
       ↓         ↓                   ↓
┌──────────────────┐        ┌──────────────────┐
│ yolo_detector_   │        │ segmentation_    │
│ node             │        │ node             │
├──────────────────┤        ├──────────────────┤
│ Subscribes to:   │        │ Subscribes to:   │
│ /robot/camera/   │        │ /robot/camera/   │
│  image_raw       │        │  image_raw       │
│ Publishes to:    │        │ Publishes to:    │
│ /robot/          │        │ /robot/          │
│  detections      │        │  segmentation    │
└─────┬────────────┘        └────┬─────────────┘
      │ DetectionArray           │ SegmentationMask
      │                          │
      └──────────┬───────────────┘
                 │ Perception Results
                 ↓
         ┌──────────────────┐
         │ llm_planner_node │ [Lesson 4: LLM Planning]
         ├──────────────────┤
         │ Subscribes to:   │
         │ /robot/          │
         │  voice_command   │
         │ /robot/          │
         │  detections      │
         │ Publishes to:    │
         │ /robot/task_plan │
         └────┬─────────────┘
              │ TaskPlan
              ↓
         ┌──────────────────┐
         │ grasp_planner_   │ [Lesson 5: Manipulation]
         │ node             │
         ├──────────────────┤
         │ Services:        │
         │ /robot/          │
         │  generate_grasps │
         │ Publishes to:    │
         │ /robot/          │
         │  grasps_offered  │
         └────┬─────────────┘
              │ GraspCandidate[]
              ↓
         ┌──────────────────┐
         │ motion_planning_ │
         │ node             │
         ├──────────────────┤
         │ Services:        │
         │ /robot/          │
         │  plan_motion     │
         │ Publishes to:    │
         │ /robot/          │
         │  planned_path    │
         └────┬─────────────┘
              │ Trajectory
              ↓
         ┌──────────────────┐
         │ orchestrator_    │ [Lesson 6: Integration]
         │ node             │
         ├──────────────────┤
         │ Coordinates all  │
         │ components       │
         │ Actions:         │
         │ /robot/          │
         │  execute_task    │
         └────┬─────────────┘
              │
              ↓
        ┌──────────┐
        │   Robot  │
        │  (Isaac  │
        │   Sim)   │
        └──────────┘
              │
              ↓
         ┌──────────────────┐
         │ safety_validator │ [Lesson 7: Safety]
         │ _node            │
         ├──────────────────┤
         │ Subscribes to:   │
         │ /robot/state     │
         │ Publishes to:    │
         │ /robot/          │
         │  safety_events   │
         └──────────────────┘
```

## ROS 2 Node Interfaces

### 1. Voice Input Node (whisper_ros2_node)

**Purpose**: Capture microphone audio and transcribe to text using OpenAI Whisper

**Subscriptions**:
- None (uses system microphone directly)

**Publications**:
- `/robot/voice_command` → `VoiceCommand` message
  - `text`: Transcribed command text
  - `confidence`: Confidence score [0.0, 1.0]
  - `timestamp`: When transcription completed

- `/robot/voice_confidence` → `Float32` message
  - Single confidence value for monitoring

**Services**: None

**Actions**: None

**Parameters**:
- `model_name`: "base" (default), "small", "medium", "large"
- `confidence_threshold`: 0.7 (default)
- `recording_duration`: 5.0 seconds

### 2. Object Detection Node (yolo_detector_node)

**Purpose**: Detect objects in camera image using YOLO-v8

**Subscriptions**:
- `/robot/camera/image_raw` → `sensor_msgs/Image`
  - RGB camera stream from Isaac Sim

**Publications**:
- `/robot/detections` → `DetectionArray` message
  - `timestamp`: When detection completed
  - `image_height`, `image_width`: Image dimensions
  - `detections[]`: Array of DetectionResult
    - `class_name`: Detected object class
    - `confidence`: Detection confidence [0.0, 1.0]
    - `x1, y1, x2, y2`: Bounding box pixels
    - `width, height`: Box dimensions

- `/robot/camera/annotated` → `sensor_msgs/Image`
  - Visualization with bounding boxes drawn

**Services**: None

**Actions**: None

**Parameters**:
- `model_name`: "yolov8n.pt" (default, nano model)
- `confidence_threshold`: 0.5
- `use_gpu`: true

### 3. Segmentation Node (segmentation_node)

**Purpose**: Perform semantic segmentation to classify pixels

**Subscriptions**:
- `/robot/camera/image_raw` → `sensor_msgs/Image`

**Publications**:
- `/robot/segmentation` → `sensor_msgs/Image`
  - Color-coded segmentation mask
  - Each pixel maps to class ID

**Services**: None

**Actions**: None

**Parameters**:
- `model_name`: "fcn_resnet50"
- `threshold`: 0.5

### 4. LLM Planner Node (llm_planner_node)

**Purpose**: Convert high-level voice commands to executable task plans

**Subscriptions**:
- `/robot/voice_command` → `VoiceCommand`
- `/robot/detections` → `DetectionArray`
- `/robot/state` → `RobotState`

**Publications**:
- `/robot/task_plan` → `TaskPlan` message
  - `task_id`: Unique plan ID
  - `task_description`: Original goal
  - `subtasks[]`: List of executable subtasks
  - `subtask_descriptions[]`: Human-readable descriptions
  - `is_feasible`: Whether plan respects constraints
  - `feasibility_reason`: Why infeasible (if applicable)

**Services**:
- `/robot/plan_task` → `PlanTask` service
  - Request: task description + detected objects
  - Response: TaskPlan + feasibility

- `/robot/validate_plan` → `ValidatePlan` service
  - Request: TaskPlan + current robot state
  - Response: validity + constraint violations

**Actions**: None

**Parameters**:
- `llm_backend`: "openai"
- `model_name`: "gpt-4"
- `temperature`: 0.3
- `max_subtasks`: 10

### 5. Grasp Planner Node (grasp_planner_node)

**Purpose**: Generate grasping strategies for detected objects

**Subscriptions**:
- `/robot/detections` → `DetectionArray`

**Publications**:
- `/robot/grasps_offered` → `GraspCandidate[]`
  - Array of possible grasp approaches

**Services**:
- `/robot/generate_grasps` → `GenerateGrasps` service
  - Request: DetectionResult + 3D object position/size
  - Response: GraspCandidate[] + feasibility

**Actions**: None

**Parameters**:
- `grasp_model`: "dexnet"
- `grasp_quality_threshold`: 0.3

### 6. Motion Planning Node (motion_planning_node)

**Purpose**: Plan collision-free arm trajectories

**Subscriptions**:
- `/robot/state` → `RobotState` (current pose)

**Publications**:
- `/robot/planned_path` → `trajectory_msgs/JointTrajectory`
  - Series of joint angle waypoints

**Services**:
- `/robot/plan_motion` → `PlanMotion` service
  - Request: start pose + goal pose
  - Response: trajectory + feasibility

**Actions**: None

**Parameters**:
- `planner_type`: "rrt"
- `planning_time_limit`: 5.0 seconds

### 7. Arm Control Node (arm_control_node)

**Purpose**: Execute joint trajectories on robot

**Subscriptions**: None

**Publications**: None

**Services**: None

**Actions**:
- `/robot/move_arm` → `MoveArm` action
  - Goal: target position + orientation
  - Feedback: current progress, joint positions
  - Result: success + error message

**Parameters**:
- `velocity_scale`: 0.5 (% of max velocity)
- `acceleration_scale`: 0.5

### 8. Orchestrator Node (orchestrator_node)

**Purpose**: Coordinate all components into cohesive system

**Subscriptions**:
- `/robot/voice_command` → `VoiceCommand`
- `/robot/detections` → `DetectionArray`
- `/robot/task_plan` → `TaskPlan`
- `/robot/safety_events` → `SafetyEvent`

**Publications**:
- `/robot/execution_status` → `ExecutionStatus`
  - Current task progress
  - Error information if failed

**Services**: None

**Actions**:
- `/robot/execute_task` → `ExecuteTask` action
  - Goal: task_id + plan
  - Feedback: status updates
  - Result: success + final status

**Parameters**:
- `replanning_enabled`: true
- `max_retries`: 3

### 9. Safety Validator Node (safety_validator_node)

**Purpose**: Monitor execution for safety violations

**Subscriptions**:
- `/robot/state` → `RobotState`
- `/robot/execution_status` → `ExecutionStatus`
- `/robot/task_plan` → `TaskPlan`

**Publications**:
- `/robot/safety_events` → `SafetyEvent`
  - Safety violations, warnings, critical events
- `/robot/emergency_stop` → `Bool`
  - Emergency stop signal

**Services**: None

**Actions**: None

**Parameters**:
- `emergency_stop_enabled`: true
- `collision_check_frequency`: 100.0 Hz
- `workspace_min`: [-0.5, -0.5, 0.3]
- `workspace_max`: [1.0, 0.5, 2.0]

## Message Flow Scenarios

### Scenario 1: Voice-Controlled Object Detection

```
User speaks: "Find the red cube"
              ↓
whisper_node → VoiceCommand("/robot/voice_command")
              ↓
yolo_detector_node → DetectionArray("/robot/detections")
              ↓
(Application processes detections and responds to user)
```

**Latency Budget**:
- Whisper transcription: <500ms
- YOLO detection: <100ms
- Total: <600ms

### Scenario 2: Task Planning & Execution

```
Voice command: "Pick up the cube and place it on the shelf"
              ↓
voice_command → llm_planner_node → TaskPlan
              ↓              ↓
detections ────┘              ↓
              ↓
TaskPlan: [
  "approach object at [0.5, 0.3]",
  "open gripper",
  "close gripper",
  "lift to height 0.8m",
  "move to shelf location [0.7, 0.2]",
  "open gripper to release"
]
              ↓
orchestrator_node → ExecuteTask (action)
              ↓
motion_planning_node → planned trajectories
              ↓
arm_control_node → move robot via /robot/move_arm actions
              ↓
safety_validator_node → monitor for violations
              ↓
ExecutionStatus: success or failure + reason
```

**Latency Budget**:
- LLM planning: <3s
- Grasp generation: <1s
- Motion planning: <5s
- Execution: depends on task (typically 5-30s)
- Total: <40 seconds per task

### Scenario 3: Safety Monitoring

```
During execution:
              ↓
arm_control_node → RobotState updates (10Hz)
              ↓
safety_validator_node → checks against limits
              ↓
Violation detected → SafetyEvent("/robot/safety_events")
              ↓
If critical: → EmergencyStop("/robot/emergency_stop")
              ↓
orchestrator_node → halts execution, asks for replanning
```

## ROS 2 Quality of Service (QoS)

All nodes use consistent QoS settings for reliability:

| Node | Topic | Reliability | History | Durability |
|------|-------|-------------|---------|-----------|
| whisper_node | /robot/voice_command | RELIABLE | KEEP_LAST(10) | VOLATILE |
| yolo_detector | /robot/detections | BEST_EFFORT | KEEP_LAST(5) | VOLATILE |
| segmentation | /robot/segmentation | BEST_EFFORT | KEEP_LAST(1) | VOLATILE |
| llm_planner | /robot/task_plan | RELIABLE | KEEP_LAST(10) | VOLATILE |
| grasp_planner | /robot/grasps_offered | RELIABLE | KEEP_LAST(5) | VOLATILE |
| motion_planner | /robot/planned_path | RELIABLE | KEEP_LAST(1) | VOLATILE |
| orchestrator | /robot/execution_status | RELIABLE | KEEP_LAST(20) | VOLATILE |
| safety_validator | /robot/safety_events | RELIABLE | KEEP_LAST(100) | VOLATILE |

## Launch Files

Central launch file coordinates all nodes:

```bash
ros2 launch chapter4_integration vla_system.launch.py
```

Launch file starts:
1. Isaac Sim bridge (if not already running)
2. whisper_ros2_node
3. yolo_detector_node
4. segmentation_node
5. llm_planner_node
6. grasp_planner_node
7. motion_planning_node
8. arm_control_node
9. orchestrator_node
10. safety_validator_node
11. RViz for visualization

## Testing

Each node can be tested independently:

```bash
# Test voice input
ros2 run chapter4_voice whisper_ros2_node

# Test vision in separate terminal
ros2 run chapter4_vision yolo_detector_node
ros2 topic echo /robot/detections

# Test planning
ros2 call /robot/plan_task "..."

# Test end-to-end
ros2 run chapter4_integration orchestrator_node
```

## Error Handling

All nodes implement consistent error handling:

1. **Non-critical errors**: Log warning, publish to relevant topic, continue
2. **Critical errors**: Publish SafetyEvent, pause execution, await operator intervention
3. **Fatal errors**: Shutdown node gracefully, allow parent process to restart

Error codes defined in `chapter4_common/utils.py`.

## Performance Targets

| Component | Latency Target | Throughput | CPU Usage |
|-----------|----------------|-----------|-----------|
| Whisper transcription | <500ms | - | 10-20% |
| YOLO detection | <100ms | 10 FPS | 15-30% |
| Segmentation | <150ms | 6-7 FPS | 20-40% |
| LLM planning | <3s | 1 plan/3s | 5-10% |
| Motion planning | <5s | 1 plan/5s | 10-25% |
| Safety checking | <10ms | 100 Hz | <5% |
| Total system | <15s (voice→execution) | 1 task/15s | <100% combined |

## Next Steps

1. Implement message definitions and ROS 2 build system
2. Create launch files to coordinate nodes
3. Implement each node following this contract
4. Integration testing with Isaac Sim
5. Performance profiling and optimization
