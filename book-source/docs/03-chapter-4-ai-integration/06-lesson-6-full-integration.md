# Lesson 6: Full-Stack Integration & Orchestration

## Learning Objectives

By the end of this lesson, you will:
1. Understand system-level orchestration (coordinating all VLA components)
2. Design state machines for robust autonomous behavior
3. Implement error handling and recovery strategies
4. Monitor system health and detect failures
5. Validate end-to-end performance and latency
6. Deploy integrated system in Isaac Sim

## System Orchestration Overview

The orchestrator is the **brain** that coordinates all components:

```
┌─────────────────────────────────────────────────────┐
│           Orchestrator Node (Brain)                  │
├─────────────────────────────────────────────────────┤
│ • Listens to voice commands                         │
│ • Requests perception (vision)                       │
│ • Calls planning (LLM)                              │
│ • Manages grasp & motion planning                    │
│ • Executes arm motion                                │
│ • Monitors execution and handles failures            │
│ • Reports status to user                             │
└──────────────────────────────────────────────────────┘
         ↓         ↓         ↓         ↓        ↓
      Voice    Vision    Planning  Manipulation Safety
       Nodes    Nodes     Nodes     Nodes      Nodes
```

### State Machine Design

Robust orchestration requires a **state machine** that handles all scenarios:

```
                    ┌─────────────┐
                    │    START    │
                    └──────┬──────┘
                           │
                           ↓
                    ┌─────────────┐
         ┌─────────→│   IDLE      │←─────────┐
         │          └──────┬──────┘          │
         │ (done)           │ (voice heard)  │ (reset)
         │                  ↓                │
         │          ┌─────────────┐          │
         │ ┌───────→│ LISTENING   │          │
         │ │        └──────┬──────┘          │
         │ │ (timeout)     │ (command received)
         │ │               ↓                 │
         │ │        ┌─────────────┐          │
         │ │        │ PERCEIVING  │          │
         │ │        └──────┬──────┘          │
         │ │               │ (percepts collected)
         │ │               ↓                 │
         │ │        ┌─────────────┐          │
         │ │   ┌───→│  PLANNING   │          │
         │ │   │    └──────┬──────┘          │
         │ │   │           │ (plan generated)
         │ │   │replan     ↓                 │
         │ │   │    ┌─────────────┐          │
         │ │   └────│ EXECUTING   │          │
         │ │        └──────┬──────┘          │
         │ │               │ (success)  (failure)
         │ │               ↓                 │
         │ │        ┌─────────────┐          │
         └─┼────────│  OBSERVING  │          │
         │ │        └──────┬──────┘          │
         │ │               │                 │
         └─┴───────────────┴─────────────────┘
```

**States**:
- **IDLE**: Waiting for user command
- **LISTENING**: Capturing voice input
- **PERCEIVING**: Gathering vision data
- **PLANNING**: Calling LLM planner
- **EXECUTING**: Moving robot
- **OBSERVING**: Checking if execution succeeded

**Transitions**:
- Command recognized → PERCEIVING
- All percepts ready → PLANNING
- Plan generated → EXECUTING
- Execution complete → OBSERVING
- Plan failed → back to PLANNING (replan)
- User interrupt → back to IDLE

## Orchestrator Implementation

File: `chapter4_integration/src/orchestrator_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import json
import uuid
from enum import Enum
from datetime import datetime
from typing import Optional

class SystemState(Enum):
    IDLE = "IDLE"
    LISTENING = "LISTENING"
    PERCEIVING = "PERCEIVING"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    OBSERVING = "OBSERVING"
    ERROR = "ERROR"

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')

        # Subscribe to all sensor/component topics
        self.voice_sub = self.create_subscription(
            String, '/robot/voice_command', self.voice_callback, 10
        )
        self.detections_sub = self.create_subscription(
            String, '/robot/detections', self.detections_callback, 10
        )
        self.plan_sub = self.create_subscription(
            String, '/robot/task_plan', self.plan_callback, 10
        )
        self.execution_status_sub = self.create_subscription(
            String, '/robot/execution_status', self.execution_callback, 10
        )

        # Publish system status
        self.status_pub = self.create_publisher(
            String, '/robot/system_status', 10
        )

        # State tracking
        self.state = SystemState.IDLE
        self.current_task_id = None
        self.current_command = None
        self.current_detections = None
        self.current_plan = None
        self.execution_result = None
        self.retry_count = 0
        self.max_retries = 3

        # Timers for state transitions
        self.timer = self.create_timer(0.1, self.state_machine_tick)

        self.get_logger().info('Orchestrator Node initialized')

    def voice_callback(self, msg):
        """Handle incoming voice command"""
        self.current_command = msg.data
        self.current_task_id = str(uuid.uuid4())
        self.state = SystemState.PERCEIVING
        self.get_logger().info(f'Voice command received: "{self.current_command}"')

    def detections_callback(self, msg):
        """Store latest perception data"""
        try:
            self.current_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse detections')

    def plan_callback(self, msg):
        """Handle incoming task plan"""
        try:
            self.current_plan = json.loads(msg.data)
            self.state = SystemState.EXECUTING
            self.get_logger().info('Task plan received, transitioning to EXECUTING')
        except json.JSONDecodeError:
            self.state = SystemState.ERROR

    def execution_callback(self, msg):
        """Monitor execution status"""
        try:
            self.execution_result = json.loads(msg.data)
            if self.execution_result.get('is_complete', False):
                self.state = SystemState.OBSERVING
        except json.JSONDecodeError:
            pass

    def state_machine_tick(self):
        """Main state machine update"""
        if self.state == SystemState.IDLE:
            self._state_idle()
        elif self.state == SystemState.LISTENING:
            self._state_listening()
        elif self.state == SystemState.PERCEIVING:
            self._state_perceiving()
        elif self.state == SystemState.PLANNING:
            self._state_planning()
        elif self.state == SystemState.EXECUTING:
            self._state_executing()
        elif self.state == SystemState.OBSERVING:
            self._state_observing()
        elif self.state == SystemState.ERROR:
            self._state_error()

        self._publish_status()

    def _state_idle(self):
        """IDLE state: wait for command"""
        pass  # Wait for voice_callback to trigger

    def _state_listening(self):
        """LISTENING state: capturing voice"""
        pass  # Voice node handles this

    def _state_perceiving(self):
        """PERCEIVING state: gather perception data"""
        # We have command, now wait for detections
        if self.current_detections:
            # Move to planning once we have percepts
            self.state = SystemState.PLANNING
            self.get_logger().info('Perception data collected, transitioning to PLANNING')

    def _state_planning(self):
        """PLANNING state: request LLM plan"""
        if not self.current_plan:
            # Publish planning request (LLM planner listens)
            request = {
                'task_id': self.current_task_id,
                'command': self.current_command,
                'detections': self.current_detections
            }
            # In real implementation, would call planning service
            self.get_logger().info('Requesting plan from LLM planner...')

    def _state_executing(self):
        """EXECUTING state: execute task"""
        if self.execution_result and self.execution_result.get('is_complete'):
            if self.execution_result.get('success'):
                self.state = SystemState.OBSERVING
                self.get_logger().info('Execution succeeded')
            else:
                # Execution failed, maybe retry
                if self.retry_count < self.max_retries:
                    self.retry_count += 1
                    self.state = SystemState.PLANNING  # Replan
                    self.get_logger().info(f'Execution failed, retrying ({self.retry_count}/{self.max_retries})')
                else:
                    self.state = SystemState.ERROR
                    self.get_logger().error('Execution failed after max retries')

    def _state_observing(self):
        """OBSERVING state: validate task completion"""
        # Check if execution succeeded
        if self.execution_result and self.execution_result.get('success'):
            self.get_logger().info('Task completed successfully!')
            self.state = SystemState.IDLE
            self.retry_count = 0
        else:
            self.state = SystemState.ERROR

    def _state_error(self):
        """ERROR state: handle failures"""
        self.get_logger().error(f'System error: {self.execution_result}')
        # Reset after error
        if self.retry_count >= self.max_retries:
            self.state = SystemState.IDLE
            self.retry_count = 0

    def _publish_status(self):
        """Publish current system status"""
        status = {
            'timestamp': datetime.now().isoformat(),
            'state': self.state.value,
            'current_task_id': self.current_task_id,
            'current_command': self.current_command,
            'retry_count': self.retry_count
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    orchestrator = OrchestratorNode()
    rclpy.spin(orchestrator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Task Status Monitor

File: `chapter4_integration/src/task_status_monitor.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class TaskStatusMonitor(Node):
    """Monitor and log task execution progress"""

    def __init__(self):
        super().__init__('task_status_monitor')

        # Subscribe to all status topics
        self.voice_sub = self.create_subscription(
            String, '/robot/voice_command', self.log_voice, 10
        )
        self.detection_sub = self.create_subscription(
            String, '/robot/detections', self.log_detection, 10
        )
        self.plan_sub = self.create_subscription(
            String, '/robot/task_plan', self.log_plan, 10
        )
        self.execution_sub = self.create_subscription(
            String, '/robot/execution_status', self.log_execution, 10
        )
        self.system_status_sub = self.create_subscription(
            String, '/robot/system_status', self.log_system_status, 10
        )

        self.execution_trace = []

        self.get_logger().info('Task Status Monitor initialized')

    def log_voice(self, msg):
        """Log voice command"""
        self.execution_trace.append({
            'component': 'voice',
            'timestamp': datetime.now().isoformat(),
            'data': msg.data
        })
        self.get_logger().info(f'[VOICE] {msg.data}')

    def log_detection(self, msg):
        """Log detected objects"""
        try:
            data = json.loads(msg.data)
            num_detections = len(data.get('detections', []))
            self.execution_trace.append({
                'component': 'vision',
                'timestamp': datetime.now().isoformat(),
                'num_detections': num_detections
            })
            self.get_logger().info(f'[VISION] Detected {num_detections} objects')
        except json.JSONDecodeError:
            pass

    def log_plan(self, msg):
        """Log task plan"""
        try:
            data = json.loads(msg.data)
            subtasks = data.get('subtasks', [])
            self.execution_trace.append({
                'component': 'planning',
                'timestamp': datetime.now().isoformat(),
                'num_subtasks': len(subtasks)
            })
            self.get_logger().info(f'[PLANNING] Generated {len(subtasks)} subtasks')
        except json.JSONDecodeError:
            pass

    def log_execution(self, msg):
        """Log execution progress"""
        try:
            data = json.loads(msg.data)
            progress = data.get('progress_percent', 0)
            self.execution_trace.append({
                'component': 'execution',
                'timestamp': datetime.now().isoformat(),
                'progress': progress
            })
            self.get_logger().info(f'[EXECUTION] Progress: {progress:.1f}%')
        except json.JSONDecodeError:
            pass

    def log_system_status(self, msg):
        """Log system status"""
        try:
            data = json.loads(msg.data)
            state = data.get('state', 'UNKNOWN')
            self.get_logger().info(f'[SYSTEM] State: {state}')
        except json.JSONDecodeError:
            pass

    def get_execution_trace(self):
        """Retrieve execution trace for debugging"""
        return self.execution_trace


def main(args=None):
    rclpy.init(args=args)
    monitor = TaskStatusMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch File for Full Pipeline

File: `chapter4_integration/launch/vla_pipeline.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch all VLA components in correct order"""

    # 1. Perception nodes (vision)
    yolo_node = Node(
        package='chapter4_vision',
        executable='yolo_detector_node',
        name='yolo_detector',
        parameters=[{
            'model_name': 'yolov8n.pt',
            'confidence_threshold': 0.5
        }]
    )

    segmentation_node = Node(
        package='chapter4_vision',
        executable='segmentation_node',
        name='segmentation'
    )

    # 2. Voice node
    voice_node = Node(
        package='chapter4_voice',
        executable='whisper_ros2_node',
        name='whisper'
    )

    # 3. Planning node (LLM)
    planning_node = Node(
        package='chapter4_planning',
        executable='llm_planner_node',
        name='llm_planner',
        parameters=[{
            'use_openai': True,
            'model': 'gpt-4'
        }]
    )

    # 4. Manipulation nodes (grasp + motion)
    grasp_planner = Node(
        package='chapter4_manipulation',
        executable='grasp_planner_node',
        name='grasp_planner'
    )

    motion_planner = Node(
        package='chapter4_manipulation',
        executable='motion_planner_node',
        name='motion_planner'
    )

    # 5. Orchestrator (coordinates everything)
    orchestrator = Node(
        package='chapter4_integration',
        executable='orchestrator_node',
        name='orchestrator'
    )

    # 6. Monitoring
    status_monitor = Node(
        package='chapter4_integration',
        executable='task_status_monitor',
        name='status_monitor'
    )

    # 7. Safety (runs independently)
    safety_validator = Node(
        package='chapter4_safety',
        executable='safety_validator_node',
        name='safety_validator'
    )

    return LaunchDescription([
        yolo_node,
        segmentation_node,
        voice_node,
        planning_node,
        grasp_planner,
        motion_planner,
        orchestrator,
        status_monitor,
        safety_validator
    ])
```

## End-to-End Test Scenario

```python
def test_end_to_end_manipulation():
    """Test complete pipeline: voice → perception → planning → manipulation"""

    # Launch Isaac Sim with scene
    sim = IsaacSimLauncher()
    sim.load_scene('scene_2_manipulation_task.yaml')

    # Place objects
    sim.spawn_object('red_cube', [0.5, 0.3, 0.1])
    sim.spawn_object('shelf', [0.7, 0.2, 1.2])

    # Launch ROS 2 nodes
    ros_nodes = launch_vla_pipeline()

    # Simulate voice input
    voice_publisher = create_publisher('/robot/voice_command', String)
    voice_publisher.publish('Pick up the red cube and place it on the shelf')

    # Monitor execution
    execution_trace = []
    start_time = time.time()

    while time.time() - start_time < 30:
        # Collect status updates
        status = get_latest_status()
        execution_trace.append(status)

        if status['state'] == 'IDLE':
            # Task completed
            break

    elapsed_time = time.time() - start_time

    # Validate
    assert elapsed_time < 15, f'Task took {elapsed_time}s, should be <15s'
    assert sim.get_object_position('red_cube')[2] > 1.0, 'Cube not on shelf'

    print(f"✓ End-to-end test PASSED ({elapsed_time:.2f}s)")

    return execution_trace
```

## Exercises

### Exercise L6-1: State Machine Design

**Objective**: Design robust state machine for new task

**Steps**:
1. Define task: "Sort colored blocks by color"
2. Draw state diagram (at least 6 states)
3. Label all transitions and conditions
4. Identify failure cases (what if color detection fails?)
5. Define recovery strategies for each failure

**Success Criteria**:
- ✓ State diagram covers happy path
- ✓ All failure modes have recovery
- ✓ No deadlock states
- ✓ Clear transition conditions

### Exercise L6-2: End-to-End Integration

**Objective**: Run complete VLA pipeline

**Steps**:
1. Launch Isaac Sim with humanoid robot
2. Launch all nodes: `ros2 launch chapter4_integration vla_pipeline.launch.py`
3. Give voice command: "Find the red cube"
4. Monitor `/robot/system_status` in another terminal
5. Check execution trace: `rostopic echo /robot/execution_trace`

**Success Criteria**:
- ✓ All nodes launch without errors
- ✓ State machine transitions work
- ✓ Voice command processed
- ✓ Objects detected
- ✓ Plan generated
- ✓ Task completes or fails gracefully

## Real Hardware Considerations

### Sim-to-Real Transfer Challenges

**In Simulation**:
- Perception perfect (camera calibration exact)
- Physics deterministic (same grasp → always succeeds)
- No latency (communication instantaneous)
- No sensor failures

**On Real Hardware**:
- Camera calibration 5cm+ error
- Object slippage, deformation, unexpected properties
- Network latency 50-500ms
- Sensors fail intermittently
- Humans watching—need to be predictable

### Domain Randomization

To prepare for real world, train on randomized simulations:

```python
def randomize_simulation():
    """Add noise to simulation for robustness"""
    # Random lighting
    brightness = random.uniform(0.5, 2.0)

    # Random object textures
    texture = random.choice(['matte', 'glossy', 'reflective'])

    # Random physics
    friction = random.uniform(0.2, 1.0)
    density = random.uniform(0.5, 1.5)

    # Random camera noise
    gaussian_noise = random.normal(0, 0.02)

    return {
        'lighting': brightness,
        'texture': texture,
        'friction': friction,
        'density': density,
        'camera_noise': gaussian_noise
    }
```

## Key Takeaways

1. **Orchestration** coordinates all components into unified system
2. **State machines** provide robust framework for complex behavior
3. **Error handling** and **replanning** are non-optional
4. **Monitoring** and **status reporting** help debug failures
5. **Testing** must cover happy path AND failure scenarios
6. **Sim-to-real** requires domain randomization and robustness strategies

## Next Steps

In **Lesson 7**, you'll implement **safety protocols**:
- Kill switch (emergency stop)
- Workspace boundaries
- Force limits
- Safe recovery states

These are **non-negotiable** for real robot deployment!

See you in Lesson 7! 🛡️
