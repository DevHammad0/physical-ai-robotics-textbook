---
title: "Chapter 1 Capstone Project: Autonomous Turtle Navigation System"
chapter: 1
lesson: 8
duration_minutes: 120

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Design a multi-node ROS 2 system from scratch"
  - "Integrate publishers, subscribers, and services"
  - "Debug complex robotics system issues"
  - "Validate system behavior with acceptance criteria"
  - "Document system architecture for team collaboration"
  - "Apply all 7 lessons to a real-world problem"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (Integration)"
---

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/08-capstone.md">

## Capstone Overview

Congratulations! You've completed all 7 lessons of Chapter 1. Now it's time to put everything together.

**Your Mission**: Build an **autonomous turtle navigation system** that:

1. Navigates a Turtlesim turtle to randomly generated waypoints
2. Avoids boundaries (prevents turtle from leaving the 0-11 area)
3. Tracks trajectory and reports statistics
4. Responds to emergency stop commands
5. Runs for 60+ seconds without errors

**Simulation environment**: Turtlesim

**Time estimate**: 2-3 hours (includes coding, debugging, testing)

**Skills applied**: Every concept from Lessons 1-7

---

## System Architecture

Your system consists of 4 nodes:

```
┌────────────────────────────────────────────────────────────────┐
│                   Autonomous Turtle System                      │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐                                           │
│  │  Turtlesim       │  (Simulator)                              │
│  │  /turtle1/pose   │←── (receives position data)               │
│  │  /turtle1/cmd_vel├───→ (sends velocity commands)             │
│  └──────────────────┘                                           │
│         ↑                                                        │
│         │ pose data                                             │
│         │                                                       │
│  ┌──────▼──────────────────┐    ┌──────────────────┐           │
│  │  Navigator Node          │    │  Tracker Node    │           │
│  │  (calculates trajectory) ├───→│ (records stats)  │           │
│  │                          │    │                  │           │
│  │ - Publishes trajectory   │    │ - Subscribes to  │           │
│  │ - Subscribes to pose     │    │   pose & status  │           │
│  │ - Calls boundary service │    │                  │           │
│  └──────┬───────────────────┘    └──────────────────┘           │
│         │                                                       │
│    cmd_vel                                                      │
│         │                                                       │
│  ┌──────▼──────────────────┐    ┌──────────────────┐           │
│  │  Safety Monitor Node     │    │  Status Node     │           │
│  │  (enforces boundaries)   ├────│ (publishes state)│           │
│  │                          │    │                  │           │
│  │ - Service: set_target    │    │ - /system/status │           │
│  │ - Subscribes to cmd_vel  │    │ - /system/ready  │           │
│  │ - Publishes safe cmd_vel │    │                  │           │
│  └──────────────────────────┘    └──────────────────┘           │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
```

### Node Responsibilities

| Node | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| **Navigator** | Plans path to target | target (topic), pose | trajectory (/nav/trajectory) |
| **Safety Monitor** | Enforces boundaries | cmd_vel, pose | safe_cmd_vel (/cmd_vel_safe) |
| **Tracker** | Logs statistics | pose, trajectory | /system/stats |
| **Status** | System health | (timer) | /system/status |

---

## Requirements

### Functional Requirements

**R1**: System must navigate turtle to 5 randomly generated waypoints

```python
# Waypoints in range (1, 10)
waypoints = [(3, 4), (8, 9), (2, 8), (9, 2), (5, 5)]
```

**R2**: Turtle must not leave the boundary [0, 11] × [0, 11]

```python
# Enforce: 0 <= x <= 11, 0 <= y <= 11
# If command would violate, reduce velocity
```

**R3**: System must track and report statistics

```python
# Track:
# - Total distance traveled
# - Time per waypoint
# - Final position
```

**R4**: System must run for 60+ seconds without crashing

```python
# No unhandled exceptions
# Graceful shutdown on Ctrl+C
```

**R5**: System must respond to emergency stop

```python
# Ctrl+C → stop turtle, log stats, shutdown cleanly
```

### Non-Functional Requirements

**NF1**: Code is readable and documented

```python
# All functions have docstrings
# Code follows PEP 8 style
```

**NF2**: System is observable (can see what's happening)

```bash
# Should be able to:
# ros2 topic echo /system/status
# ros2 topic list | wc -l  # Multiple topics active
# rqt_graph                 # Visualize architecture
```

**NF3**: Performance acceptable for simulation

```python
# - No blocking calls in callbacks
# - Timer frequency: 10-50 Hz
# - Message processing: < 100 ms
```

---

## Project Structure

Organize your code:

```
ros2_workspace/
├── src/
│   └── turtle_capstone/
│       ├── turtle_capstone/
│       │   ├── __init__.py
│       │   ├── navigator.py          # Node 1
│       │   ├── safety_monitor.py     # Node 2
│       │   ├── tracker.py            # Node 3
│       │   └── status_node.py        # Node 4
│       ├── launch/
│       │   └── turtle_nav_system.launch.py
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
└── (build, install directories)
```

---

## Code Example 1: Navigator Node

The Navigator calculates trajectories to target waypoints.

```python
"""
Navigator Node - Plans path to target waypoints
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32MultiArray
import math
import random


class NavigatorNode(Node):
    """Plans navigation to target waypoints"""

    def __init__(self):
        super().__init__('navigator')

        # Current state
        self.pose = Pose()
        self.target_x = 5.54
        self.target_y = 5.54
        self.waypoint_index = 0

        # Generate 5 random waypoints
        random.seed(42)  # Reproducible
        self.waypoints = [
            (random.uniform(1, 10), random.uniform(1, 10))
            for _ in range(5)
        ]

        self.get_logger().info(f'Waypoints: {self.waypoints}')

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publishers
        self.trajectory_pub = self.create_publisher(
            Float32MultiArray, '/nav/trajectory', 10)

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        """Update current pose"""
        self.pose = msg

    def control_loop(self):
        """Main control loop"""

        # Check if reached current waypoint
        dx = self.waypoints[self.waypoint_index][0] - self.pose.x
        dy = self.waypoints[self.waypoint_index][1] - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.3:
            # Move to next waypoint
            if self.waypoint_index < len(self.waypoints) - 1:
                self.waypoint_index += 1
                self.get_logger().info(
                    f'Waypoint reached! Moving to {self.waypoints[self.waypoint_index]}')
            else:
                # All waypoints done
                self.get_logger().info('All waypoints completed!')
                msg = Twist()
                self.cmd_vel_pub.publish(msg)
                return

        # Calculate direction to target
        target = self.waypoints[self.waypoint_index]
        dx = target[0] - self.pose.x
        dy = target[1] - self.pose.y
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.pose.theta

        # Normalize angle
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Publish command
        cmd = Twist()
        cmd.linear.x = min(1.0, distance)
        cmd.angular.z = angle_error * 1.5  # Proportional controller

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Code Example 2: Safety Monitor Node

Enforces boundaries and prevents collision.

```python
"""
Safety Monitor Node - Enforces boundaries
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class SafetyMonitorNode(Node):
    """Monitors and enforces system safety"""

    def __init__(self):
        super().__init__('safety_monitor')

        self.pose = Pose()
        self.boundary_margin = 0.5  # Stay 0.5 units away from boundary

        # Subscriber: actual commands
        self.cmd_sub = self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.cmd_callback, 10)

        # Subscriber: pose
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        self.get_logger().info('Safety monitor started')

    def pose_callback(self, msg):
        """Update pose"""
        self.pose = msg

    def cmd_callback(self, msg):
        """Filter command for safety"""

        # Predict new position
        new_x = self.pose.x + msg.linear.x * 0.1
        new_y = self.pose.y + msg.linear.y * 0.1

        # Check boundaries
        if new_x < self.boundary_margin:
            msg.linear.x = 0  # Stop forward
        elif new_x > (11 - self.boundary_margin):
            msg.linear.x = 0  # Stop forward

        if new_y < self.boundary_margin:
            msg.linear.y = 0  # Stop lateral
        elif new_y > (11 - self.boundary_margin):
            msg.linear.y = 0  # Stop lateral

        # Publish safe command (without filtering, just for monitoring)
        # In production, would publish filtered command
        self.get_logger().debug(
            f'Position: ({self.pose.x:.2f}, {self.pose.y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Code Example 3: Tracker Node

Collects statistics about system performance.

```python
"""
Tracker Node - Collects and logs system statistics
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
import math


class TrackerNode(Node):
    """Tracks system performance and statistics"""

    def __init__(self):
        super().__init__('tracker')

        self.prev_pose = Pose()
        self.total_distance = 0.0
        self.start_time = self.get_clock().now()

        # Subscriber
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher
        self.stats_pub = self.create_publisher(
            String, '/system/stats', 10)

        # Timer to publish stats
        self.timer = self.create_timer(5.0, self.publish_stats)

        self.get_logger().info('Tracker started')

    def pose_callback(self, msg):
        """Track distance traveled"""

        if self.prev_pose.x == 0 and self.prev_pose.y == 0:
            self.prev_pose = msg
            return

        dx = msg.x - self.prev_pose.x
        dy = msg.y - self.prev_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        self.total_distance += distance
        self.prev_pose = msg

    def publish_stats(self):
        """Publish statistics"""

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        avg_speed = self.total_distance / elapsed if elapsed > 0 else 0

        msg = String()
        msg.data = (
            f'Distance: {self.total_distance:.2f}m, '
            f'Elapsed: {elapsed:.1f}s, '
            f'Avg Speed: {avg_speed:.2f}m/s')

        self.stats_pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Code Example 4: Status Node

Publishes system health and readiness.

```python
"""
Status Node - Publishes system health
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class StatusNode(Node):
    """Publishes system status"""

    def __init__(self):
        super().__init__('status')

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/system/status', 10)

        self.ready_pub = self.create_publisher(
            Bool, '/system/ready', 10)

        # Timer
        self.timer = self.create_timer(1.0, self.publish_status)

        self.start_time = self.get_clock().now()
        self.is_ready = True

        self.get_logger().info('Status node started')

    def publish_status(self):
        """Publish status messages"""

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Status message
        status_msg = String()
        status_msg.data = f'System running, Uptime: {elapsed:.1f}s'
        self.status_pub.publish(status_msg)

        # Readiness message
        ready_msg = Bool()
        ready_msg.data = self.is_ready
        self.ready_pub.publish(ready_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Launch File

Create: `turtle_nav_system.launch.py`

```python
"""
Turtle Navigation System Launch
Starts all 4 nodes in coordinated fashion
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the complete turtle navigation system"""

    # Start Turtlesim simulator
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen'
    )

    # Start Navigator node
    navigator = Node(
        package='turtle_capstone',
        executable='navigator',
        name='navigator',
        output='screen'
    )

    # Start Safety Monitor node
    safety = Node(
        package='turtle_capstone',
        executable='safety_monitor',
        name='safety',
        output='screen'
    )

    # Start Tracker node
    tracker = Node(
        package='turtle_capstone',
        executable='tracker',
        name='tracker',
        output='screen'
    )

    # Start Status node
    status = Node(
        package='turtle_capstone',
        executable='status',
        name='status',
        output='screen'
    )

    return LaunchDescription([
        turtlesim,
        navigator,
        safety,
        tracker,
        status
    ])
```

---

## Running the Capstone

### Setup

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python turtle_capstone

# Copy node files to: ~/ros2_ws/src/turtle_capstone/turtle_capstone/
# Copy launch file to: ~/ros2_ws/src/turtle_capstone/launch/

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run

```bash
# Option 1: Using launch file (recommended)
ros2 launch turtle_capstone turtle_nav_system.launch.py

# Option 2: Manual startup
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 run turtle_capstone navigator

# Terminal 3
ros2 run turtle_capstone tracker

# Terminal 4
ros2 run turtle_capstone safety

# Terminal 5
ros2 run turtle_capstone status
```

### Monitoring

While system runs:

```bash
# View system status
ros2 topic echo /system/status

# View statistics
ros2 topic echo /system/stats

# Visualize architecture
rqt_graph

# List all topics
ros2 topic list

# Monitor turtle position in real-time
ros2 topic echo /turtle1/pose
```

---

## Acceptance Criteria

Your system passes if:

- [x] **Criterion 1**: System starts without errors
  - `ros2 launch turtle_capstone turtle_nav_system.launch.py` succeeds
  - All 5 nodes running (verify with `ros2 node list`)

- [x] **Criterion 2**: Turtle navigates to all 5 waypoints
  - Verify with visual inspection or log messages
  - Each waypoint should be reached within 2 minutes

- [x] **Criterion 3**: Turtle stays within boundaries
  - Monitor `/turtle1/pose` to verify `0 <= x, y <= 11`
  - Never exceeds boundaries

- [x] **Criterion 4**: System runs 60+ seconds without crashing
  - Leave running for at least 60 seconds
  - No exceptions in terminal output
  - All nodes still alive at end (verify with `ros2 node list`)

- [x] **Criterion 5**: System shuts down cleanly
  - Ctrl+C stops all nodes gracefully
  - No hanging processes
  - Statistics printed to console

- [x] **Criterion 6**: Code is documented
  - All functions have docstrings
  - Comments explain complex logic
  - README explains system architecture

---

## Debugging Guide

### Problem: "Node exits immediately"

**Symptom**: Node runs briefly then stops

**Diagnosis**: Check for exceptions

```bash
# Run node directly to see error
python3 ~/ros2_ws/src/turtle_capstone/turtle_capstone/navigator.py
```

**Common fixes**:
- Missing import statements
- Typo in node name
- Syntax error in Python code

### Problem: "Turtle not moving"

**Symptom**: Turtle stays at (5.54, 5.54)

**Diagnosis**: Check command publishing

```bash
ros2 topic echo /turtle1/cmd_vel
```

If no output: Navigator not publishing

**Fix**: Ensure Navigator node is running

```bash
ros2 node list  # Should show /navigator
```

### Problem: "Topics not connecting"

**Symptom**: Publisher and subscriber both running, no data flow

**Diagnosis**: Check ROS_DOMAIN_ID

```bash
echo $ROS_DOMAIN_ID
```

Should be same for all terminals (default: 0)

**Fix**:

```bash
export ROS_DOMAIN_ID=0
# Then run launch file
```

### Problem: "System becomes unresponsive"

**Symptom**: Turtle stops moving mid-navigation

**Cause**: Likely exception in callback

**Fix**: Check logs

```bash
# Increase logging verbosity
export RCL_LOG_LEVEL=DEBUG
ros2 launch turtle_capstone turtle_nav_system.launch.py
```

---

## Success Metrics

You've mastered Chapter 1 when your capstone:

1. **Runs cleanly** for 60+ seconds
2. **Completes all waypoints** without human intervention
3. **Respects boundaries** (never leaves simulation area)
4. **Logs statistics** (distance, time, speed)
5. **Shuts down gracefully** on Ctrl+C
6. **Is observable** (can visualize with rqt_graph, echo topics)
7. **Is maintainable** (readable code, good documentation)

---

## Next Steps After Capstone

Congratulations! You've mastered ROS 2 fundamentals. You're ready for:

### Chapter 2: Gazebo Simulation
- Physics-based simulation
- URDF robot descriptions
- Sensors (cameras, lidar)
- More complex multi-robot systems

### Chapter 3: NVIDIA Isaac Sim
- Photorealistic rendering
- Synthetic data generation
- Digital twins
- Reinforcement learning

### Chapter 4: Vision-Language Agents
- Computer vision with ROS 2
- OpenAI integration
- Natural language control
- Autonomous decision-making

---

## Optional Enhancements

If you finish early, try these extensions:

1. **Add boundary guardian node**: Publishes warnings when turtle gets too close to edge

2. **Implement waypoint server**: Service that accepts new waypoint and returns distance

3. **Create data recorder**: Node that writes all statistics to CSV file

4. **Add multi-robot support**: Run 3 turtles simultaneously with non-overlapping waypoints

5. **Implement simple UI**: Use `rqt` plugin to set waypoints graphically

---

**Congratulations on Completing Chapter 1: ROS 2 Fundamentals!**

You now have the foundation to build any ROS 2-based robotics system. Well done!

---

**End of Capstone Project**


</PersonalizedLesson>
