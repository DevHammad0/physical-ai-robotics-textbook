---
title: "Building Multi-Node Systems & Launch Files"
chapter: 1
lesson: 7
duration_minutes: 75

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Understand ROS 2 launch files and their purpose"
  - "Write launch files that start multiple nodes simultaneously"
  - "Pass parameters to nodes via launch files"
  - "Debug multi-node systems using ROS 2 tools"
  - "Design coordinated multi-node robotics architectures"
  - "Understand node lifecycle and startup ordering"
  - "Use rqt_graph to visualize system architecture"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (AI Collaboration)"
---

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/07-launch-files.md">

## Introduction

So far, you've written individual nodes (publishers, subscribers). Real robotics systems consist of **many coordinated nodes** working together:

- Camera driver (publishes images)
- Motion planner (subscribes to images, publishes trajectories)
- Motor controller (subscribes to trajectories, publishes motor commands)
- State manager (publishes system status)
- Logging node (subscribes to everything, records data)

Starting these manually in separate terminals is tedious and error-prone. **Launch files** solve this: a single file that starts all nodes with proper configuration.

**What you'll build**:
- A launch file that starts Turtlesim + multiple nodes
- A 3-node system: velocity publisher, position tracker, supervisor
- Integration testing of multi-node systems

**Simulation environment**: Turtlesim (from Lesson 2)

**Time estimate**: 75 minutes

By the end, you'll understand:
- ROS 2 launch file syntax (.launch.py)
- Parameter passing and configuration
- Multi-node debugging and visualization
- Production-ready system orchestration

---

## Concept 1: Why Launch Files?

### Without Launch Files

Start 5 nodes manually:

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
python3 publisher_node.py

# Terminal 3
python3 subscriber_node.py

# Terminal 4
python3 state_manager.py

# Terminal 5
python3 logger.py
```

**Problems**:
- Need 5 terminals open
- Easy to forget one
- Hard to restart (kill each manually)
- No consistent configuration
- Difficult for team collaboration

### With Launch Files

One command:

```bash
ros2 launch my_package turtle_system.launch.py
```

Starts all 5 nodes with proper configuration, dependencies, logging.

---

## Concept 2: Launch File Syntax (.launch.py)

ROS 2 launch files are Python scripts that create a `LaunchDescription`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate ROS 2 launch configuration"""

    # List of nodes to start
    nodes = [
        # Node 1: Turtlesim simulator
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Node 2: Publisher
        Node(
            package='my_package',
            executable='publisher_node',
            name='publisher',
            parameters=[
                {'speed': 1.0},
                {'frequency': 10}
            ]
        ),

        # Node 3: Subscriber
        Node(
            package='my_package',
            executable='subscriber_node',
            name='subscriber'
        )
    ]

    return LaunchDescription(nodes)
```

### Key Components

1. **Node**: Represents a ROS 2 node to start
2. **package**: ROS 2 package name (must be installed)
3. **executable**: Node script to run
4. **name**: Node instance name (can run same executable multiple times)
5. **parameters**: Configuration passed to node

---

## Concept 3: Parameter Passing

Nodes accept parameters from launch files:

### In Node Code

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare parameter with default value
        self.declare_parameter('speed', 1.0)

        # Get parameter
        speed = self.get_parameter('speed').value

        self.get_logger().info(f'Speed parameter: {speed}')
```

### In Launch File

```python
Node(
    package='my_package',
    executable='my_node',
    name='my_node',
    parameters=[
        {'speed': 2.5},
        {'frequency': 20}
    ]
)
```

### Command-Line Override

```bash
# Override launch file parameter
ros2 launch my_package system.launch.py speed:=3.0
```

---

## Code Example 1: Simple Launch File

Create: `simple_turtle_launch.py`

```python
"""
Simple Turtlesim Launch File
Starts: Turtlesim + keyboard teleoperation node
Simulation environment: Turtlesim
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create launch configuration"""

    # Start Turtlesim simulator
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen'  # Print logs to console
    )

    # Start keyboard teleoperation
    teleop_node = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='teleop',
        output='screen'
    )

    # Combine nodes into launch description
    return LaunchDescription([
        turtlesim_node,
        teleop_node
    ])
```

### How to Use

```bash
# Place file in: ~/ros2_ws/src/my_package/launch/
# Or run directly:
ros2 launch simple_turtle_launch.py
```

### Expected Result

- Turtlesim window opens
- Keyboard control active (press 'w', 'a', 's', 'd')

---

## Code Example 2: Launch File with Parameters

Create: `configured_turtle_launch.py`

```python
"""
Configured Turtle Launch with Parameters
Starts: Turtlesim + velocity publisher with configurable parameters
Simulation environment: Turtlesim
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create launch configuration with parameters"""

    # Declare launch arguments (parameters user can override)
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='1.0',
        description='Forward velocity of turtle (m/s)'
    )

    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='10',
        description='Publishing frequency (Hz)'
    )

    # Start Turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen'
    )

    # Start velocity publisher with parameters from launch arguments
    publisher_node = Node(
        package='my_first_package',
        executable='publisher_node',
        name='publisher',
        output='screen',
        parameters=[
            {'speed': LaunchConfiguration('speed')},
            {'frequency': LaunchConfiguration('frequency')}
        ]
    )

    return LaunchDescription([
        speed_arg,
        frequency_arg,
        turtlesim_node,
        publisher_node
    ])
```

### How to Use

```bash
# Use default parameters
ros2 launch configured_turtle_launch.py

# Override parameters
ros2 launch configured_turtle_launch.py speed:=2.0 frequency:=20

# See available parameters
ros2 launch configured_turtle_launch.py --show-args
```

---

## Code Example 3: Multi-Node Turtle Control System

Let's create a complete 3-node system:

1. **Supervisor Node**: Manages system state
2. **Publisher Node**: Sends movement commands
3. **Subscriber Node**: Tracks position

Create: `turtle_control_system.launch.py`

```python
"""
Complete Turtle Control System Launch
Manages: Turtlesim + Supervisor + Publisher + Subscriber
Simulation environment: Turtlesim
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Multi-node turtle control system"""

    # Arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='auto',
        description='Mode: auto (autonomous) or manual (keyboard)'
    )

    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='1.0',
        description='Turtle velocity (m/s)'
    )

    # Turtlesim simulator
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen'
    )

    # Supervisor node (manages state)
    supervisor = Node(
        package='my_first_package',
        executable='supervisor_node',
        name='supervisor',
        output='screen',
        parameters=[
            {'mode': LaunchConfiguration('mode')}
        ]
    )

    # Publisher node (sends velocity commands)
    publisher = Node(
        package='my_first_package',
        executable='publisher_node',
        name='publisher',
        output='screen',
        parameters=[
            {'speed': LaunchConfiguration('speed')},
            {'frequency': 10}
        ]
    )

    # Subscriber node (tracks position)
    subscriber = Node(
        package='my_first_package',
        executable='subscriber_node',
        name='tracker',
        output='screen'
    )

    # Print startup message
    log_message = LogInfo(msg='Turtle control system started!')

    return LaunchDescription([
        mode_arg,
        speed_arg,
        log_message,
        turtlesim,
        supervisor,
        publisher,
        subscriber
    ])
```

---

## Code Example 4: Supervisor Node

Create: `supervisor_node.py`

```python
"""
Supervisor Node - Manages system state
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SupervisorNode(Node):
    """Manages overall system state and coordination"""

    def __init__(self):
        super().__init__('supervisor')

        # Declare mode parameter
        self.declare_parameter('mode', 'auto')
        mode = self.get_parameter('mode').value

        # Status publisher
        self.status_pub = self.create_publisher(
            String, '/system/status', 10)

        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)

        # State tracking
        self.mode = mode
        self.start_time = self.get_clock().now()
        self.message_count = 0

        self.get_logger().info(f'Supervisor started in {mode} mode')

    def publish_status(self):
        """Publish system status"""

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        msg = String()
        msg.data = f'Mode: {self.mode}, Uptime: {elapsed:.1f}s'

        self.status_pub.publish(msg)
        self.message_count += 1

        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()

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

## Debugging Multi-Node Systems

### Visualization: rqt_graph

Visualize the system architecture:

```bash
rqt_graph
```

This shows:
- All running nodes
- All topics and connections
- Message flow between nodes

### Viewing All Topics

```bash
ros2 topic list
ros2 topic list -t  # Show message types
```

### Monitoring Topic Traffic

```bash
# See messages on a topic
ros2 topic echo /turtle1/pose

# See message rate
ros2 topic hz /turtle1/pose

# Get bandwidth
ros2 topic bw /turtle1/pose
```

### Logging and Filtering

```bash
# Set log level to DEBUG
export RCL_LOG_LEVEL=DEBUG
ros2 launch turtle_control_system.launch.py

# View only errors
ros2 topic echo /rosout --filter "severity.label=='ERROR'"
```

---

## Troubleshooting Multi-Node Systems

### Error: "Package not found"

**Cause**: Node package not installed or not in ROS_PACKAGE_PATH

**Solution**:

```bash
# Install package
sudo apt install ros-humble-<package_name>

# Or source your workspace
source ~/ros2_ws/install/setup.bash
```

### Error: "Executable not found"

**Cause**: Wrong executable name in launch file

**Solution**:

```bash
# Check available executables in package
ros2 pkg executables turtlesim

# Use correct name in launch file
```

### Error: "Node crashes on startup"

**Cause**: Missing dependencies or initialization error

**Solution**:

1. Run with direct Python debugging:

```bash
python3 -u ~/ros2_ws/src/my_package/my_package/node.py
```

2. This shows traceback

3. Fix the error, test node independently

4. Then use launch file again

### Error: "Nodes not communicating"

**Symptoms**: Publisher sends, subscriber doesn't receive

**Diagnosis**:

```bash
# 1. Check topic exists
ros2 topic list | grep <topic_name>

# 2. Check topic has data
ros2 topic echo <topic_name>

# 3. Check node names
ros2 node list

# 4. Check connections
rqt_graph

# 5. Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be same for all nodes
```

**Solution**: Usually ROS_DOMAIN_ID mismatch

```bash
export ROS_DOMAIN_ID=0
ros2 launch turtle_control_system.launch.py
```

---

## Best Practices

### 1. Use Parameters for Configuration

```python
# GOOD: Configurable
Node(
    ...,
    parameters=[{'speed': LaunchConfiguration('speed')}]
)

# BAD: Hardcoded
Node(
    ...,
    parameters=[{'speed': 1.0}]
)
```

### 2. Output to Console

```python
# GOOD: See all output
Node(..., output='screen')

# BAD: No output
Node(...)
```

### 3. Startup Order

Use launch actions to control order:

```python
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnProcessStart

# Start B only after A starts
register_event = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=node_a,
        on_start=[node_b]
    )
)
```

### 4. Remapping Topics

Change topic names in launch file:

```python
Node(
    ...,
    remappings=[
        ('/turtle1/cmd_vel', '/my_robot/cmd_vel'),
        ('/turtle1/pose', '/my_robot/odometry')
    ]
)
```

---

## Layer 2: AI Collaboration Prompts

### Prompt 1: Launch File Scaling

"Ask Claude: 'How would you modify a launch file to start 10 instances of the same node with different configurations?'"

**What you're exploring**: Parameterized multi-node systems

### Prompt 2: Namespace Management

"Ask Claude: 'What is a ROS 2 namespace? How does it help organize multi-robot systems?'"

**What you're exploring**: System architecture at scale

### Prompt 3: Debugging Multi-Node Issues

"Ask Claude: 'When two nodes claim to publish the same topic, what happens? How do you debug this with ROS 2 tools?'"

**What you're exploring**: Namespace conflicts and debugging

---

## Self-Assessment Checklist

- [ ] I understand the purpose of launch files
- [ ] I can write a launch file that starts multiple nodes
- [ ] I can pass parameters to nodes via launch files
- [ ] I can visualize system architecture with rqt_graph
- [ ] I can debug communication issues between nodes
- [ ] I understand startup ordering and dependencies
- [ ] I can modify topic names with remapping
- [ ] I can read and understand ROS 2 launch file syntax

---

## Next Steps

**You've completed Lessons 1-7!** You have all the foundational knowledge for ROS 2 robotics.

**Next**: The **Capstone Project** combines everything into a complete 3-node robotics system.

**Optional Challenge**: Extend the turtle control system:

1. Add a fourth node: "Boundary Guardian" that prevents turtle from leaving the 0-11 area
2. Stop the turtle if it gets close to boundaries
3. Publish warnings to `/system/warnings` topic

---

**End of Lesson 7: Multi-Node Systems & Launch Files**


</PersonalizedLesson>
