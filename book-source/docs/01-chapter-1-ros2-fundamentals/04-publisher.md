---
title: "Your First ROS 2 Publisher"
chapter: 1
lesson: 4
duration_minutes: 60
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []
learning_objectives:
  - "Create a ROS 2 node class using rclpy.Node"
  - "Implement a publisher and publish messages to a topic"
  - "Use timer callbacks to trigger periodic message publishing"
  - "Write a complete, runnable ROS 2 node in Python"
  - "Debug publisher issues using ROS 2 CLI tools"
  - "Apply error handling and logging in ROS 2 nodes"

primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (AI Collaboration)"
---

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/04-publisher.md">

## Introduction

Now it's time to write your first ROS 2 code! In this lesson, you'll create a Python node that publishes velocity commands to Turtlesim, making the turtle move.

**What you'll build**: A `SimpleTurtlePublisher` node that:
- Creates a publisher on the `/turtle1/cmd_vel` topic
- Sends `Twist` messages (velocity commands) at 10 Hz
- Makes the turtle move forward indefinitely
- Handles graceful shutdown

**Simulation environment**: Turtlesim (from Lesson 2)

**Time estimate**: 60 minutes (including running and debugging)

By the end, you'll understand:
- How to structure a ROS 2 node class
- How publishers work (create, initialize, publish)
- How timer callbacks enable periodic publishing
- How to debug using `ros2 topic echo` and `ros2 topic hz`

---

## Foundational Knowledge Review

Before diving into code, let's review key concepts from Lessons 1-3:

### Node Structure (From Lesson 3)

Every ROS 2 node:
1. Inherits from `rclpy.node.Node`
2. Has an `__init__` method that sets up publishers, subscribers, and timers
3. Has callback methods for handling events (timers, subscriptions)
4. Is initialized with `rclpy.init()`
5. Runs with `rclpy.spin(node)`

### Publisher API (New)

A publisher is created in the node constructor:

```python
self.publisher = self.create_publisher(
    msg_type,          # e.g., geometry_msgs.msg.Twist
    topic_name,        # e.g., '/turtle1/cmd_vel'
    queue_size         # e.g., 10 (max queued messages)
)
```

Publishing a message:

```python
msg = Twist()          # Create message
msg.linear.x = 1.0     # Set fields
self.publisher.publish(msg)  # Send it
```

### Timer Callbacks (New)

For periodic tasks, create a timer:

```python
self.timer = self.create_timer(
    0.1,                    # Period in seconds (10 Hz = 0.1s)
    self.timer_callback    # Function to call periodically
)
```

The callback runs every 0.1 seconds:

```python
def timer_callback(self):
    msg = Twist()
    msg.linear.x = 1.0
    self.publisher.publish(msg)
```

### Message Types (From Lesson 3)

The `Twist` message represents velocity:

```
geometry_msgs.msg.Twist:
  linear:
    x: float (m/s)       # forward/backward
    y: float (m/s)       # sideways
    z: float (m/s)       # vertical
  angular:
    x: float (rad/s)     # roll
    y: float (rad/s)     # pitch
    z: float (rad/s)     # yaw (turning)
```

For Turtlesim:
- Set `linear.x` to move forward (positive) or backward (negative)
- Set `angular.z` to turn (positive = counter-clockwise)

---

## Concept 1: Python Package Structure for ROS 2

ROS 2 nodes live in **packages**. A package is a directory with metadata about your node.

### Directory Structure

```
ros2_workspace/
├── src/
│   └── my_first_package/
│       ├── my_first_package/
│       │   ├── __init__.py
│       │   └── publisher_node.py      # Your node code
│       ├── setup.py
│       ├── setup.cfg
│       ├── package.xml
│       └── resource/
│           └── my_first_package
└── install/
└── build/
```

**Key files**:
- `package.xml`: Package metadata (name, version, dependencies, maintainer)
- `setup.py`: Python package configuration (entry points, package data)
- `setup.cfg`: Python package metadata
- `publisher_node.py`: Your actual node code

We'll create this structure in a moment. For now, understand that ROS 2 expects this layout.

---

## Concept 2: The rclpy.Node Base Class

Every ROS 2 node inherits from `rclpy.node.Node`. Let's understand the key methods:

### Creating a Node Class

```python
from rclpy.node import Node

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')  # Node name

        # Create publishers, subscribers, timers here
        self.publisher = self.create_publisher(...)
        self.timer = self.create_timer(...)
```

The `__init__` method:
1. Calls `super().__init__()` with the node name
2. Sets up publishers, subscribers, parameters, timers
3. Does NOT enter the spin loop yet

### Key Methods in Node

| Method | Purpose | Example |
|--------|---------|---------|
| `create_publisher()` | Create a publisher | `self.create_publisher(Twist, '/cmd_vel', 10)` |
| `create_subscription()` | Create a subscriber | `self.create_subscription(Image, '/camera/image', callback, 10)` |
| `create_timer()` | Schedule periodic callback | `self.create_timer(0.1, self.callback)` |
| `get_logger()` | Get logger for this node | `self.get_logger().info("message")` |
| `declare_parameter()` | Declare a configurable parameter | `self.declare_parameter('speed', 1.0)` |

---

## Concept 3: Timer Callbacks and Periodic Publishing

### Understanding Timers

A timer calls a function at regular intervals:

```python
# Create timer: call self.timer_callback() every 0.1 seconds (10 Hz)
self.timer = self.create_timer(0.1, self.timer_callback)

def timer_callback(self):
    msg = Twist()
    msg.linear.x = 1.0
    self.publisher.publish(msg)
```

### Frequency vs. Period

- **Frequency** (Hz): Messages per second
  - 10 Hz = 10 messages/second
  - 1 Hz = 1 message/second
  - 30 Hz = 30 messages/second

- **Period** (seconds): Time between messages
  - 10 Hz → period = 1/10 = 0.1 seconds
  - 1 Hz → period = 1/1 = 1.0 seconds
  - 30 Hz → period = 1/30 ≈ 0.033 seconds

**Conversion**: `period = 1.0 / frequency`

### Why Periodic Publishing?

Sensor and command data needs to be sent continuously:
- **Sensor data** (30 Hz): Camera images, lidar scans
- **Control commands** (10-50 Hz): Motor commands, velocity
- **Status updates** (1 Hz): Battery level, temperature

If you publish once and stop, the subscriber sees one message and then nothing. Periodic publishing ensures continuous data flow.

---

## Code Example 1: Simple Publisher (Minimal)

Let's create your first node. This is the absolute minimum to understand the concepts.

### Create the Node File

Create a file at: `publisher_node.py`

```python
"""
Simple ROS 2 Publisher Node
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleTurtlePublisher(Node):
    """Publishes velocity commands to Turtlesim at 10 Hz."""

    def __init__(self):
        super().__init__('simple_turtle_publisher')

        # Create publisher on /turtle1/cmd_vel topic
        self.publisher = self.create_publisher(
            Twist,                  # Message type
            '/turtle1/cmd_vel',     # Topic name
            10                      # Queue size
        )

        # Create timer to call callback every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Publishes a Twist message every 0.1 seconds."""
        msg = Twist()
        msg.linear.x = 1.0      # 1 m/s forward
        msg.angular.z = 0.0     # No rotation

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Total lines**: 38 | **Concepts used**: 5

### Running the Code

Open two terminals:

**Terminal 1: Launch Turtlesim**
```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2: Run your publisher**
```bash
source /opt/ros/humble/setup.bash
cd path/to/publisher_node.py
python3 publisher_node.py
```

**Expected behavior**: The turtle moves forward continuously in a straight line.

### Verifying with ROS 2 Tools

Open a **third terminal** and verify the publisher is working:

```bash
source /opt/ros/humble/setup.bash

# Check topic exists
ros2 topic list | grep cmd_vel

# Watch messages
ros2 topic echo /turtle1/cmd_vel

# Check publish frequency
ros2 topic hz /turtle1/cmd_vel
```

**Expected output**:
```
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

And frequency should show ~10 Hz.

---

## Code Example 2: Publisher with Logging

Real-world nodes log their activity. Let's add logging to the publisher:

```python
"""
ROS 2 Publisher with Logging
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class LoggingTurtlePublisher(Node):
    """Publishes velocity commands with logging output."""

    def __init__(self):
        super().__init__('logging_turtle_publisher')

        # Get logger for this node
        self.get_logger().info('LoggingTurtlePublisher initializing...')

        # Create publisher
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Counter for messages
        self.msg_count = 0

        self.get_logger().info('LoggingTurtlePublisher ready!')

    def timer_callback(self):
        """Publishes velocity and logs every 10th message."""
        msg = Twist()
        msg.linear.x = 1.5      # 1.5 m/s forward
        msg.angular.z = 0.0     # No rotation

        self.publisher.publish(msg)

        self.msg_count += 1

        # Log every 10 messages (once per second at 10 Hz)
        if self.msg_count % 10 == 0:
            self.get_logger().info(
                f'Published {self.msg_count} messages. '
                f'Moving at {msg.linear.x} m/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LoggingTurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running**:
```bash
python3 logging_publisher.py
```

**Expected output**:
```
[INFO] [logging_turtle_publisher]: LoggingTurtlePublisher initializing...
[INFO] [logging_turtle_publisher]: LoggingTurtlePublisher ready!
[INFO] [logging_turtle_publisher]: Published 10 messages. Moving at 1.5 m/s
[INFO] [logging_turtle_publisher]: Published 20 messages. Moving at 1.5 m/s
[INFO] [logging_turtle_publisher]: Published 30 messages. Moving at 1.5 m/s
...
```

---

## Code Example 3: Publisher with Parameters

Real nodes use parameters for configuration. Let's make speed configurable:

```python
"""
ROS 2 Publisher with Parameters
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ParameterizedTurtlePublisher(Node):
    """Publisher with configurable speed parameter."""

    def __init__(self):
        super().__init__('parameterized_turtle_publisher')

        # Declare parameters with default values
        self.declare_parameter('linear_velocity', 1.0)   # Default: 1 m/s
        self.declare_parameter('angular_velocity', 0.0)  # Default: no rotation
        self.declare_parameter('publish_rate', 10.0)      # Default: 10 Hz

        self.get_logger().info('Parameters declared. Waiting for config...')

        # Get initial parameter values
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        pub_rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create timer based on publish_rate
        timer_period = 1.0 / pub_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Publishing at {pub_rate} Hz with '
            f'linear={self.linear_vel} m/s, '
            f'angular={self.angular_vel} rad/s'
        )

    def timer_callback(self):
        """Publishes with configured velocities."""
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_vel

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedTurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running with custom parameters**:
```bash
# Run with default parameters
python3 param_publisher.py

# Or set parameters via ROS 2
python3 param_publisher.py &
ros2 param set /parameterized_turtle_publisher linear_velocity 2.5
ros2 param set /parameterized_turtle_publisher angular_velocity 0.5
```

**Expected output**:
```
[INFO] [parameterized_turtle_publisher]: Parameters declared. Waiting for config...
[INFO] [parameterized_turtle_publisher]: Publishing at 10.0 Hz with linear=1.0 m/s, angular=0.0 rad/s
```

The turtle moves at 1 m/s straight, or you can change speed dynamically with `ros2 param set`.

---

## Code Example 4: Publisher with Error Handling

Production code needs error handling. Here's a robust version:

```python
"""
ROS 2 Publisher with Error Handling and Logging
Simulation environment: Turtlesim
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobustTurtlePublisher(Node):
    """Publisher with comprehensive error handling."""

    def __init__(self):
        super().__init__('robust_turtle_publisher')

        self.logger = self.get_logger()

        try:
            # Declare parameters
            self.declare_parameter('linear_velocity', 1.0)
            self.declare_parameter('angular_velocity', 0.0)
            self.declare_parameter('publish_rate', 10.0)

            # Validate parameter values
            self.linear_vel = self.get_parameter('linear_velocity').value
            self.angular_vel = self.get_parameter('angular_velocity').value
            pub_rate = self.get_parameter('publish_rate').value

            if pub_rate <= 0.0:
                raise ValueError(f'publish_rate must be positive, got {pub_rate}')

            # Create publisher
            self.publisher = self.create_publisher(
                Twist,
                '/turtle1/cmd_vel',
                10
            )

            # Create timer
            timer_period = 1.0 / pub_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

            # Statistics
            self.msg_count = 0
            self.error_count = 0

            self.logger.info(
                f'RobustTurtlePublisher initialized successfully. '
                f'Publishing at {pub_rate} Hz'
            )

        except Exception as e:
            self.logger.error(f'Initialization failed: {e}')
            raise

    def timer_callback(self):
        """Publishes with error handling."""
        try:
            msg = Twist()
            msg.linear.x = self.linear_vel
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.angular_vel

            # Validate message before publishing
            if not self._validate_message(msg):
                self.error_count += 1
                return

            self.publisher.publish(msg)
            self.msg_count += 1

            # Log statistics every 100 messages
            if self.msg_count % 100 == 0:
                self.logger.debug(
                    f'Published {self.msg_count} messages. '
                    f'Errors: {self.error_count}'
                )

        except Exception as e:
            self.logger.error(f'Error in timer_callback: {e}')
            self.error_count += 1

    @staticmethod
    def _validate_message(msg):
        """Validate message fields."""
        # Check that velocities are reasonable
        if abs(msg.linear.x) > 10.0:
            return False  # Limit to 10 m/s max
        if abs(msg.angular.z) > 3.14:
            return False  # Limit to 1π rad/s max
        return True


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RobustTurtlePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupt received, shutting down...')
    except Exception as e:
        print(f'Fatal error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Features**:
- Parameter validation (checks pub_rate > 0)
- Error handling in callbacks (doesn't crash on exceptions)
- Message validation (ensures velocities are reasonable)
- Statistics tracking (counts successful publishes and errors)
- Graceful shutdown with error status

---

## How to Run Your First Publisher

### Step 1: Create a Working Directory

```bash
mkdir -p ~/ros2_workspace/src/my_first_package/my_first_package
cd ~/ros2_workspace/src/my_first_package
```

### Step 2: Copy Code Example 1

Create `my_first_package/publisher_node.py` and copy Code Example 1 into it.

### Step 3: Create package.xml

Create `package.xml` in the package directory:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_first_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package with a publisher node</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 4: Create setup.py

Create `setup.py` in the package directory:

```python
from setuptools import find_packages, setup

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_cmake_core/cmake/package_cmake_template',
         ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My first ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_first_package.publisher_node:main',
        ],
    },
)
```

### Step 5: Create __init__.py

Create `my_first_package/__init__.py` (empty file):

```python
# Marker file for Python package
```

### Step 6: Build the Package

```bash
cd ~/ros2_workspace
colcon build --symlink-install
source install/setup.bash
```

**Expected output**:
```
Starting >>> my_first_package
Finished <<< my_first_package [0.0s]

Summary: 1 package finished [0.0s]
```

### Step 7: Run the Node

Now you can run it with `ros2 run`:

```bash
# In one terminal
ros2 run turtlesim turtlesim_node

# In another
ros2 run my_first_package publisher_node
```

Or directly:
```bash
python3 my_first_package/publisher_node.py
```

---

## Debugging Your Publisher

### Problem 1: "ModuleNotFoundError: No module named 'geometry_msgs'"

**Cause**: ROS 2 environment not sourced, or package not installed.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

### Problem 2: "Turtle doesn't move"

**Cause**: Topic name wrong, or turtlesim not running.

**Solution**:
1. Verify turtlesim is running: `ros2 node list` (should show `/turtlesim`)
2. Verify your topic: `ros2 topic list` (should show `/turtle1/cmd_vel`)
3. Check topic info: `ros2 topic info /turtle1/cmd_vel` (should show 1 publisher)

### Problem 3: "Connection timeout" or "Cannot connect to ROS network"

**Cause**: `rclpy.init()` not called, or ROS 2 daemon crashed.

**Solution**:
```bash
ros2 daemon stop
ros2 daemon start
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

### Problem 4: "Turtle moves but very slowly/fast"

**Cause**: Velocity value wrong.

**Solution**: Check your `linear.x` value:
- `linear.x = 0.5` → moves at 0.5 m/s (slow)
- `linear.x = 2.0` → moves at 2.0 m/s (fast)
- `linear.x = 0.0` → doesn't move

Adjust the value in your code and rerun.

### Problem 5: "Topics won't echo messages"

**Cause**: Publisher isn't being called, or timer isn't working.

**Solution**:
1. Add logging to timer_callback
2. Check timer period is correct
3. Verify `rclpy.spin()` is running (blocks the terminal)

---

## Self-Assessment Checklist

Before moving to Lesson 5, verify:

- [ ] I can explain the rclpy.Node class structure
- [ ] I know how to create a publisher with `create_publisher()`
- [ ] I understand timer callbacks and periodic publishing
- [ ] I can write a complete ROS 2 node from scratch
- [ ] I can run my node with `ros2 run` (after colcon build)
- [ ] I can verify my publisher works with `ros2 topic echo`
- [ ] I understand the frequency-to-period conversion (period = 1/frequency)
- [ ] I can debug common publisher issues using ROS 2 tools
- [ ] I recognize that pub/sub communication is asynchronous

If any checks failed, re-read the corresponding section.

---

## Further Exploration (Optional)

1. **Modify the Publisher**:
   - Make the turtle move in a circle (set both linear.x and angular.z)
   - Change publish rate to 5 Hz, 30 Hz, and observe the difference
   - Create a square motion pattern

2. **Advanced Logging**:
   - Use different log levels: `info()`, `warn()`, `error()`, `debug()`
   - Set logger level: `self.get_logger().set_level(logging.DEBUG)`

3. **ROS 2 Documentation**:
   - Publisher and Subscriber documentation: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
   - Geometry Msgs documentation: https://docs.ros.org/en/humble/Concepts/Intermediate-Concepts/About-Builtin-Interfaces.html

---

## What's Next?

**Lesson 5: Your First ROS 2 Subscriber** (Coming in Chapter 1, Lesson 5)

You'll learn:
- How to create a subscriber node
- How callbacks work when messages arrive
- How to build a complete pub/sub system (publisher + subscriber)
- How to synchronize data between multiple nodes

**Estimated time**: 60 minutes

**Prerequisites**: Understand publishers from this lesson

---

## Layer 2: AI Collaboration Prompts

### For Understanding Node Structure

> "I just wrote my first ROS 2 node. Ask Claude: 'What would happen if I forgot to call `rclpy.init()` before creating the node? Why is this necessary?'"

### For Debugging Issues

> "My turtle isn't moving even though my code looks right. Ask Claude: 'Walk me through the debugging process. What are all the places where things could go wrong between publishing a message and the turtle moving?'"

### For Advanced Features

> "Ask Claude: 'My publisher publishes 10 Hz, but Turtlesim expects continuous commands. What happens if the turtle receives no messages for 1 second? Does it stop or keep moving?'"

---

## Concept Count & CEFR Validation

**Total B1 Concepts Introduced**: 8

1. rclpy.Node class inheritance and structure
2. Publisher creation with `create_publisher()`
3. Message instantiation and field assignment
4. Publishing messages with `publisher.publish()`
5. Timer callbacks for periodic execution
6. Frequency and period conversion (Hz ↔ seconds)
7. Logging and debugging with `get_logger()`
8. Error handling and graceful shutdown

**CEFR B1 Alignment**: ✓ Requires understanding of node lifecycle (A2), adds hands-on coding with concrete output

**Code Complexity**: 4 working examples (increasing complexity from 38 to ~90 lines)

---

**Chapter 1, Lesson 4 Complete**

You've successfully written your first ROS 2 publisher!

Next: Lesson 5 — Your First ROS 2 Subscriber (Part of Chapter 1)


</PersonalizedLesson>
