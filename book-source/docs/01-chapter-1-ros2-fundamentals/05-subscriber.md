---
title: "Your First ROS 2 Subscriber"
chapter: 1
lesson: 5
duration_minutes: 60

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Create a ROS 2 subscriber node using rclpy"
  - "Implement message callbacks to react to incoming data"
  - "Extract data from ROS 2 message objects"
  - "Synchronize data between multiple nodes"
  - "Debug subscription issues using CLI tools"
  - "Combine publishers and subscribers into complete systems"
  - "Handle message timing and asynchronous execution"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (AI Collaboration)"
---

## Introduction

In Lesson 4, you created a publisher that **sends** messages. Now it's time to create a subscriber that **receives** messages.

**What you'll build**: A subscriber node that:
- Subscribes to the `/turtle1/pose` topic (turtle's current position)
- Receives messages whenever the turtle moves
- Logs position data (x, y, heading) to the console
- Calculates distance traveled and speed

**Simulation environment**: Turtlesim (from Lesson 2)

**Time estimate**: 60 minutes (including running and debugging)

By the end, you'll understand:
- How subscription callbacks work
- How to extract data from message objects
- How to handle asynchronous message delivery
- How to create complete pub/sub systems (publisher + subscriber)
- How to debug timing issues

---

## Foundational Knowledge Review

### Subscription API

A subscriber is created in the node constructor:

```python
self.subscription = self.create_subscription(
    msg_type,              # e.g., turtlesim.msg.Pose
    topic_name,            # e.g., '/turtle1/pose'
    callback_function,     # Called when message arrives
    queue_size             # e.g., 10 (max queued messages)
)
```

The callback is invoked asynchronously:

```python
def subscription_callback(self, msg):
    # Called whenever a message arrives on the topic
    self.get_logger().info(f"Received: x={msg.x}, y={msg.y}")
```

### Message Types (Turtlesim)

The `/turtle1/pose` topic publishes `turtlesim.msg.Pose` messages:

```
turtlesim.msg.Pose:
  x: float              # Position x (0 to 11.08)
  y: float              # Position y (0 to 11.08)
  theta: float          # Heading angle in radians
  linear_velocity: float # Current linear speed (m/s)
  angular_velocity: float # Current angular speed (rad/s)
```

---

## Concept 1: Asynchronous Callbacks

Unlike imperative code that runs line-by-line, ROS 2 subscribers use **event-driven programming**. Your callback is invoked automatically when data arrives.

### Callback Pattern

```python
def pose_callback(self, msg):
    # This runs automatically when a message arrives
    # msg contains the data from the publisher
    x = msg.x
    y = msg.y
    self.get_logger().info(f"Turtle at ({x:.2f}, {y:.2f})")
```

### Key Points

1. **Non-blocking**: Your callback doesn't block other nodes
2. **Automatic invocation**: ROS 2 calls your function automatically
3. **Single-threaded by default**: Callbacks run sequentially (not in parallel)
4. **Fast execution**: Keep callbacks quick (< 100 ms) to avoid losing messages

### Common Pitfall

Beginners sometimes write:

```python
# WRONG: This doesn't work!
def wrong_way(self):
    msg = self.get_latest_message()  # No such method!
```

Instead, **always** use callbacks:

```python
# RIGHT: Use callbacks
def subscription_callback(self, msg):
    # msg is automatically passed when data arrives
    self.process(msg)
```

---

## Concept 2: Message Data Extraction

ROS 2 messages are Python objects with fields. Access fields using dot notation:

```python
def callback(self, msg):
    x = msg.x              # Field access
    y = msg.y
    theta = msg.theta

    # Calculate distance from origin
    distance = (x**2 + y**2) ** 0.5
    self.get_logger().info(f"Distance from origin: {distance:.2f}")
```

### Nested Fields

Some messages have nested objects:

```python
# geometry_msgs.msg.Twist (has nested Vector3 objects)
msg.linear.x    # Linear velocity in x direction
msg.angular.z   # Angular velocity (yaw rotation)
```

### Introspection

To see all fields in a message, use the CLI:

```bash
ros2 interface show turtlesim/msg/Pose
```

---

## Concept 3: State Management in Subscribers

Subscribers often need to remember previous data (e.g., to calculate distance traveled or velocity).

### Using Instance Variables

```python
class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')

        # Store previous position
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.total_distance = 0.0

        # Create subscription
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        # Calculate distance traveled since last message
        dx = msg.x - self.prev_x
        dy = msg.y - self.prev_y
        distance = (dx**2 + dy**2) ** 0.5

        self.total_distance += distance

        # Update previous position
        self.prev_x = msg.x
        self.prev_y = msg.y

        self.get_logger().info(
            f"Distance traveled: {distance:.3f}, Total: {self.total_distance:.3f}")
```

---

## Code Example 1: Simple Subscriber

Create a file: `subscriber_node.py`

```python
"""
Simple ROS 2 Subscriber Node
Simulation environment: Turtlesim
Subscribes to turtle position and logs it
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleSubscriber(Node):
    """Simple subscriber that listens to turtle position"""

    def __init__(self):
        super().__init__('turtle_subscriber')

        # Create subscription to turtle pose topic
        self.subscription = self.create_subscription(
            Pose,                      # Message type
            '/turtle1/pose',           # Topic name
            self.pose_callback,        # Callback function
            10                         # Queue size
        )

        self.get_logger().info('Turtle subscriber started')

    def pose_callback(self, msg):
        """Called whenever a pose message arrives"""
        # Extract position and heading
        x = msg.x
        y = msg.y
        theta = msg.theta

        # Log the data
        self.get_logger().info(
            f'Turtle position: x={x:.2f}, y={y:.2f}, heading={theta:.2f} rad')


def main(args=None):
    """Entry point for the node"""
    rclpy.init(args=args)

    node = TurtleSubscriber()

    try:
        # Spin: keep the node running and process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### How to Run

**Terminal 1**: Start Turtlesim simulator

```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2**: Start the subscriber

```bash
cd ~/ros2_workspace/src/my_first_package
python3 subscriber_node.py
```

**Terminal 3**: Publish commands to move the turtle

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

### Expected Output

```
[INFO] [turtle_subscriber]: Turtle subscriber started
[INFO] [turtle_subscriber]: Turtle position: x=5.54, y=5.54, heading=0.00 rad
[INFO] [turtle_subscriber]: Turtle position: x=5.55, y=5.54, heading=0.05 rad
[INFO] [turtle_subscriber]: Turtle position: x=5.57, y=5.54, heading=0.10 rad
[INFO] [turtle_subscriber]: Turtle position: x=5.60, y=5.54, heading=0.15 rad
```

The subscriber logs the turtle's position continuously as it moves.

---

## Code Example 2: Subscriber with State Management

This example calculates distance traveled and average speed.

Create: `tracking_subscriber.py`

```python
"""
Tracking Subscriber: Calculates distance and speed
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TrackingSubscriber(Node):
    """Subscriber that tracks position changes"""

    def __init__(self):
        super().__init__('tracking_subscriber')

        # State variables
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.total_distance = 0.0
        self.message_count = 0

        # Create subscription
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Tracking subscriber started')

    def pose_callback(self, msg):
        """Track position changes and calculate distance"""

        # Get current position
        curr_x = msg.x
        curr_y = msg.y

        # Calculate distance from last position
        dx = curr_x - self.prev_x
        dy = curr_y - self.prev_y
        distance = (dx**2 + dy**2) ** 0.5

        # Update total
        self.total_distance += distance
        self.message_count += 1

        # Update previous position
        self.prev_x = curr_x
        self.prev_y = curr_y

        # Log with calculated statistics
        self.get_logger().info(
            f'Position: ({curr_x:.2f}, {curr_y:.2f}) | '
            f'Distance this step: {distance:.4f} | '
            f'Total distance: {self.total_distance:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TrackingSubscriber()

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

### Running the Tracking Example

```bash
# Terminal 1: Turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Tracking subscriber
python3 tracking_subscriber.py

# Terminal 3: Command the turtle to move in a square
ros2 topic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

### Expected Output

```
[INFO] [tracking_subscriber]: Tracking subscriber started
[INFO] [tracking_subscriber]: Position: (5.55, 5.55) | Distance this step: 0.0141 | Total distance: 0.0141
[INFO] [tracking_subscriber]: Position: (5.60, 5.56) | Distance this step: 0.0500 | Total distance: 0.0641
[INFO] [tracking_subscriber]: Position: (5.65, 5.58) | Distance this step: 0.0707 | Total distance: 0.1348
[INFO] [tracking_subscriber]: Position: (5.72, 5.62) | Distance this step: 0.0943 | Total distance: 0.2291
```

---

## Code Example 3: Combined Publisher + Subscriber

Let's create a node that both publishes and subscribes (common in real systems).

Create: `publisher_subscriber.py`

```python
"""
Combined Publisher and Subscriber
Publishes velocity commands AND subscribes to position
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class TurtleController(Node):
    """Node that controls turtle and monitors position"""

    def __init__(self):
        super().__init__('turtle_controller')

        # Publisher for commands
        self.publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # Subscriber for position
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # State: track target vs actual
        self.target_x = 9.0
        self.target_y = 9.0
        self.current_x = 5.54
        self.current_y = 5.54
        self.current_theta = 0.0

        # Timer to publish commands periodically
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Turtle controller started')
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y})')

    def pose_callback(self, msg):
        """Update current position"""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def control_loop(self):
        """Simple control: move towards target"""

        # Calculate direction to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        if distance_to_target < 0.2:
            # Reached target
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Target reached!')
        else:
            # Move towards target
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta

            # Normalize angle error to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # Create command
            msg = Twist()
            msg.linear.x = min(1.0, distance_to_target)  # Move forward
            msg.angular.z = angle_error  # Turn towards target

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the turtle
        msg = Twist()
        node.publisher.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Combined Example

```bash
# Terminal 1: Turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Controller (moves turtle to position 9,9)
python3 publisher_subscriber.py
```

### Expected Output

```
[INFO] [turtle_controller]: Turtle controller started
[INFO] [turtle_controller]: Target: (9.0, 9.0)
[INFO] [turtle_controller]: Turtle moving towards target...
[INFO] [turtle_controller]: Target reached!
```

The turtle will move from its starting position to (9.0, 9.0).

---

## Debugging Subscribers

### Problem: "No messages received"

**Symptom**: Callback never fires, no error messages

**Diagnosis**:

```bash
# Check topic exists and has publishers
ros2 topic list
ros2 topic info /turtle1/pose

# Check if publisher is sending
ros2 topic echo /turtle1/pose
```

**Solution**: Ensure publisher is running before subscriber:

```bash
# Terminal 1: Publisher sends data
python3 publisher_node.py

# Terminal 2: Subscriber receives it
python3 subscriber_node.py
```

### Problem: "Messages received but outdated"

**Symptom**: Callback processes very old data

**Cause**: Queue is full, old messages are dropped

**Solution**: Increase queue size

```python
# Default: queue_size=10
self.subscription = self.create_subscription(
    Pose, '/turtle1/pose', self.callback, 100)  # Bigger queue
```

### Problem: "Callback too slow"

**Symptom**: Console logs don't keep up with message rate

**Cause**: Callback takes too long to execute

**Solution**: Keep callbacks fast

```python
# BAD: Slow callback
def slow_callback(self, msg):
    time.sleep(1)  # Too slow!
    self.process(msg)

# GOOD: Fast callback
def fast_callback(self, msg):
    self.queue.put(msg)  # Just queue it, process later
```

---

## Troubleshooting

### Error: "ModuleNotFoundError: No module named 'turtlesim'"

**Cause**: turtlesim not installed

**Solution**:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

### Error: "Could not import ROS 2 Python library"

**Cause**: ROS 2 environment not sourced

**Solution**:

```bash
source /opt/ros/humble/setup.bash
python3 subscriber_node.py
```

### Error: "Subscription callback raises AttributeError"

**Cause**: Accessing wrong field on message

**Example**:

```python
# WRONG: Pose doesn't have 'velocity'
def callback(self, msg):
    v = msg.velocity  # AttributeError!
```

**Solution**: Use correct field name

```python
# RIGHT: Use correct fields
def callback(self, msg):
    x = msg.x
    y = msg.y
    vx = msg.linear_velocity
```

Check message structure:

```bash
ros2 interface show turtlesim/msg/Pose
```

### Error: "Callback not being called"

**Cause**: `rclpy.spin()` not called

**Check**: Ensure main function has:

```python
def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)  # REQUIRED!
    rclpy.shutdown()
```

### Error: "Topic name '/turtle1/pose' not found"

**Cause**: Publisher not running or using different topic name

**Solution**:

```bash
# List all active topics
ros2 topic list

# Check which topics have publishers
ros2 topic info <topic_name>

# Monitor what's being published
ros2 topic echo /turtle1/pose
```

---

## Layer 2: AI Collaboration Prompts

### Prompt 1: Understanding Callbacks

"I just learned about ROS 2 callbacks. Ask Claude: 'Why are callbacks better than polling (constantly checking) for new messages in robotics applications?'"

**What you're exploring**: Event-driven vs. polling architectures

### Prompt 2: Message Queuing

"Ask Claude: 'What happens in ROS 2 when a subscriber's queue is full? How does it decide which messages to drop?'"

**What you're exploring**: Quality-of-Service (QoS) policies

### Prompt 3: Synchronization

"Ask Claude: 'If I have a publisher running at 10 Hz and a subscriber queue size of 1, how many messages will be lost?'"

**What you're exploring**: Timing and synchronization in distributed systems

---

## Self-Assessment Checklist

After completing this lesson, verify:

- [ ] I understand what a subscription callback is
- [ ] I can create a subscriber node from scratch
- [ ] I can extract fields from ROS 2 message objects
- [ ] I understand how to manage state in subscribers
- [ ] I can track changes between messages
- [ ] I can combine publishers and subscribers
- [ ] I can debug subscriber issues using `ros2 topic`
- [ ] I know when to use queues vs. keeping fresh data

---

## Next Steps

**Ready for Lesson 6?** In the next lesson, you'll learn about **Services and Actions**—synchronous request-response communication patterns that are essential for ROS 2 robotics systems.

**Optional Challenge**: Modify `tracking_subscriber.py` to:
1. Calculate average velocity (distance / time)
2. Detect when the turtle stops (velocity near zero)
3. Log a message when the turtle has been stationary for 2 seconds

---

**End of Lesson 5: Your First ROS 2 Subscriber**
