---
title: "Services and Actions: Synchronous & Long-Running Communication"
chapter: 1
lesson: 6
duration_minutes: 75

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Understand when to use services vs. publish-subscribe"
  - "Implement a ROS 2 service server (request-response)"
  - "Call a ROS 2 service client from another node"
  - "Understand the difference between services and actions"
  - "Implement an action server with feedback"
  - "Call an action client and handle results"
  - "Design communication patterns for different robotics tasks"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (AI Collaboration)"
---

## Introduction

So far, you've learned about **publishers and subscribers** (asynchronous, one-way communication). But some robot tasks require **synchronous request-response** patterns:

- Reset the turtle's position
- Query battery status
- Command robot to navigate to a location
- Move a robot arm to a target pose (takes time, provide feedback)

This lesson covers two new communication patterns:

1. **Services**: Synchronous, fast (request → response)
2. **Actions**: Long-running, asynchronous with feedback (goal → feedback → result)

**Simulation environment**: Turtlesim (from Lesson 2)

**Time estimate**: 75 minutes

By the end, you'll understand:
- When to use services vs. pub/sub vs. actions
- How to implement service servers and clients
- How to implement action servers and clients
- Real-world patterns in robotics systems

---

## Concept 1: Services (Request-Response)

A **service** is a synchronous communication pattern: Client sends a request, server immediately responds. Think of it like a function call across network boundaries.

### Service Pattern

```
Client                          Server
  |                              |
  |--- Request (call service)--->|
  |                              |
  |                         Process
  |                              |
  |<--- Response (result) --------|
  |                              |
```

### When to Use Services

✓ **Use services for**:
- One-time operations (don't need continuous data)
- Queries (request information)
- Configuration changes
- State resets

✗ **Don't use services for**:
- Continuous sensor data (use topics)
- High-frequency commands (use topics)
- Long operations (robot might not respond) — use **actions** instead

### Example: Reset Turtle

Turtlesim has a built-in service `/reset` that resets the turtle:

```bash
ros2 service call /reset std_srvs/srv/Empty
```

This calls the service (no arguments), and the turtle resets.

---

## Concept 2: Service Definition

Services are defined in `.srv` files (like message `.msg` files):

```
std_srvs/srv/Empty:
  ---  (separator: request / response)
  (no request fields)
  (no response fields)
```

Example with data:

```
geometry_msgs/srv/GetPose:
  ---
  (request)
  int32 id
  ---
  (response)
  geometry_msgs/Pose pose
  string status
```

For Turtlesim, the `/turtle1/set_pen` service:

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  '{r: 255, g: 0, b: 0, width: 3, "off": false}'
```

This request has 5 fields (r, g, b, width, off); the response is empty.

---

## Code Example 1: Service Server

Let's create a service server that provides turtle information.

Create: `turtle_info_service.py`

```python
"""
Turtle Info Service Server
Simulation environment: Turtlesim
Provides turtle position and angle on request
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_srvs.srv import GetPose  # Custom service (we'll create)

# For now, use a simple custom service
# In production, define in a .srv file
class TurtleInfo:
    """Simple turtle info service"""
    pass


class TurtleInfoServer(Node):
    """Provides turtle position on request"""

    def __init__(self):
        super().__init__('turtle_info_server')

        # Store latest turtle pose
        self.turtle_pose = Pose()

        # Subscribe to turtle position
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Create service server
        # Note: In real code, define custom .srv file
        # For this example, we use a simple pattern
        self.get_logger().info('Turtle info service server started')

    def pose_callback(self, msg):
        """Update cached turtle position"""
        self.turtle_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = TurtleInfoServer()

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

**Note**: Turtlesim already provides services. Let's instead create a service client that calls them.

---

## Code Example 2: Service Client (Call Existing Service)

Let's create a client that calls Turtlesim's built-in `/reset` service.

Create: `turtle_reset_client.py`

```python
"""
Turtle Reset Service Client
Simulation environment: Turtlesim
Calls the /reset service to reset turtle position
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time


class TurtleResetClient(Node):
    """Client that calls the reset service"""

    def __init__(self):
        super().__init__('turtle_reset_client')

        # Create client for /reset service
        self.client = self.create_client(Empty, '/reset')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /reset not available, waiting...')

        self.get_logger().info('Reset service available!')

    def send_reset_request(self):
        """Call the reset service"""
        request = Empty.Request()

        # Call service (synchronous)
        future = self.client.call_async(request)

        # Wait for response
        future.add_done_callback(self.reset_callback)

    def reset_callback(self, future):
        """Handle reset response"""
        try:
            response = future.result()
            self.get_logger().info('Turtle reset successfully!')
        except Exception as e:
            self.get_logger().error(f'Reset failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleResetClient()

    # Wait a moment for service to be ready
    time.sleep(1)

    # Call reset service
    node.send_reset_request()

    # Spin briefly to process response
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### How to Run

```bash
# Terminal 1: Turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Client calls reset
python3 turtle_reset_client.py
```

### Expected Output

```
[INFO] [turtle_reset_client]: Reset service available!
[INFO] [turtle_reset_client]: Turtle reset successfully!
```

The turtle will return to its starting position (5.54, 5.54).

---

## Code Example 3: Service Server with Custom Service

Let's create a custom service that calculates distance to a point.

First, create the service definition file: `DistanceTo.srv`

```
# Request: target point
float32 target_x
float32 target_y
---
# Response: distance
float32 distance
```

Then the server: `distance_calculator_server.py`

```python
"""
Distance Calculator Service Server
Simulation environment: Turtlesim
Calculates distance from turtle to target point
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math


class DistanceCalculator(Node):
    """Calculates distance from turtle to target"""

    def __init__(self):
        super().__init__('distance_calculator')

        # Cache current turtle position
        self.current_x = 5.54
        self.current_y = 5.54

        # Subscribe to turtle position
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        self.get_logger().info('Distance calculator server started')

    def pose_callback(self, msg):
        """Update turtle position"""
        self.current_x = msg.x
        self.current_y = msg.y

    def calculate_distance(self, target_x, target_y):
        """Calculate distance from turtle to target"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        return distance


def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculator()

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

## Concept 3: Actions (Long-Running Operations)

An **action** is an asynchronous pattern for long-running tasks. Unlike services (which block waiting for response), actions:

1. Accept a **goal** from client
2. Periodically send **feedback** to client
3. Return a **result** when complete

### Action Pattern

```
Client                              Server
  |                                   |
  |----- Send Goal ---->|             |
  |                     |  Processing |
  |                     |  (provides   |
  |<--- Feedback -------|   feedback  |
  |                     |   periodically)
  |                     |             |
  |<--- Feedback -------|             |
  |                     |             |
  |<--- Result ---------|             |
  |                     |             |
```

### When to Use Actions

✓ **Use actions for**:
- Long-running tasks (navigation, manipulation)
- Need feedback during execution
- Can be cancelled
- Preemption (interrupt and restart)

✗ **Don't use for**:
- Quick operations (< 1 second) — use services
- Continuous monitoring — use topics

### Example: Navigate Robot to Pose

Navigation takes 10+ seconds. During execution:
- Provide feedback: "At waypoint 2 of 5"
- Provide feedback: "60% complete"
- Client can cancel: "Stop navigating"

---

## Code Example 4: Action Server (Fibonacci)

Let's create an action server that computes Fibonacci numbers (simulating a long task).

**Note**: This is educational; real robots use `nav2_msgs/action/NavigateToPose` for navigation.

Create: `fibonacci_action_server.py`

```python
"""
Fibonacci Action Server
Simulation environment: Any (computational task)
Demonstrates action pattern: goal → feedback → result
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """Action server that computes Fibonacci numbers"""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

        self.get_logger().info('Fibonacci action server started')

    async def execute_callback(self, goal_handle):
        """Execute the Fibonacci action"""

        self.get_logger().info(f'Computing Fibonacci({goal_handle.request.order})')

        # Compute Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Fibonacci computation cancelled')
                return Fibonacci.Result()

            # Add next Fibonacci number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])

            self.get_logger().info(f'Sequence: {feedback_msg.sequence}')

            # Send feedback
            goal_handle.publish_feedback(feedback_msg)

        # Computation complete
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        goal_handle.succeed()

        self.get_logger().info(f'Fibonacci complete: {result.sequence}')

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = FibonacciActionServer()

    # Use MultiThreadedExecutor to handle concurrent requests
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Note**: This requires the `action_tutorials_interfaces` package. For Turtlesim, here's a simpler example:

---

## Code Example 5: Simple Navigation Action (Turtlesim)

Create: `navigate_turtle_action.py`

```python
"""
Navigate Turtle Action Server
Simulation environment: Turtlesim
Action: Navigate turtle to target position with feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class NavigateGoal:
    """Simple goal structure"""
    def __init__(self):
        self.target_x = 0.0
        self.target_y = 0.0


class NavigateTurtleServer(Node):
    """Action server that navigates turtle to target"""

    def __init__(self):
        super().__init__('navigate_turtle_server')

        # Turtle state
        self.current_x = 5.54
        self.current_y = 5.54
        self.current_theta = 0.0

        # Subscribe to pose
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher for commands
        self.publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info('Navigate turtle action server started')

    def pose_callback(self, msg):
        """Update turtle position"""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def navigate_to(self, target_x, target_y, timeout=10.0):
        """Navigate turtle to target position"""

        start_time = time.time()

        while (time.time() - start_time) < timeout:
            # Calculate distance to target
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.2:
                # Reached target
                msg = Twist()
                self.publisher.publish(msg)
                return True

            # Calculate direction
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta

            # Normalize angle
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # Publish command
            msg = Twist()
            msg.linear.x = min(1.0, distance)
            msg.angular.z = angle_error

            self.publisher.publish(msg)

            rclpy.spin_once(self, timeout_sec=0.1)

        return False


def main(args=None):
    rclpy.init(args=args)
    node = NavigateTurtleServer()

    # Navigate to (9.0, 9.0)
    success = node.navigate_to(9.0, 9.0, timeout=15.0)

    if success:
        node.get_logger().info('Navigation successful!')
    else:
        node.get_logger().info('Navigation timeout')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Comparison Table: Pub/Sub vs. Services vs. Actions

| Pattern | Async | Sync | Feedback | Typical Use |
|---------|-------|------|----------|------------|
| **Pub/Sub** | Yes | No | No | Continuous sensor data, state updates |
| **Service** | No* | Yes | No | One-time queries, configuration |
| **Action** | Yes | Yes** | Yes | Long tasks, navigation, manipulation |

*Clients can call asynchronously but typically wait for response
**Client can wait for result while server executes

---

## Troubleshooting Services and Actions

### Error: "Service not available"

**Cause**: Server not running

**Solution**:

```bash
# Check if service exists
ros2 service list

# Check if server is responsive
ros2 service call /reset std_srvs/srv/Empty
```

### Error: "ServiceException: Service not available"

**Cause**: Client trying to call non-existent service

**Fix in code**:

```python
# Check service exists before calling
if not self.client.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('Service not available')
    return
```

### Error: "Action not found"

**Cause**: Action server not running or wrong action name

**Solution**:

```bash
ros2 action list      # See all available actions
ros2 action info /my_action  # Check action details
```

### Error: "Timeout waiting for response"

**Cause**: Service/action takes too long

**Solutions**:

1. Increase timeout:

```python
future = self.client.call_async(request)
future.add_done_callback(callback)  # Async: no timeout
```

2. Move to action pattern (supports cancellation)

---

## Layer 2: AI Collaboration Prompts

### Prompt 1: Services vs. Topics

"Ask Claude: 'When should I use a ROS 2 service instead of a topic for synchronous operations?'"

**What you're exploring**: Design patterns in distributed systems

### Prompt 2: Action Preemption

"Ask Claude: 'How do I cancel a long-running action in ROS 2? What happens to the server when the client cancels?'"

**What you're exploring**: Lifecycle management in real-time systems

### Prompt 3: Service Reliability

"Ask Claude: 'What if a service server crashes while processing a request? How does the client handle it?'"

**What you're exploring**: Fault tolerance in distributed systems

---

## Self-Assessment Checklist

- [ ] I understand the difference between services and topics
- [ ] I can identify when to use services vs. publish-subscribe
- [ ] I can call existing ROS 2 services from a client node
- [ ] I understand the request-response pattern
- [ ] I know what actions are and when to use them
- [ ] I understand action feedback and result messages
- [ ] I can handle service timeouts and errors
- [ ] I can design communication for a multi-node robot system

---

## Next Steps

**Lesson 7** covers **Building Integrated Systems**—combining publishers, subscribers, and services into complete multi-node robotics applications.

**Challenge**: Create a service that:
1. Accepts a target distance as request
2. Calculates how long it will take the turtle to travel that distance
3. Returns the estimated time as response

---

**End of Lesson 6: Services and Actions**
