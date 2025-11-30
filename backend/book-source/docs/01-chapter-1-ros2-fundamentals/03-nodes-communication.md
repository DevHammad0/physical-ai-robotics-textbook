---
title: "Nodes and Communication Patterns"
chapter: 1
lesson: 3
duration_minutes: 60

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "A2"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Explain the node lifecycle and how nodes initialize, run, and shut down"
  - "Distinguish between publish-subscribe (asynchronous) and service (synchronous) patterns"
  - "Use ROS 2 CLI tools to inspect running nodes and topics"
  - "Understand message types and how they enforce type safety"
  - "Apply ROS 2 naming conventions (nodes, topics, services)"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (AI Collaboration Hints)"
---

## Introduction

Now that your ROS 2 environment is working, it's time to understand the architecture that makes robotics possible. This lesson explores the fundamental building blocks: **nodes**, **topics**, **messages**, and **communication patterns**.

By the end of this lesson, you'll understand:
- How nodes are independent processes that communicate through topics
- Why asynchronous (pub/sub) and synchronous (services) patterns exist
- How to inspect a running ROS 2 system using CLI tools
- The relationship between message types, topics, and strong typing

**Simulation environment**: Turtlesim (we'll use it to observe real communication)

---

## Concept 1: Nodes and the Node Lifecycle

### What is a Node?

A **node** is an independent, executable process that:
- Runs continuously (until stopped)
- Performs one specific responsibility
- Communicates with other nodes through topics and services
- Can fail without crashing other nodes (fault isolation)

**Real-world analogy**: A node is like an employee in a company. An HR employee processes payroll, the marketing employee creates ads, and the sales employee closes deals. They communicate through emails (topics) and meetings (services), but each job is independent.

### Node Lifecycle: Four Stages

Every ROS 2 node goes through four stages:

```
┌──────────────┐
│   Created    │  Node object instantiated in memory
└──────┬───────┘
       │
       ↓
┌──────────────┐
│ Initialized  │  rclpy.init() called, connects to ROS 2 network
└──────┬───────┘
       │
       ↓
┌──────────────┐
│   Running    │  rclpy.spin() called, processes messages & timers
└──────┬───────┘
       │
       ↓
┌──────────────┐
│  Shutting    │  rclpy.shutdown() called, cleanup and exit
│   Down       │
└──────────────┘
```

### Node Lifecycle Details

#### Stage 1: Created
```python
from rclpy.node import Node

node = Node('my_node')  # Node object created, but not active
```

The node object exists in memory but doesn't connect to ROS 2 yet.

#### Stage 2: Initialized
```python
import rclpy

rclpy.init()  # Connects to ROS 2 network, becomes discoverable
node = Node('my_node')
```

After `rclpy.init()`, the node registers with ROS 2, and other nodes can see it.

#### Stage 3: Running
```python
rclpy.spin(node)  # Processes messages, timers, and callbacks
```

The node now:
- Listens for incoming messages on subscribed topics
- Calls timer callbacks (periodic tasks)
- Handles service requests
- Blocks until interrupted (Ctrl+C)

#### Stage 4: Shutting Down
```python
rclpy.shutdown()  # Cleanup and exit
```

The node:
- Closes all publishers and subscribers
- Deregisters from ROS 2 network
- Releases file handles and memory
- Exits gracefully

### Complete Node Lifecycle (Example)

```python
import rclpy

# Stage 1: Created
class MyNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('my_node')

# Stage 2: Initialized
rclpy.init()
node = MyNode()

# Stage 3: Running
rclpy.spin(node)

# Stage 4: Shutting Down (automatic after Ctrl+C)
rclpy.shutdown()
```

---

## Concept 2: Topics and Pub/Sub Communication

### What is a Topic?

A **topic** is a named communication channel (like a radio station):
- One or more publishers write messages to it
- One or more subscribers read messages from it
- Publishers and subscribers are **decoupled** (don't know about each other)
- Communication is **asynchronous** (non-blocking)

```
Publisher 1 ──┐
              ├──→ Topic: /sensor/data ──→ Subscriber 1
Publisher 2 ──┘                        ──→ Subscriber 2
                                       ──→ Subscriber 3
```

### Why Pub/Sub is Asynchronous

When a publisher sends a message, it doesn't wait for subscribers to receive it. It just publishes and continues executing.

```
Time →

Publisher:  publish()  publish()  publish()  publish()  (continues immediately)
              ↓          ↓          ↓          ↓
Subscriber A: [receives] [receives] (busy)    (receives)
Subscriber B:            [receives] [receives] [receives]
```

**Benefit**: If Subscriber A is temporarily busy, it doesn't block the publisher or Subscriber B.

### Topic Naming Conventions

ROS 2 uses hierarchical, descriptive topic names:

```
/robot_name/subsystem/sensor_type/data_type

/turtlebot4/camera/rgb/image_raw       # Robot camera
/turtlebot4/lidar/scan                 # 2D laser scanner
/turtlebot4/imu/data                   # Inertial measurement unit
/turtlebot4/motor/joint_state           # Joint angles
```

**Rules**:
- Start with `/` (absolute topic names)
- Use `snake_case` for multi-word names
- Be hierarchical and descriptive
- Avoid single-letter names

---

## Concept 3: Services and Request-Response Communication

### What is a Service?

A **service** is a synchronous remote procedure call (RPC):
- A client sends a request
- A server processes it and sends a response
- The client **waits** for the response (blocking)
- One-to-one communication

```
Client: "Please set color to red"
  ↓
Server: [processes request]
  ↓
Server: "Color set successfully"
  ↓
Client: [continues execution]
```

### Why Services Are Synchronous

Unlike pub/sub, services use a request-response pattern:

```
Time →

Client:  call_service() [blocks here]                  [continues]
           ↓
Server:             [receives request]  [processes]  [sends response]
                                                        ↓
```

The client waits; the server responds. This is useful for:
- Commands that require confirmation ("Did the LED turn on?")
- Queries that need a specific answer ("What's the current joint angle?")
- Configuration changes ("Set maximum speed to 2 m/s")

---

## Concept 4: Messages and Type Safety

### What is a Message?

A **message** is a data structure with strongly-typed fields:

```
message Twist:
  geometry:
    Vector3 linear:
      float64 x      # m/s
      float64 y      # m/s
      float64 z      # m/s
    Vector3 angular:
      float64 x      # rad/s
      float64 y      # rad/s
      float64 z      # rad/s
```

### Why Type Safety Matters

ROS 2 enforces type compatibility:

```
Publisher publishes: geometry_msgs/Twist
Subscriber expects: geometry_msgs/Twist
Result: ✓ Connection succeeds, subscriber receives data

Publisher publishes: geometry_msgs/Twist
Subscriber expects: sensor_msgs/Image
Result: ✗ ROS 2 rejects connection (type mismatch)
```

This prevents subtle bugs where a subscriber mistakenly interprets velocity data as an image.

### Common Message Types in ROS 2

| Message | Fields | Use Case |
|---------|--------|----------|
| `std_msgs/Float64` | `data: float` | Single numeric value |
| `std_msgs/String` | `data: string` | Text messages |
| `geometry_msgs/Twist` | `linear: Vector3`, `angular: Vector3` | Velocity commands (Lesson 4) |
| `geometry_msgs/Pose` | `position: Point`, `orientation: Quaternion` | Robot position and orientation |
| `sensor_msgs/Image` | `height`, `width`, `encoding`, `data` | Camera image (Chapter 2) |
| `sensor_msgs/LaserScan` | `angle_min`, `angle_max`, `ranges` | 2D lidar data (Chapter 2) |

---

## Concept 5: ROS 2 Naming Conventions

### Node Names

Node names identify unique processes. Follow these rules:

```
valid_node_name       ✓ Good (lowercase, underscores)
camera_driver_v2      ✓ Good (descriptive)
CameraDriver          ✗ Bad (use snake_case, not camelCase)
camera_driver_v2!     ✗ Bad (no special characters)
```

**Convention**: `<subsystem>_<function>_<version>` or `<subsystem>_<function>`

```
camera_driver          # Reads from camera
motion_planner         # Plans trajectories
motor_controller       # Commands motors
odom_publisher         # Publishes odometry
```

### Topic Names

Topics follow a hierarchical namespace:

```
/robot_name/subsystem/sensor/data_type

/turtlebot/camera/rgb/image_raw          ✓ Good (hierarchical)
/turtlebot/motor/left_wheel/angular_pos  ✓ Good (specific joint)
/image_raw                                ✗ Bad (not hierarchical)
/camera.image                             ✗ Bad (use /, not dots)
```

**Benefits of hierarchies**:
- Easy to group related topics: `ros2 topic list | grep /turtlebot/camera`
- Clear ownership: Which subsystem owns this topic?
- Namespacing: Run multiple robots (`/robot1/*`, `/robot2/*`) without conflicts

### Service Names

Services follow a similar convention:

```
/robot/subsystem/service_name

/turtlebot/camera/set_resolution         ✓ Good
/turtlebot/motor/emergency_stop          ✓ Good
/set_color                                ✗ Bad (not namespaced)
```

---

## Comparing Pub/Sub vs. Services

When should you use each pattern?

| Criterion | Pub/Sub | Service |
|-----------|---------|---------|
| **Timing** | Asynchronous (fire and forget) | Synchronous (wait for response) |
| **Flow** | One-way (publisher → subscribers) | Two-way (client ↔ server) |
| **Participants** | Many-to-many | One-to-one |
| **Blocking** | Non-blocking | Blocking (client waits) |
| **Use Case** | Streaming data (sensors) | Commands needing confirmation |
| **Example** | Image stream, velocity | Set LED color, query joint angle |
| **Latency** | Higher (data may queue) | Lower (immediate response) |

### Decision Tree

```
Does the sender need a response?
  ├─ Yes → Use Service
  │   └─ Example: "Is the gripper open?"
  │
  └─ No → Use Pub/Sub
      ├─ Streaming data? → Pub/Sub
      │   └─ Example: Camera images, sensor readings
      │
      └─ Periodic command? → Could be either
          ├─ If stateful (e.g., "turn on") → Service
          └─ If continuous (e.g., velocity) → Pub/Sub
```

---

## Observing ROS 2 Communication in Turtlesim

Let's use ROS 2 CLI tools to observe real communication in the running Turtlesim system.

### Setup: Launch Turtlesim Again

If Turtlesim isn't running, open three terminals:

**Terminal 1**:
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2**:
```bash
ros2 run turtlesim turtle_teleop_key
```

**Terminal 3** (for our experiments):
```bash
# We'll run commands here
```

### Exploring Nodes with `ros2 node`

List all nodes:
```bash
ros2 node list
```

**Output**:
```
/turtlesim
/teleop_turtle
```

Get info about a specific node:
```bash
ros2 node info /turtlesim
```

**Output**:
```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: std_msgs/msg/ColorRGBA
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter: rcl_interfaces/srv/GetParameter
    /turtlesim/set_parameter: rcl_interfaces/srv/SetParameter
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:
    /turtle1/set_pen: turtlesim/srv/SetPen
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

**Interpretation**:
- `/turtlesim` subscribes to `/turtle1/cmd_vel` (movement commands from teleop)
- `/turtlesim` publishes `/turtle1/pose` (turtle's current position) and `/turtle1/color_sensor` (color under turtle)
- `/turtlesim` offers services like `/turtle1/set_pen` (change pen color/width)

### Exploring Topics with `ros2 topic`

List all active topics:
```bash
ros2 topic list
```

**Output**:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Get info about a topic:
```bash
ros2 topic info /turtle1/cmd_vel
```

**Output**:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

**Meaning**: One publisher (`teleop_turtle`) sends velocity commands, and one subscriber (`turtlesim`) receives them.

### Observing Message Data with `ros2 topic echo`

Watch velocity commands as you control the turtle:

**Terminal 3**:
```bash
ros2 topic echo /turtle1/cmd_vel
```

**Then in Terminal 2**, press 'w' to move forward.

**Output in Terminal 3**:
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

Each `---` separator marks a new message. You see repeated messages because the teleop node publishes at a fixed rate (~10 Hz) while a key is held down.

### Measuring Topic Frequency with `ros2 topic hz`

How often are messages published?

```bash
ros2 topic hz /turtle1/cmd_vel
```

**Output** (while holding 'w'):
```
average rate: 10.01
  min: 90.088ms  max: 110.234ms  std dev: 8.343ms  window: 10
```

The teleop node publishes ~10 times per second (10 Hz or 100ms between messages).

### Inspecting Message Types with `ros2 interface show`

What are the exact fields in a Twist message?

```bash
ros2 interface show geometry_msgs/msg/Twist
```

**Output**:
```
geometry_msgs/Vector3 linear
	float64 x
	float64 y
	float64 z
geometry_msgs/Vector3 angular
	float64 x
	float64 y
	float64 z
```

This defines the structure that every Twist message must follow.

---

## Concept 6: The Message Graph

A **message graph** visualizes all nodes and topics in a system. ROS 2 includes a tool for this:

```bash
ros2 run rqt_graph rqt_graph
```

A window appears showing:

```
[teleop_turtle] ──/turtle1/cmd_vel──→ [turtlesim]
                                           ↓
                                    /turtle1/pose
                                           ↓
                                    (subscribers)
```

This graph shows:
- **Nodes** as boxes (`teleop_turtle`, `turtlesim`)
- **Topics** as labeled arrows (`/turtle1/cmd_vel`, `/turtle1/pose`)
- **Direction** of data flow (→ means publisher to subscriber)

This visual representation is invaluable for understanding complex robot systems.

---

## Concept 7: Parameter Servers and Dynamic Configuration

ROS 2 includes a **parameter server** for runtime configuration:

List all parameters:
```bash
ros2 param list
```

**Output**:
```
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```

Get a parameter value:
```bash
ros2 param get /turtlesim background_b
```

**Output**:
```
Integer value is: 255
```

Set a parameter:
```bash
ros2 param set /turtlesim background_b 100
```

The background color should change in the Turtlesim window!

Parameters allow runtime configuration without recompiling code. In Lesson 4, you'll learn to create your own parameters.

---

## Self-Assessment Checklist

Before moving to Lesson 4, verify:

- [ ] I understand the four stages of node lifecycle (created, initialized, running, shutdown)
- [ ] I can explain why pub/sub is asynchronous and services are synchronous
- [ ] I can use `ros2 node list` and `ros2 node info` to inspect running nodes
- [ ] I can use `ros2 topic list` and `ros2 topic echo` to observe message data
- [ ] I understand that topics enforce message type safety
- [ ] I can apply ROS 2 naming conventions (nodes, topics, services)
- [ ] I recognize common message types (Twist, Pose, Image)
- [ ] I can explain when to use pub/sub vs. services
- [ ] I understand that ROS 2 systems are loosely coupled (nodes don't know about each other)

If any checks failed, re-read the corresponding concept section.

---

## Common Misconceptions Clarified

### Misconception 1: "A topic is like a shared variable"

**Wrong**: Topics are not variables; they're communication channels. A topic doesn't "store" the latest message; it delivers messages to subscribers.

**Correct**: Topics are like a news broadcast. The latest news is gone after it airs; it doesn't persist. (Technically, ROS 2 can buffer recent messages with QoS settings, but this is advanced.)

### Misconception 2: "If I don't subscribe to a topic, it won't publish"

**Wrong**: Publishers send messages regardless of subscribers. If no one is listening, messages are just discarded.

**Correct**: Like a radio station—it broadcasts whether anyone is listening or not.

### Misconception 3: "Services are better than pub/sub for everything"

**Wrong**: Services block the client, which is inefficient for streaming data. If you use services for camera images at 30 Hz, clients will be constantly blocked.

**Correct**: Use services only when you need a response. Use pub/sub for continuous data.

### Misconception 4: "All nodes must have the same ROS_DOMAIN_ID"

**Wrong**: Nodes with different domain IDs (e.g., 0 and 1) are completely isolated. They won't communicate.

**Correct**: Nodes with the **same** domain ID communicate. Different domain IDs let you run multiple independent ROS 2 systems on the same network.

---

## Troubleshooting

### "ros2 topic echo" shows no messages

**Cause**: Topic exists but no one is publishing to it.

**Solution**:
1. Run `ros2 topic list` to see all topics
2. Check which node should be publishing
3. Verify that node is running: `ros2 node list`
4. Check if the publisher is active with `ros2 topic info /topic_name`

### "Connection refused" when using ros2 commands

**Cause**: ROS 2 daemon not running or environment not sourced.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 daemon start
```

### rqt_graph doesn't appear

**Cause**: rqt not installed.

**Solution**:
```bash
sudo apt install -y ros-humble-rqt ros-humble-rqt-graph
```

### Nodes in different domain IDs can't communicate

**Cause**: Domain IDs don't match.

**Verification**:
```bash
echo $ROS_DOMAIN_ID  # Check your domain ID
```

**Solution**: Set the same domain ID before running all nodes:
```bash
export ROS_DOMAIN_ID=0
```

---

## Further Exploration (Optional)

1. **ROS 2 Concepts**: https://docs.ros.org/en/humble/Concepts/Intermediate-Concepts.html
2. **Message Types Reference**: https://docs.ros.org/en/humble/Concepts/Intermediate-Concepts/About-Builtin-Interfaces.html
3. **rqt Tools Documentation**: https://wiki.ros.org/rqt
4. **Quality of Service (QoS)**: https://docs.ros.org/en/humble/Concepts/Intermediate-Concepts/About-Quality-of-Service-Settings.html (advanced topic)

---

## What's Next?

In **Lesson 4: Your First ROS 2 Publisher**, you'll:
- Create a Python node from scratch
- Implement a publisher that sends velocity commands
- Observe the turtle move in response to your code
- Debug using the CLI tools from this lesson

**Estimated time**: 60 minutes

**Prerequisites for Lesson 4**: Understand nodes, topics, and pub/sub patterns from this lesson

---

## Layer 2: AI Collaboration Prompts

### For Understanding Node Lifecycle

> "I learned about node initialization, running, and shutdown. Ask Claude: 'What happens if I call `rclpy.spin()` before creating any publishers or subscribers? What's the point of spinning an empty node?'"

### For Understanding Communication Patterns

> "I'm confused about when to use pub/sub vs. services. Ask Claude: 'Give me 5 real-world examples where pub/sub is better, and 5 where services are better. What's the key difference?'"

### For Understanding Type Safety

> "Ask Claude: 'Why does ROS 2 enforce message types? What would happen if a publisher sent random data without a defined structure?'"

---

## Concept Count & CEFR Validation

**Total A2 Concepts Introduced**: 7

1. Node definition and lifecycle (created, initialized, running, shutdown)
2. Publish-subscribe pattern (asynchronous, many-to-many)
3. Service pattern (synchronous, request-response)
4. Message types and strong typing
5. ROS 2 naming conventions (nodes, topics, services)
6. Topic hierarchies and namespacing
7. CLI tools for system inspection (ros2 node, ros2 topic, rqt_graph)

**CEFR A2 Alignment**: ✓ Foundational architectural concepts, clear definitions, concrete examples with Turtlesim

---

**Chapter 1, Lesson 3 Complete**

Next: Lesson 4 — Your First ROS 2 Publisher (60 min)
