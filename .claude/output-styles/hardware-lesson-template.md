---
name: hardware-lesson-template
description: Docusaurus lesson template for Physical AI & Robotics with simulation metadata, CEFR proficiency levels, and safety-first 4-Layer method. Includes YAML frontmatter for automated chapter generation and RAG chatbot integration.
---

# Hardware Lesson Template

Use this template for all Physical AI & Robotics lessons. This template enforces simulation-first teaching, safety-progressive autonomy, and the 4-Layer pedagogical method.

---

## Template File Structure

**File location**: `book-source/docs/[chapter-slug]/[lesson-slug].md`

**Filename convention**: Lowercase, hyphen-separated
- ✅ Good: `02-understanding-ros2-nodes.md`
- ✅ Good: `03-gazebo-urdf-basics.md`
- ❌ Avoid: `Lesson 2 - Understanding ROS2 Nodes.md`

---

## YAML Frontmatter

Place this at the very start of the lesson file:

```yaml
---
# Docusaurus Metadata (for sidebar + navigation)
title: "[Lesson Title]"
sidebar_position: 2  # Number within chapter
sidebar_label: "[Short Title for Navigation]"

# Chapter + Lesson Organization
chapter: 1  # Chapter number
lesson: 2   # Lesson number within chapter
lesson_slug: "understanding-ros2-nodes"  # For URLs

# Duration & Complexity
duration_minutes: 45
estimated_difficulty: "beginner"  # beginner, intermediate, advanced

# ROBOTICS-SPECIFIC METADATA (Critical for Book + RAG Chatbot)

# Simulation Environment (MANDATORY)
# Which simulator(s) does this lesson use?
simulation_required:
  - "turtlesim"  # or "gazebo", "isaac_sim", "mujoco"

# Safety Level (Constitutional - IX)
# Controls what we say about hardware progression
safety_level: "simulation_only"  # or "benchtop_testing", "field_deployment"

# CEFR Language Proficiency (Controls cognitive load)
# A1: Complete beginner
# A2: Elementary (max 5-7 concepts per lesson)
# B1: Intermediate (max 7-10 concepts)
# B2: Upper intermediate
# C1: Advanced
# C2: Mastery (no limits)
cefr_level: "A2"

# Hardware Prerequisites (for chapter sequencing)
# What simulated hardware does this lesson assume?
# If lesson requires student to know "sensor integration", list it
hardware_prerequisites: []
# Example: ["realsense_camera_knowledge", "lidar_integration"]

# Conceptual Concepts (for validation)
# List all unique concepts taught
concepts:
  - "ROS 2 nodes"
  - "publishers"
  - "subscribers"
  - "messages"
  - "topics"
  - "rclpy library"

# Pedagogical Layer (for output-style selection + validation)
# Which layer(s) does this lesson implement?
primary_layer: "Layer 1 and Layer 2"
# Layers: "Layer 1" (Foundation), "Layer 2" (Collaboration), "Layer 3" (Intelligence), "Layer 4" (Orchestration)

# Learning Objectives (Measurable, SMART format)
learning_objectives:
  - "Understand the difference between ROS 2 nodes and topics"
  - "Create a minimal ROS 2 publisher node using rclpy"
  - "Verify message flow using ros2 topic echo"
  - "Debug common publisher connection issues"

# Assessment Type (for validation pipeline)
assessment_type: "practical_exercise"  # or "quiz", "project", "discussion"

# Keywords for RAG Chatbot
keywords:
  - "ROS 2"
  - "nodes"
  - "publishers"
  - "subscribers"
  - "topics"
  - "messages"
  - "Turtlesim"

---
```

---

## Lesson Markdown Content

Use this structure for lesson content:

### H1: Lesson Title (Only One H1 per lesson)

```markdown
# Understanding ROS 2 Nodes and Topics
```

### Introduction Section (2-3 paragraphs)

**Purpose**: Hook reader, establish context, explain why this matters

```markdown
## Introduction

In the previous lesson, we set up ROS 2 and verified installation. Now we'll understand the fundamental building blocks of robotics software: **nodes** (independent processes) and **topics** (communication channels).

Why does this matter? Every robot system—from simple Turtlesim to complex humanoids—uses nodes and topics to coordinate behavior. Understanding these concepts first will make everything else (sensors, actuators, navigation) much clearer.

In this lesson, you'll learn how nodes publish data (like sensor readings) and how other nodes subscribe to that data. We'll build this understanding through simulation first, using Turtlesim.
```

### Prerequisites Section

**Purpose**: Clarify what students need before starting

```markdown
## Prerequisites

**Prior Knowledge**:
- How to open a terminal and run commands
- ROS 2 installation verified (from Chapter 1, Lesson 1)
- Understanding of publisher-subscriber pattern (if from previous lesson, mention it)

**Simulation Environment**:
- **Simulator**: Turtlesim
- **Setup**: `ros2 run turtlesim turtlesim_node`
- **Estimated Setup Time**: 2 minutes

**Learning Outcomes**:
By the end of this lesson, you'll be able to:
- Explain the roles of nodes and topics in ROS 2
- Create a ROS 2 publisher node in Python
- Verify message flow using command-line tools
```

### Layer 1: Manual Foundation (Build Physical Intuition)

**Purpose**: Build mental models BEFORE code. Students understand "what" before "how".

```markdown
## Layer 1: Understanding Nodes and Topics Manually

### Concept: What is a ROS 2 Node?

A **node** is an independent process in ROS 2. Think of it like a person in a team:
- Each person (node) has specific responsibilities
- People (nodes) communicate by talking (messages)
- Multiple people (nodes) can work simultaneously

In robotics, examples of nodes:
- A "sensor reader" node: reads camera/lidar data
- A "controller" node: decides how to move
- A "decision maker" node: plans high-level actions

### Concept: What is a Topic?

A **topic** is a communication channel between nodes. Think of it like a bulletin board:
- One node writes a message to the board (publishes)
- Other nodes read from the board (subscribe)
- One bulletin board (topic) can have many readers and writers

Example robot topics:
- `/camera/image`: Camera sends image data
- `/cmd_vel`: Controller sends motion commands
- `/battery/status`: Battery sends voltage updates

### Manual Exercise: Understanding Communication

Before we write code, let's understand node communication conceptually.

**Exercise**: Imagine a robot with three nodes:
1. **Camera Node**: Reads images, publishes to `/camera/image`
2. **Processor Node**: Reads from `/camera/image`, analyzes, publishes to `/detection/objects`
3. **Mover Node**: Reads from `/detection/objects`, commands movement

**Draw this on paper** (or in image):
- Draw three boxes (nodes)
- Draw arrows between them (topics)
- Label which is publisher, which is subscriber

**What to observe**:
- How does data flow? (Camera → Processor → Mover)
- What if Processor was slow? (Does Camera still publish? Yes.)
- What if Mover wants more information? (Can it also read from `/camera/image`? Yes.)

This understanding is KEY before coding.
```

### Layer 2: AI Collaboration (Three Roles Framework)

**Purpose**: Students learn to work WITH AI. Three Roles (Teacher, Student, Co-Worker) emerge naturally WITHOUT exposing the framework.

```markdown
## Layer 2: Creating Your First ROS 2 Node (With AI Partnership)

### Setting Up Turtlesim

First, let's start the simulator:

```bash
# Terminal 1: Launch Turtlesim
ros2 run turtlesim turtlesim_node
```

You should see a window with a small turtle in the center.

### Creating a Publisher Node

Now we'll create a ROS 2 publisher—a node that sends messages to a topic.

```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleCommandPublisher(Node):
    def __init__(self):
        super().__init__('turtle_command_publisher')
        # Create a publisher that sends Twist (motion) messages
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create a timer that calls our function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Create a Twist message (velocity command)
        msg = Twist()
        msg.linear.x = 1.0  # Move forward 1 m/s
        msg.angular.z = 0.0  # Don't rotate

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}')

        self.counter += 1
        if self.counter >= 5:  # Stop after 2.5 seconds
            self.publisher_.publish(Twist())  # Send stop command
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommandPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [turtle_command_publisher]: Publishing: linear.x=1.0
[INFO] [turtle_command_publisher]: Publishing: linear.x=1.0
[INFO] [turtle_command_publisher]: Publishing: linear.x=1.0
[INFO] [turtle_command_publisher]: Publishing: linear.x=1.0
[INFO] [turtle_command_publisher]: Publishing: linear.x=1.0
```

**What to observe**: The turtle should move forward for ~2.5 seconds, then stop.

### 💬 AI CoLearning Prompt

Ask Claude AI: **"I created a ROS 2 publisher, but it's not moving the turtle. What are common reasons why messages might not be received?"**

Expected answers from AI:
- Node not running
- Topic name mismatch (`/turtle1/cmd_vel` vs other names)
- Message type mismatch (Twist vs other types)
- Node not spinning correctly

This exercise teaches you to:
1. Ask AI for patterns you don't know
2. Verify AI answers by testing
3. Think about the system holistically

### 🤝 Collaboration Exercise: Debug with AI

"Your ROS 2 node compiles and runs, but the turtle doesn't move. Work with Claude to:
1. List all possible reasons
2. Create diagnostic commands
3. Fix the issue"

Diagnostic commands to try:
```bash
# Check if your node is running
ros2 node list

# Check what topics exist
ros2 topic list

# Check if messages are being published
ros2 topic echo /turtle1/cmd_vel
```

**Work with AI**: "My `ros2 topic echo` doesn't show messages. What does that tell us?"
```

### Troubleshooting Section (Mandatory)

```markdown
## Troubleshooting

### Error: "Could not import geometry_msgs.msg"

**Reason**: geometry_msgs package not installed

**Solution**:
```bash
sudo apt install ros-humble-geometry-msgs
```

### Error: "node not found: /turtle1/cmd_vel"

**Reason**: Turtlesim node not running

**Solution**:
Make sure Terminal 1 is running: `ros2 run turtlesim turtlesim_node`

### Error: "rclpy.init() called twice"

**Reason**: You already ran `rclpy.init()` in another terminal

**Solution**:
Make sure you're using different terminals for different nodes

### Turtle doesn't move when I publish

**Diagnosis steps**:
1. Is Turtlesim still running? (Check Terminal 1)
2. Are messages being published?
   ```bash
   ros2 topic echo /turtle1/cmd_vel
   ```
3. Is the topic name correct? (Check: `/turtle1` not `/turtle`)
4. Is message format correct? (Use Twist for velocity)
```

### Self-Assessment Checklist

```markdown
## Self-Assessment

Before moving to the next lesson, verify you can:

- [ ] Explain the difference between a node and a topic
- [ ] Create a ROS 2 node that publishes messages
- [ ] Verify messages are being published using `ros2 topic echo`
- [ ] Debug common ROS 2 connection issues
- [ ] Explain why publisher-subscriber pattern is useful
```

### Next Steps

```markdown
## What's Next?

In the next lesson, we'll create a **subscriber** node—a node that READS messages from topics instead of publishing them.

You'll learn:
- How to subscribe to a topic
- How to process incoming messages
- How publishers and subscribers work together

**Preview**: Soon you'll have:
- A publisher node (sends motion commands)
- A subscriber node (reads sensor data)
- These nodes communicating through ROS 2 topics

This is the foundation of all robotics software!

[→ Next Lesson: Creating Subscriber Nodes](03-creating-subscriber-nodes.md)
```

### Additional Resources

```markdown
## Resources

- [Official ROS 2 Documentation: Creating a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Topic Command Cheat Sheet](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Geometry Msgs: Twist Documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Simulation/Turtlesim/Turtlesim.html)
```

---

## Template Validation Checklist

Before publishing lesson, verify ALL items:

- [ ] **Frontmatter complete**: All YAML fields populated
- [ ] **Simulation specified**: Every code block specifies simulator (Turtlesim/Gazebo/Isaac)
- [ ] **Safety-first**: No hardware deployment suggested for A2/B1
- [ ] **Three Roles present**: At least one CoLearning prompt (💬) per lesson
- [ ] **Code tested**: Every example includes "Expected Output"
- [ ] **Troubleshooting complete**: Common errors documented with solutions
- [ ] **Framework invisible**: Students experience pedagogy, don't see "Layer" labels
- [ ] **CEFR respected**: Concept count within tier limits (A2: 5-7, B1: 7-10)
- [ ] **No forward references**: No mentions of future concepts
- [ ] **Self-assessment clear**: Checkbox list of verifiable skills
- [ ] **Next lesson linked**: Clear path to next lesson
- [ ] **Markdown correct**: Proper heading hierarchy, code blocks, links
- [ ] **RAG metadata**: Keywords present for chatbot indexing
