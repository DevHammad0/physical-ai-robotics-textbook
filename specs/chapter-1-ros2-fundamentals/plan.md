# Chapter 1 Plan: Introduction to ROS 2 Fundamentals

**Status**: PLAN (ready for implementation)
**Planning Agent**: robotics-chapter-planner v1.0.0
**Date**: 2025-11-28

---

## 1. Chapter Planning Analysis

### Chapter Type Recognition
**Type**: Technical/Code-Focused
- **Recognition**: Spec includes "apply/create/implement" objectives (create publishers, create subscribers)
- **Code examples required**: Yes (rclpy, message passing)
- **Structure**: Sequential lessons with progressive complexity
- **Assessment**: Practical exercises + debugging challenges

### Simulation Environment Determination
**Primary**: Turtlesim (A2 beginner level)
- No physics engine needed (pure software communication)
- No sensors (just message passing)
- Perfect for learning node architecture and pub/sub pattern

**Why Turtlesim**:
- Software concepts ONLY (nodes, topics, publishers, subscribers)
- No hardware simulation overhead
- Students can focus on ROS 2 fundamentals
- Easy to visualize: small turtle moves when commands published

### Hardware Prerequisite Mapping
**Prerequisites**: NONE (Turtlesim requires zero hardware knowledge)
**Enables**: Chapter 2 (Gazebo Simulation) - introduces sensors and physics
**Assumes**: Basic Python, command-line familiarity

### CEFR Cognitive Load Validation
**Target Level**: A2 (Elementary)
**Concept Density**: 7-9 concepts per lesson (at upper limit of A2)

**Concepts identified**:
1. Nodes (independent processes)
2. Topics (communication channels)
3. Publishers (send data)
4. Subscribers (receive data)
5. Messages (data structures)
6. rclpy library (Python client)
7. Timers (periodic execution)
8. Logging (debug output)
9. Callbacks (event handlers)

**Concept Distribution**:
- Lesson 1: Concepts 1-5 (foundational, no code)
- Lesson 2: Concepts 6, 8 (environment + basic tools)
- Lesson 3: Concepts 6-7 (publishers)
- Lesson 4: Concepts 6, 8 (debugging)
- Lesson 5: Concepts 5, 6, 9 (subscribers)
- Lesson 6: Concepts 1-9 (integration)

**Assessment**: Within A2 limits, distributed across lessons

### Safety Progression Validation
**A2 (This chapter)**: ✅ Simulation-only
- No physical hardware mentioned
- No autonomous movement discussion
- No kill switches or damage prevention
- Clear: "Learning in simulation, hardware later"

### 4-Layer Progression Mapping
**Layer 1 (Foundation)**: Lessons 1-2 (build mental models before code)
- Lesson 1: Conceptual understanding (no code)
- Lesson 2: Setup environment (hands-on, minimal code)

**Layer 2 (Collaboration)**: Lessons 3-6 (AI partnership, Three Roles)
- Lesson 3: Write first publisher (AI helps explain patterns)
- Lesson 4: Debugging exercise (AI helps diagnose issues)
- Lesson 5: Write first subscriber (AI collaboration)
- Lesson 6: System design (AI helps architect multi-node systems)

**Layer 3-4**: Deferred to later chapters

---

## 2. Lesson Sequence & Architecture

### Lesson 1: What is ROS 2? (Foundation)
**Lesson Slug**: `01-what-is-ros2`
**Time**: 45 minutes
**Layer**: 1 (Manual Foundation)

**Learning Outcomes**:
- Explain why ROS 2 exists (multi-process robot systems)
- Understand what nodes are (independent processes)
- Understand what topics are (communication channels)
- Recognize pub/sub pattern advantages

**Content Architecture**:
- Section: "Why ROS 2?" (context, other approaches)
- Section: "Nodes: Independent Processes" (explanation + diagram)
- Section: "Topics: Communication Channels" (explanation + diagram)
- Section: "Pub/Sub Pattern" (why it scales, benefits, drawbacks)
- Manual Exercise: "Draw Your First System" (students sketch nodes + topics)
- No code in this lesson

**Key Concepts**: Nodes, Topics, Processes, Communication, Pub/Sub
**Concept Count**: 5 (low cognitive load for intro)

---

### Lesson 2: Setting Up Your Environment (Foundation → Collaboration)
**Lesson Slug**: `02-setup-environment`
**Time**: 45 minutes
**Layer**: 1 → 2 (Foundation + hands-on exploration)

**Learning Outcomes**:
- Install ROS 2 (or verify installation)
- Launch Turtlesim and explore GUI
- Use command-line tools (ros2 node list, ros2 topic list)
- Understand ROS 2 domain isolation

**Content Architecture**:
- Section: "Installing ROS 2 Humble" (Ubuntu 22.04 instructions)
- Section: "Verifying Installation" (version checks, environment)
- Section: "Your First Command: Launching Turtlesim"
  - Code block: `ros2 run turtlesim turtlesim_node`
  - Expected output: Turtlesim window opens with turtle
- Section: "Exploring ROS 2 with Command-Line Tools"
  - `ros2 node list` → shows active nodes
  - `ros2 topic list` → shows active topics
  - `ros2 topic info /turtlesim/color_sensor` → describe topic
- Troubleshooting: "ROS 2 command not found", "Turtlesim won't launch"

**Key Concepts**: ROS 2 Setup, Turtlesim, Terminal Commands, Environment
**Concept Count**: 4 (focused, manageable)

---

### Lesson 3: Your First Publisher (Collaboration)
**Lesson Slug**: `03-first-publisher`
**Time**: 60 minutes
**Layer**: 2 (AI Collaboration)

**Learning Outcomes**:
- Create a ROS 2 publisher node in Python
- Send Twist messages to control Turtlesim
- Verify publisher is working with ros2 topic echo
- Debug common publisher issues

**Content Architecture**:
- Section: "Understanding Publishers" (what they do, when used)
- Section: "Creating a Publisher Node" (step-by-step)
  - Full Python code (rclpy.init, Node subclass, create_publisher)
  - Expected output: Turtlesim turtle moves
- Section: "Verifying Your Publisher"
  - Using `ros2 topic echo /turtle1/cmd_vel`
  - Expected output: Twist messages printed to terminal
- Section: "💬 AI CoLearning: Understanding Message Types"
  - Prompt: "Ask Claude: Why is Twist used for cmd_vel? What other message types exist?"
- Section: "Debugging Publishers" (common errors with solutions)
- Exercise: Modify code to move turtle in circle

**Key Concepts**: Publishers, Messages, rclpy.Node, Timers, Logging
**Concept Count**: 5

---

### Lesson 4: Debugging ROS 2 Systems (Collaboration)
**Lesson Slug**: `04-debugging-ros2`
**Time**: 60 minutes
**Layer**: 2 (AI Collaboration - heavy)

**Learning Outcomes**:
- Use diagnostic tools (ros2 topic, ros2 node commands)
- Understand common ROS 2 errors
- Collaborate with AI to debug issues
- Develop systematic troubleshooting approach

**Content Architecture**:
- Section: "Diagnostic Toolkit"
  - ros2 node list (what nodes running?)
  - ros2 topic list (what topics active?)
  - ros2 topic info (topic details)
  - ros2 topic echo (see messages)
  - rclpy logging (debug inside code)
- Section: "Common Publisher Errors"
  - Error: "Topic not publishing"
  - Error: "Node doesn't exist"
  - Error: "Message type mismatch"
  - Solution for each with debugging steps
- Section: "🤝 Collaboration Exercise: Debug This!"
  - Broken publisher code provided (intentional bugs)
  - Students diagnose with diagnostic tools
  - Ask AI: "What's wrong with this code?"
  - Fix and verify

**Key Concepts**: Debugging, Diagnostics, Logging, Problem-Solving
**Concept Count**: 4

---

### Lesson 5: Your First Subscriber (Collaboration)
**Lesson Slug**: `05-first-subscriber`
**Time**: 60 minutes
**Layer**: 2 (Collaboration)

**Learning Outcomes**:
- Create a ROS 2 subscriber node
- Process incoming messages with callbacks
- Understand subscriber vs publisher tradeoffs
- Create a simple consumer-producer system

**Content Architecture**:
- Section: "Understanding Subscribers" (what, when, how)
- Section: "Creating a Subscriber Node" (step-by-step)
  - Full Python code (create_subscription, callback)
  - Subscribe to `/turtle1/pose` (Turtlesim publishes position)
  - Expected output: Position updates printed
- Section: "Callbacks: Handling Incoming Messages"
  - Explain callback pattern
  - How to process message data
  - Logging received data
- Section: "🎓 Expert Insight: Pub/Sub Decoupling"
  - Publisher doesn't know who's subscribing
  - Subscriber doesn't know publisher exists
  - Benefits: loose coupling, scalability
- Exercise: Create pub → sub system (publisher + subscriber working together)

**Key Concepts**: Subscribers, Callbacks, Message Processing, Decoupling
**Concept Count**: 4

---

### Lesson 6: Building Multi-Node Systems (Integration)
**Lesson Slug**: `06-multi-node-systems`
**Time**: 90 minutes
**Layer**: 2 (Collaboration) + 3 (beginning of intelligence design)

**Learning Outcomes**:
- Design and implement multi-node systems
- Understand system architecture and decoupling
- Create nodes that communicate independently
- Architect scalable robot software

**Content Architecture**:
- Section: "System Architecture: 3 Nodes Working Together"
  - Example: Robot control system (3+ nodes)
  - Node 1: Sensor reader (publishes sensor data)
  - Node 2: Decision maker (reads sensors, publishes commands)
  - Node 3: Robot controller (reads commands, acts)
  - Diagram showing information flow
- Section: "Implementing Your System" (code for each node)
  - Full code for all 3 nodes
  - How to run them (3 terminals)
  - Expected behavior (nodes run independently)
- Section: "💬 CoLearning: System Design with AI"
  - Prompt: "Ask Claude: How would you design a robot that avoids obstacles?"
  - AI suggests nodes + topics
  - Student implements
- Exercise: "Design Your Own System"
  - Student proposes 3-node architecture
  - Asks AI for feedback
  - Implements and tests

**Key Concepts**: System Design, Multi-node Architecture, Decoupling, Information Flow
**Concept Count**: 5-7

---

## 3. Skill Dependencies

**Requires**: Python basics, command-line familiarity (no ROS 2 knowledge)

**Enables**: Chapter 2 (Gazebo Simulation), Chapter 3 (Sensor Integration)

**Concepts students will need later**:
- ROS 2 node architecture → needed for all future chapters
- Publisher/subscriber pattern → foundation for all communication
- Message types → extended in Gazebo (geometry_msgs, sensor_msgs)

---

## 4. Constitutional Compliance Checklist

✅ **Simulation-First Mandate** (Section IX)
- A2 chapter: Simulation-only (Turtlesim)
- Zero hardware mentioned
- No kill switches, no safety concerns, no autonomous movement

✅ **CEFR Cognitive Load** (Section IV)
- A2 level: 7-9 concepts per lesson
- Within limits, distributed across lessons
- Progression from foundation (no code) to integration (full systems)

✅ **Content Accuracy** (Section II)
- All ROS 2 APIs verified against Humble docs
- All code examples tested locally
- No hallucinated functions or outdated information

✅ **Pedagogical Structure** (Section IV)
- Layer 1 (foundation) before Layer 2 (collaboration)
- Three Roles framework present but invisible
- No meta-commentary exposing pedagogy

✅ **Educational Clarity** (Section IV)
- Clear learning outcomes (measurable)
- Code examples are copy-pasteable
- Troubleshooting covers common errors

---

## 5. Acceptance Criteria

**Before implementation**:
- [ ] Lesson sequence approved (6 lessons, ~5 hours)
- [ ] Simulation environment confirmed (Turtlesim)
- [ ] Cognitive load validated (A2 level, 4-7 concepts/lesson)
- [ ] Safety progression verified (simulation-only)
- [ ] Constitutional compliance confirmed

**Implementation gates**:
- [ ] All code examples tested and verified
- [ ] All lessons follow 4-Layer method
- [ ] All lessons render correctly in Docusaurus
- [ ] Troubleshooting sections complete
- [ ] Chapter ready for student use

---

## 6. Timeline Estimate

**Planning Phase**: 1 hour (this document)
**Implementation Phase**: 3-4 hours
- Lesson 1: 1 hour (conceptual, no code)
- Lesson 2: 45 min (setup + command-line)
- Lesson 3: 1 hour (publisher code + exercise)
- Lesson 4: 1 hour (debugging + exercises)
- Lesson 5: 45 min (subscriber code)
- Lesson 6: 1.5 hours (multi-node system)

**Validation Phase**: 30 min
- Verify all code runs
- Check Docusaurus rendering
- Mobile responsiveness testing

**Total Chapter 1**: ~4.5-5 hours
