# Chapter 1 Specification: Introduction to ROS 2 Fundamentals

**Chapter Number**: 1
**Status**: SPECIFICATION (ready for planning)
**Date**: 2025-11-28

---

## 1. Chapter Overview

**Title**: Introduction to ROS 2 Fundamentals

**Target Audience**: Students with programming background (Python), no prior ROS 2 experience

**CEFR Level**: A2 (Elementary) - Beginner-friendly with max 7-10 concepts

**Duration**: 3-4 hours total (5-6 lessons @ 45-60 min each)

**Simulation Environment**: Turtlesim (no physics, pure software concepts)

**Hardware Tier**: A2 (Simulation-only, zero physical hardware required)

---

## 2. Chapter Purpose & Learning Outcomes

### Goal
Students understand the foundational architecture of ROS 2: how nodes communicate via topics and services, and can create a basic publisher/subscriber system.

### Learning Outcomes

By the end of Chapter 1, students will be able to:

1. **Understand ROS 2 Architecture**
   - Explain what a ROS 2 node is and why it's useful
   - Describe the pub/sub pattern and why it enables scalability
   - Understand message-passing communication vs direct function calls

2. **Create ROS 2 Nodes**
   - Write a Python ROS 2 node using rclpy
   - Create publishers that send messages to topics
   - Create subscribers that receive messages from topics
   - Use timers for periodic execution

3. **Debug ROS 2 Systems**
   - Use command-line tools (ros2 topic, ros2 node, rclpy logging) to debug
   - Identify common ROS 2 errors (node not running, topic mismatch, type mismatch)
   - Troubleshoot publisher/subscriber connection issues

4. **Apply AI Collaboration to Robotics**
   - Use AI to understand ROS 2 patterns
   - Ask AI for help debugging systems
   - Collaborate with AI to solve problems

---

## 3. User Stories & Acceptance Criteria

### User Story 1: Understanding the Why (Layer 1 - Foundation)
**As a** student new to robotics
**I want to** understand why ROS 2 uses nodes and topics
**So that** I can see how it relates to robot systems

**Acceptance Criteria**:
- [ ] Chapter explains nodes as "independent processes"
- [ ] Chapter explains topics as "communication channels"
- [ ] Analogies relate concepts to familiar systems (bulletin boards, team communication)
- [ ] Student can draw a diagram of nodes and topics
- [ ] No code yet - pure conceptual understanding

### User Story 2: Writing First Publisher (Layer 1 → Layer 2)
**As a** developer
**I want to** create a ROS 2 publisher in Python
**So that** I can send data from one node to another

**Acceptance Criteria**:
- [ ] Complete, runnable Python code provided
- [ ] Code uses rclpy library correctly
- [ ] Includes rclpy.init(), Node subclass, create_publisher()
- [ ] Code specifies simulator (Turtlesim) explicitly
- [ ] Expected output shown for verification
- [ ] Troubleshooting section addresses common errors

### User Story 3: Debugging with AI (Layer 2 - Collaboration)
**As a** student
**I want to** work with AI to understand publisher issues
**So that** I can debug similar problems independently

**Acceptance Criteria**:
- [ ] CoLearning prompt encourages asking AI about patterns
- [ ] Student learns to use command-line debugging tools
- [ ] Exercise shows iteration: try → observe → ask AI → adjust
- [ ] Framework stays invisible (no "Three Roles" labels)

### User Story 4: Writing First Subscriber (Layer 1 → Layer 2)
**As a** developer
**I want to** create a ROS 2 subscriber
**So that** I can read data from topics

**Acceptance Criteria**:
- [ ] Complete Python code provided
- [ ] Includes subscription, message callback
- [ ] Code handles incoming messages correctly
- [ ] Troubleshooting section for common subscriber issues

### User Story 5: Building Systems (Layer 2 → Advanced)
**As a** developer
**I want to** create multiple nodes that communicate
**So that** I can build scalable robotics systems

**Acceptance Criteria**:
- [ ] Example shows 2+ nodes working together
- [ ] Demonstrates independent execution (nodes don't wait for each other)
- [ ] Shows topic-based decoupling (nodes don't know each other)

---

## 4. Content Scope

### In Scope (MUST Include)

**Conceptual Foundation**:
- ROS 2 vs ROS 1 (why ROS 2 exists)
- Nodes: independent processes, executables
- Topics: named communication channels
- Messages: standardized data structures
- Publishers: send data to topics
- Subscribers: receive data from topics
- Services: request-reply pattern (intro only)
- Parameters: configuration system (intro only)

**Practical Skills**:
- Setting up ROS 2 environment
- Launching Turtlesim
- Writing publisher in Python (rclpy)
- Writing subscriber in Python (rclpy)
- Using ros2 topic echo for debugging
- Using ros2 node list for diagnostics
- Handling common ROS 2 errors

**Simulation Context**:
- All examples run in Turtlesim
- No physical hardware introduced
- Focus on software communication patterns

**AI Collaboration**:
- At least 3 CoLearning prompts
- Debugging exercises with AI
- Understanding ROS 2 patterns through AI partnership

### Out of Scope (MUST NOT Include)

- Physical robot hardware (simulation-first mandate)
- Gazebo or Isaac Sim (those are later chapters)
- Advanced message types (geometry_msgs, sensor_msgs only intro)
- Custom message definitions (standard messages only)
- ROS 2 parameters in depth (surface-level only)
- Launch files complexity (simple examples only)
- Quality of Service (QoS) in depth
- Real-time constraints or timing

---

## 5. Technical Requirements

### Code Quality
- All Python code follows PEP 8 style guide
- Code examples are copy-pasteable and tested
- rclpy imports are correct (rclpy.init(), Node class, message types)
- No ROS 2 domain ID issues (clear setup instructions)

### Simulation Environment
- All examples verified to run in Turtlesim
- Setup instructions included (install, launch)
- Expected outputs shown (verified manually)
- No hardware dependencies

### Documentation
- Code examples include comments explaining key lines
- Error messages documented with solutions
- Terminal commands include expected output
- Clear distinction between copy-paste sections vs explanation

### Safety & Constitutional Compliance
- **ZERO mentions of physical hardware** in A2 (Simulation-first mandate)
- Kill switches NOT discussed (no autonomous movement in Turtlesim)
- No hardware damage scenarios mentioned
- Clear: "We're learning in simulation, hardware comes later"

---

## 6. Proposed Lesson Breakdown

### Lesson 1: What is ROS 2? (45 min)
**Layer**: 1 (Foundation)
**Concepts**: ROS 2, nodes, processes, concurrency, topics, messages
**Content**:
- Why ROS 2? (context: multi-process robot systems)
- What is a node? (independent executable)
- What is a topic? (communication channel)
- Pub/Sub pattern (why it scales better than function calls)
- Manual exercises: draw diagrams, understand relationships
- No code in this lesson

### Lesson 2: Setting Up Your Environment (30 min)
**Layer**: 1 (Foundation) → Layer 2 (Collaboration)
**Concepts**: ROS 2 setup, Turtlesim, terminal commands, environment variables
**Content**:
- Install ROS 2 (if needed)
- Verify installation
- Launch Turtlesim
- Explore rclpy library
- Use command-line tools (ros2 node list, ros2 topic list)

### Lesson 3: Your First Publisher (60 min)
**Layer**: 1 (Foundation) → Layer 2 (Collaboration)
**Concepts**: Publishers, messages, rclpy.Node, create_publisher(), timers, logging
**Content**:
- Explain publisher pattern
- Write minimal publisher (cmd_vel → Turtlesim)
- Run and verify turtle moves
- Troubleshoot common errors
- CoLearning: ask AI about node lifecycle, message types

### Lesson 4: Debugging Publishers (60 min)
**Layer**: 2 (Collaboration)
**Concepts**: Debugging, ros2 topic echo, logging, diagnostic thinking
**Content**:
- Using ros2 topic echo to verify messages
- Logging messages from nodes
- Common publisher mistakes (naming, types, not spinning)
- Collaboration exercise: break code, diagnose with AI
- Practice finding and fixing issues

### Lesson 5: Your First Subscriber (60 min)
**Layer**: 1 (Foundation) → Layer 2 (Collaboration)
**Concepts**: Subscribers, callbacks, message handlers, topic echoing
**Content**:
- Explain subscriber pattern
- Write minimal subscriber (read `/turtle1/pose`)
- Process incoming messages
- Compare pub vs sub (decoupling benefit)
- Exercise: create pub + sub that communicate

### Lesson 6: Building Systems (Multi-Node) (60 min)
**Layer**: 2 (Collaboration)
**Concepts**: System design, decoupling, asynchronous communication, independent execution
**Content**:
- Create 3+ nodes that work together
- Show how nodes run independently
- Demonstrate topic-based decoupling
- Collaboration exercise: design a simple robot system
- Capstone: Build a system that controls Turtlesim via multiple nodes

**Total: ~5 lessons, ~4.5 hours**

---

## 7. Research Foundation

**Official Sources**:
- [ROS 2 Humble Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Client Library (rclpy)](https://docs.ros2.org/latest/api/rclpy/)
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Simulation/Turtlesim/Turtlesim.html)
- [ROS 2 Topic Documentation](https://docs.ros2.org/latest/api/std_msgs/)

**Verified**:
- All ROS 2 API calls match Humble distribution
- Turtlesim available on Ubuntu 22.04 LTS
- rclpy example code tested locally

---

## 8. Success Criteria

### Content Quality
- [ ] All code examples tested and verified to work
- [ ] All claims verified against ROS 2 official docs
- [ ] No hallucinated APIs or outdated information
- [ ] Troubleshooting covers 90% of common beginner errors

### Pedagogical Quality
- [ ] Lesson sequence follows 4-Layer Method (Foundation → Collaboration → Intelligence → Orchestration)
- [ ] Layer 1 establishes conceptual understanding before Layer 2 code
- [ ] Layer 2 includes at least 3 AI CoLearning exercises
- [ ] Three Roles framework invisible to students (they experience it, don't see labels)
- [ ] No meta-commentary exposing pedagogy

### Simulation-First Compliance
- [ ] Zero hardware mentioned in A2 chapter
- [ ] All examples specify Turtlesim explicitly
- [ ] Setup instructions clear for student with no prior experience
- [ ] Expected outputs shown for every code block

### Rendering & Deployment
- [ ] Chapter renders correctly in Docusaurus
- [ ] All markdown valid (no broken links, code blocks syntax-correct)
- [ ] Images (if any) load correctly
- [ ] Mobile-responsive (tested on phone viewport)

---

## 9. Acceptance Checklist

**Before marking complete**:
- [ ] All 5-6 lessons written and validated
- [ ] Constitutional compliance verified (Section IX - Physical AI Safety)
- [ ] Code examples tested (output verified)
- [ ] Chapter renders in Docusaurus
- [ ] No broken links or missing assets
- [ ] Mobile-responsive design confirmed
- [ ] Troubleshooting section comprehensive
- [ ] Learning outcomes measurable and achievable
