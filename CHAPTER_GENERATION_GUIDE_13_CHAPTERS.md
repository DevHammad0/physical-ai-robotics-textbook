# Physical AI & Robotics Textbook - Chapter Generation Guide

**Last Updated**: 2025-11-28
**Status**: Ready for execution with ready-to-use prompts for all 13 chapters

---

## Overview

This guide provides **ready-to-use prompts** for generating all 13 chapters of the Physical AI & Humanoid Robotics textbook using the **manual Spec-Driven Development (SDD) pipeline** with existing `/sp.*` commands.

**Key Approach**:
- Use existing `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` commands
- Execute each command manually in sequence for each chapter
- Get user approval/review at each stage before proceeding
- Copy and paste the provided prompts directly into each command

---

## Course Structure: 13 Chapters Across 4 Modules

### Module 1: The Robotic Nervous System (ROS 2)
- Chapter 1: Introduction to ROS 2 Fundamentals (A2, Turtlesim)
- Chapter 2: ROS 2 Advanced Concepts (A2-B1, Turtlesim/Gazebo)
- Chapter 3: Building ROS 2 Packages (B1, Turtlesim/Gazebo)

### Module 2: The Digital Twin (Gazebo & Unity)
- Chapter 4: Gazebo Simulation Fundamentals (B1, Gazebo)
- Chapter 5: URDF and Robot Description (B1, Gazebo)
- Chapter 6: Sensor Simulation in Gazebo (B1, Gazebo)
- Chapter 7: Unity Integration for Visualization (B1-C2, Unity+Gazebo)

### Module 3: The AI-Robot Brain (NVIDIA Isaac Sim)
- Chapter 8: NVIDIA Isaac Sim Introduction (C2, Isaac Sim)
- Chapter 9: Isaac ROS and VSLAM (C2, Isaac Sim)
- Chapter 10: Nav2 Path Planning for Humanoids (C2, Isaac Sim)

### Module 4: Vision-Language-Action (VLA)
- Chapter 11: Voice-to-Action with OpenAI Whisper (C2, Isaac Sim)
- Chapter 12: LLM-Driven Cognitive Planning (C2, Isaac Sim)
- Chapter 13: Capstone - Autonomous Humanoid Integration (C2, Isaac Sim)

---

## Command Execution Workflow

For **EACH CHAPTER**, execute these commands in sequence:

### Step 1: Create Specification
```bash
/sp.specify "[paste the /sp.specify prompt for your chapter below]"
```

**Output**: `specs/chapter-N-slug/spec.md`
**Review**: Examine spec.md for learning outcomes, lesson structure, CEFR validation
**Action**: Approve or provide feedback

---

### Step 2: Create Lesson Plan
```bash
/sp.plan
```

**Note**: Automatically reads `specs/chapter-N-slug/spec.md`

**Output**: `specs/chapter-N-slug/plan.md`
**Review**: Examine lesson-by-lesson architecture, 4-Layer progression, safety level
**Action**: Approve or request changes

---

### Step 3: Generate Implementation Tasks
```bash
/sp.tasks
```

**Note**: Automatically reads `specs/chapter-N-slug/plan.md`

**Output**: `specs/chapter-N-slug/tasks.md`
**Review**: Examine content checklists, acceptance criteria, time estimates
**Action**: Approve

---

### Step 4: Implement All Lessons
```bash
/sp.implement
```

**Note**: Automatically reads `specs/chapter-N-slug/tasks.md`

**Output**: `book-source/docs/chapter-N-slug/[01-08]-lesson-name.md` files
**Review**: Check code examples, verify no forward references, assess pedagogical quality
**Action**: Approve or request revisions

---

### Step 5: Build and Validate in Docusaurus
```bash
cd book-source
pnpm run build
```

**Output**: `book-source/build/` static site
**Review**: Open browser, verify chapter renders correctly, check mobile responsiveness, test navigation links
**Action**: Verify all lessons display correctly

---

### Step 6: Create PHR Record (Optional)
Create `history/prompts/chapter-N-slug/001-chapter-N-specification.spec.prompt.md` to document the workflow for future reference.

---

## Ready-to-Use Prompts by Chapter

Copy and paste the `/sp.specify` prompt for your chapter directly into the `/sp.specify` command.

---

### CHAPTER 1: Introduction to ROS 2 Fundamentals

**Target**: A2 (Beginner) | Turtlesim | 4-5 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 1: "Introduction to ROS 2 Fundamentals"

Course Context: Physical AI & Humanoid Robotics textbook, Module 1 (Robotic Nervous System).

Chapter Goals:
- Teach ROS 2 architecture: nodes, topics, publishers, subscribers
- Use Turtlesim for all examples (no hardware required)
- Target A2 (Beginner) CEFR level: 5-7 concepts per lesson maximum
- Prepare students for Gazebo in Chapter 4

Learning Outcomes:
1. Understand why ROS 2 exists (multi-process robot systems)
2. Create ROS 2 nodes in Python using rclpy
3. Implement publishers and subscribers for topic-based communication
4. Debug ROS 2 systems using command-line tools
5. Design simple multi-node systems

Content Requirements:
- All code examples tested in Turtlesim simulation
- Simulation-first safety: NO physical hardware mentioned (A2 level)
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include troubleshooting sections for common beginner errors
- Self-assessment checklists in each lesson

Lessons (Proposed):
1. What is ROS 2? (Conceptual, no code)
2. Setting Up Your Environment (Turtlesim installation)
3. Your First Publisher (Python code, Twist messages)
4. Debugging ROS 2 Systems (ros2 command-line tools)
5. Your First Subscriber (Callbacks, message handling)
6. Building Multi-Node Systems (Integration exercise)

Constitutional Requirements:
- CEFR A2: Maximum 5-7 concepts per lesson
- Simulation-first: Turtlesim only, no hardware deployment
- No forward references to future concepts
- Three Roles framework: AI as Teacher/Student/Co-Worker (invisible to students)
```

---

### CHAPTER 2: ROS 2 Advanced Concepts

**Target**: A2-B1 (Beginner-Intermediate) | Turtlesim/Gazebo | 4-5 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 2: "ROS 2 Advanced Concepts"

Course Context: Physical AI & Humanoid Robotics textbook, Module 1 (Robotic Nervous System).

Prerequisites: Students completed Chapter 1 (ROS 2 Fundamentals - nodes, topics, publishers, subscribers).

Chapter Goals:
- Teach ROS 2 services (request-response communication)
- Teach ROS 2 actions (long-running tasks with feedback)
- Teach ROS 2 parameters (runtime configuration)
- Use Turtlesim + simple Gazebo models
- Target A2-B1 CEFR level: 5-7 concepts per lesson (A2) transitioning to 7-10 (B1)

Learning Outcomes:
1. Understand when to use topics vs. services vs. actions
2. Create ROS 2 service clients and servers
3. Implement ROS 2 action servers with feedback
4. Use parameters for runtime robot configuration
5. Apply launch files to orchestrate multi-node systems

Content Requirements:
- All code examples tested in Turtlesim and/or Gazebo
- Simulation-first safety: NO physical hardware deployment (A2-B1 level)
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include troubleshooting for services/actions (timeouts, missing servers)
- Self-assessment checklists

Lessons (Proposed):
1. Communication Patterns in ROS 2 (Topics vs. Services vs. Actions)
2. ROS 2 Services (Request-response, service types, synchronous calls)
3. Your First Service (Client/server implementation in Python)
4. ROS 2 Actions (Feedback, cancellation, asynchronous execution)
5. Your First Action Server (TurtleBot patrol example)
6. Parameters and Launch Files (Dynamic configuration, multi-node launch)

Constitutional Requirements:
- CEFR A2-B1: 5-7 concepts for foundational lessons, 7-10 for advanced
- Simulation-first: Turtlesim + Gazebo, no hardware
- No forward references to URDF (Chapter 5) or sensors (Chapter 6)
- Three Roles framework present
```

---

### CHAPTER 3: Building ROS 2 Packages

**Target**: B1 (Intermediate) | Turtlesim/Gazebo | 5 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 3: "Building ROS 2 Packages"

Course Context: Physical AI & Humanoid Robotics textbook, Module 1 (Robotic Nervous System).

Prerequisites: Students completed Chapters 1-2 (ROS 2 basics, services, actions).

Chapter Goals:
- Teach ROS 2 package structure (Python and C++ basics)
- Use colcon build system
- Create custom messages and services
- Write unit tests for ROS 2 nodes
- Target B1 (Intermediate) CEFR level: 7-10 concepts per lesson

Learning Outcomes:
1. Create ROS 2 packages using ros2 pkg create
2. Understand package.xml and setup.py structure
3. Define custom message types (.msg files)
4. Build and test packages with colcon
5. Write unit tests using pytest and ROS 2 testing tools

Content Requirements:
- All examples build and run in ROS 2 Humble
- Simulation-first: Test packages in Turtlesim/Gazebo
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include troubleshooting for build errors (missing dependencies, CMake issues)
- Self-assessment checklists

Lessons (Proposed):
1. What is a ROS 2 Package? (Workspace, package structure, dependencies)
2. Creating Your First Package (ros2 pkg create, package.xml, setup.py)
3. Custom Messages and Services (Defining .msg and .srv files)
4. Building with Colcon (colcon build, sourcing workspace)
5. Testing ROS 2 Nodes (pytest, unittest, launch_testing)
6. Package Organization Best Practices (Multi-package workspaces, interfaces)

Constitutional Requirements:
- CEFR B1: 7-10 concepts per lesson
- Simulation-first: Build for Turtlesim/Gazebo deployment
- No forward references to URDF (Chapter 5)
- Three Roles framework present
```

---

### CHAPTER 4: Gazebo Simulation Fundamentals

**Target**: B1 (Intermediate) | Gazebo | 5-6 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 4: "Gazebo Simulation Fundamentals"

Course Context: Physical AI & Humanoid Robotics textbook, Module 2 (Digital Twin).

Prerequisites: Students completed Chapter 1 (ROS 2 Fundamentals).

Chapter Goals:
- Teach Gazebo physics simulation environment
- Simulate robots with URDF (Unified Robot Description Format)
- Add sensors (LiDAR, cameras, IMU) to simulated robots
- Target B1 (Intermediate) CEFR level: 7-10 concepts per lesson maximum

Learning Outcomes:
1. Launch and configure Gazebo simulation environments
2. Create simple robot models using URDF
3. Understand physics simulation (gravity, friction, collisions)
4. Integrate sensors into simulated robots
5. Connect Gazebo to ROS 2 for robot control

Content Requirements:
- All code examples tested in Gazebo 11+ simulation
- Simulation-first safety: Can mention benchtop testing concepts (B1 level)
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include Gazebo troubleshooting (model loading issues, physics instability)
- Self-assessment checklists

Lessons (Proposed):
1. What is Gazebo? (Physics engines, simulation vs. reality)
2. Your First Gazebo World (Empty world, spawning models)
3. Introduction to URDF (Links, joints, visual vs. collision)
4. Building a Simple Robot (2-DOF arm or mobile base)
5. Adding Sensors to Your Robot (LiDAR, camera, IMU)
6. Gazebo-ROS 2 Integration (Publishing sensor data, controlling actuators)

Constitutional Requirements:
- CEFR B1: Maximum 7-10 concepts per lesson
- Simulation-first: Gazebo only, can mention hardware but NOT deployment
- Safety progression: B1 = benchtop testing tier (no field deployment yet)
- No forward references to Isaac Sim (Module 3) or VLA (Module 4)
- Three Roles framework present
```

---

### CHAPTER 5: URDF and Robot Description

**Target**: B1 (Intermediate) | Gazebo | 5 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 5: "URDF and Robot Description"

Course Context: Physical AI & Humanoid Robotics textbook, Module 2 (Digital Twin).

Prerequisites: Students completed Chapters 1-4 (ROS 2 + Gazebo basics).

Chapter Goals:
- Teach URDF (Unified Robot Description Format) for robot modeling
- Understand links, joints, and kinematic chains
- Visualize robots in RViz and Gazebo
- Create simple robot models from scratch
- Target B1 (Intermediate) CEFR level: 7-10 concepts per lesson

Learning Outcomes:
1. Understand URDF XML structure (links, joints, visual, collision)
2. Create simple robot models (mobile base, robotic arm)
3. Define joint types (revolute, prismatic, fixed, continuous)
4. Visualize robots in RViz
5. Spawn URDF models in Gazebo

Content Requirements:
- All URDF models tested in RViz and Gazebo
- Simulation-first: URDF for simulated robots only (B1 level)
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include troubleshooting for URDF parsing errors, joint issues
- Self-assessment checklists

Lessons (Proposed):
1. What is URDF? (Robot description, kinematics, XML structure)
2. Links and Visuals (Geometry, materials, collision vs. visual)
3. Joints and Kinematics (Joint types, parent-child relationships)
4. Creating a Mobile Robot Base (Wheels, chassis, differential drive)
5. Creating a Simple Robotic Arm (2-3 DOF, revolute joints)
6. Visualizing in RViz and Gazebo (Launch files, robot_state_publisher)

Constitutional Requirements:
- CEFR B1: 7-10 concepts per lesson
- Simulation-first: URDF for Gazebo only, no hardware URDF yet
- No forward references to sensors (Chapter 6) or Isaac Sim (Chapter 8)
- Three Roles framework present
```

---

### CHAPTER 6: Sensor Simulation in Gazebo

**Target**: B1 (Intermediate) | Gazebo | 5 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 6: "Sensor Simulation in Gazebo"

Course Context: Physical AI & Humanoid Robotics textbook, Module 2 (Digital Twin).

Prerequisites: Students completed Chapters 1-5 (ROS 2 + Gazebo + URDF).

Chapter Goals:
- Teach sensor integration in Gazebo (LiDAR, cameras, IMU)
- Understand sensor message types in ROS 2
- Process sensor data with Python subscribers
- Visualize sensor data in RViz
- Target B1 (Intermediate) CEFR level: 7-10 concepts per lesson

Learning Outcomes:
1. Add sensors to URDF models (camera, LiDAR, IMU plugins)
2. Understand ROS 2 sensor message types (sensor_msgs/LaserScan, Image, Imu)
3. Subscribe to sensor topics and process data
4. Visualize sensor data in RViz (point clouds, camera images)
5. Implement basic perception algorithms (obstacle detection)

Content Requirements:
- All sensors tested in Gazebo simulation
- Simulation-first: Simulated sensors only (B1 level)
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include troubleshooting for sensor plugins, TF issues
- Self-assessment checklists

Lessons (Proposed):
1. Introduction to Robot Sensors (LiDAR, cameras, IMU, depth sensors)
2. Adding Cameras to URDF (Gazebo camera plugin, image topics)
3. Adding LiDAR to URDF (LaserScan messages, range data)
4. Adding IMU to URDF (Orientation, angular velocity, linear acceleration)
5. Processing Sensor Data (Image processing basics, obstacle detection from LaserScan)
6. Sensor Fusion Basics (Combining multiple sensors, TF transforms)

Constitutional Requirements:
- CEFR B1: 7-10 concepts per lesson
- Simulation-first: Gazebo sensors only, no hardware deployment
- No forward references to Isaac Sim (Chapter 8) or advanced perception (Chapter 9)
- Three Roles framework present
```

---

### CHAPTER 7: Unity Integration for Visualization

**Target**: B1-C2 (Intermediate-Advanced) | Unity+Gazebo | 5 hours | 5 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 7: "Unity Integration for Visualization"

Course Context: Physical AI & Humanoid Robotics textbook, Module 2 (Digital Twin).

Prerequisites: Students completed Chapters 1-6 (ROS 2 + Gazebo + sensors).

Chapter Goals:
- Teach Unity as visualization tool for robotics
- Integrate Unity with ROS 2 using ROS-TCP-Connector
- Create photorealistic robot visualizations
- Understand when to use Unity vs. Gazebo vs. Isaac Sim
- Target B1-C2 CEFR level: 7-10 concepts (intermediate focus)

Learning Outcomes:
1. Set up Unity with ROS 2 integration
2. Import robot models (URDF) into Unity
3. Visualize ROS 2 topics in Unity (sensor data, robot state)
4. Create custom Unity visualizations for robot data
5. Understand trade-offs: Unity (visualization) vs. Gazebo (physics) vs. Isaac Sim (photorealism + AI)

Content Requirements:
- All examples use Unity 2021+ with ROS-TCP-Connector
- Simulation-first: Unity for visualization only (B1-C2 level)
- 4-Layer pedagogy: Layer 1 (foundation) + Layer 2 (AI collaboration)
- Include troubleshooting for Unity-ROS connection issues
- Self-assessment checklists

Lessons (Proposed):
1. Why Unity for Robotics? (Visualization, AR/VR, game engines vs. simulators)
2. Setting Up Unity with ROS 2 (ROS-TCP-Connector, Unity Robotics Hub)
3. Importing URDF into Unity (URDF Importer, materials, articulation)
4. Visualizing ROS Topics in Unity (Sensor data, robot state, custom visualizers)
5. Creating Interactive Demos (AR visualizations, remote monitoring)

Constitutional Requirements:
- CEFR B1-C2: 7-10 concepts per lesson, optional advanced sections
- Simulation-first: Unity for visualization, Gazebo for physics simulation
- No forward references to Isaac Sim (Chapter 8) workflows
- Three Roles framework present
```

---

### CHAPTER 8: NVIDIA Isaac Sim Introduction

**Target**: C2 (Advanced) | Isaac Sim | 6-7 hours | 7 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 8: "NVIDIA Isaac Sim Introduction"

Course Context: Physical AI & Humanoid Robotics textbook, Module 3 (AI-Robot Brain).

Prerequisites: Students completed Chapters 1-7 (ROS 2 + Gazebo).

Chapter Goals:
- Teach NVIDIA Isaac Sim for photorealistic robot simulation
- Generate synthetic data for AI training
- Understand sim-to-real transfer techniques
- Target C2 (Advanced) CEFR level: No concept limits, complex material OK

Learning Outcomes:
1. Set up NVIDIA Isaac Sim and Omniverse
2. Create photorealistic simulation environments
3. Use Isaac Sim's AI-powered perception tools
4. Generate synthetic datasets for training
5. Apply sim-to-real transfer techniques

Content Requirements:
- All code examples tested in Isaac Sim 2023.1+
- Safety: C2 level = field deployment tier (can discuss real robot deployment with safety protocols)
- 4-Layer pedagogy: All 4 layers (foundation, collaboration, intelligence design, orchestration)
- Include Isaac Sim-specific troubleshooting (USD asset loading, RTX requirements)
- Hardware requirements documented (RTX GPU needed)

Lessons (Proposed):
1. What is Isaac Sim? (Omniverse, USD assets, photorealism)
2. Setting Up Isaac Sim (Installation, system requirements)
3. Building Your First Scene (Warehouse environment, robot assets)
4. Synthetic Data Generation (Camera sensors, domain randomization)
5. Isaac ROS Integration (Perception pipelines, SLAM)
6. Sim-to-Real Transfer Basics (What transfers, what doesn't)
7. Advanced: Training in Simulation (Reinforcement learning, imitation learning)

Constitutional Requirements:
- CEFR C2: Unlimited concepts, advanced material permitted
- Safety: Can discuss field deployment but MUST include kill switches, safety protocols
- Can reference future concepts (VLA in Chapter 11-12) for context
- Three Roles framework + Layer 3-4 pedagogical content
```

---

### CHAPTER 9: Isaac ROS and VSLAM

**Target**: C2 (Advanced) | Isaac Sim | 6 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 9: "Isaac ROS and VSLAM"

Course Context: Physical AI & Humanoid Robotics textbook, Module 3 (AI-Robot Brain).

Prerequisites: Students completed Chapters 1-8 (ROS 2 + Gazebo + Isaac Sim intro).

Chapter Goals:
- Teach Visual SLAM (Simultaneous Localization and Mapping) using Isaac ROS
- Use Isaac Sim for VSLAM testing and synthetic data generation
- Understand VSLAM algorithms (ORB-SLAM3, RTAB-Map)
- Target C2 (Advanced) CEFR level: Complex algorithms, no concept limits

Learning Outcomes:
1. Understand VSLAM concepts (keyframes, feature tracking, loop closure)
2. Set up Isaac ROS perception pipelines
3. Run VSLAM algorithms in Isaac Sim
4. Generate synthetic datasets for VSLAM training
5. Evaluate VSLAM performance (trajectory error, map quality)

Content Requirements:
- All examples tested in Isaac Sim with Isaac ROS
- Safety: C2 level, can discuss real robot deployment with safety
- 4-Layer pedagogy: All 4 layers (foundation, collaboration, intelligence, orchestration)
- Include troubleshooting for VSLAM (feature detection failures, drift)
- Self-assessment checklists

Lessons (Proposed):
1. Introduction to VSLAM (Localization vs. mapping, visual odometry)
2. Isaac ROS Setup (GEM packages, perception nodes)
3. Running VSLAM in Isaac Sim (ORB-SLAM3, camera configuration)
4. Synthetic Data for VSLAM (Domain randomization, lighting variations)
5. Evaluating VSLAM Performance (ATE, RPE, trajectory visualization)
6. Advanced: Multi-Session SLAM (Loop closure, map merging)

Constitutional Requirements:
- CEFR C2: Unlimited concepts, advanced algorithms permitted
- Safety: Include discussion of VSLAM failures in real robots, safety protocols
- No forward references to Nav2 (Chapter 10) path planning
- Three Roles framework + Layer 3-4 content
```

---

### CHAPTER 10: Nav2 Path Planning for Humanoids

**Target**: C2 (Advanced) | Isaac Sim | 6 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 10: "Nav2 Path Planning for Humanoids"

Course Context: Physical AI & Humanoid Robotics textbook, Module 3 (AI-Robot Brain).

Prerequisites: Students completed Chapters 1-9 (ROS 2 + Gazebo + Isaac Sim + VSLAM).

Chapter Goals:
- Teach Nav2 (ROS 2 navigation stack) for autonomous navigation
- Apply Nav2 to humanoid robots in Isaac Sim
- Understand path planning algorithms (Dijkstra, A*, DWA)
- Target C2 (Advanced) CEFR level: Complex planning algorithms

Learning Outcomes:
1. Understand Nav2 architecture (planners, controllers, recovery behaviors)
2. Configure Nav2 for humanoid robots
3. Implement global path planning (costmaps, A* planner)
4. Implement local trajectory planning (DWA, TEB)
5. Test autonomous navigation in Isaac Sim

Content Requirements:
- All examples tested in Isaac Sim with Nav2
- Safety: C2 level, MUST include collision avoidance, safety zones, kill switches
- 4-Layer pedagogy: All 4 layers
- Include troubleshooting for Nav2 (planner failures, costmap issues)
- Self-assessment checklists

Lessons (Proposed):
1. Introduction to Nav2 (Navigation stack, behavior trees, planners)
2. Nav2 Configuration for Humanoids (Footprint, kinematic constraints)
3. Global Path Planning (Costmaps, A* planner, inflation layers)
4. Local Trajectory Planning (DWA controller, obstacle avoidance)
5. Recovery Behaviors (Stuck detection, clearing costmaps)
6. Autonomous Navigation in Isaac Sim (Warehouse scenario, multi-waypoint missions)

Constitutional Requirements:
- CEFR C2: Unlimited concepts, advanced path planning algorithms
- Safety: MANDATORY collision avoidance, safety boundaries, emergency stop
- No forward references to VLA (Chapter 11-12)
- Three Roles framework + Layer 3-4 content
```

---

### CHAPTER 11: Voice-to-Action with OpenAI Whisper

**Target**: C2 (Advanced) | Isaac Sim | 5 hours | 5 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 11: "Voice-to-Action with OpenAI Whisper"

Course Context: Physical AI & Humanoid Robotics textbook, Module 4 (Vision-Language-Action).

Prerequisites: Students completed Chapters 1-10 (ROS 2 + Gazebo + Isaac Sim + Nav2).

Chapter Goals:
- Teach voice recognition using OpenAI Whisper
- Integrate Whisper with ROS 2 for voice-controlled robots
- Map speech to robot actions (natural language → ROS 2 commands)
- Target C2 (Advanced) CEFR level: Multi-modal AI integration

Learning Outcomes:
1. Set up OpenAI Whisper for real-time speech recognition
2. Create ROS 2 nodes for voice input processing
3. Map natural language commands to robot actions
4. Implement safety checks for voice commands
5. Test voice-controlled navigation in Isaac Sim

Content Requirements:
- All examples tested in Isaac Sim with Whisper integration
- Safety: C2 level, MANDATORY safety parsing (reject dangerous commands like "drive off cliff")
- 4-Layer pedagogy: All 4 layers (orchestration focus)
- Include troubleshooting for Whisper (audio input issues, recognition errors)
- Self-assessment checklists

Lessons (Proposed):
1. Introduction to Voice-Language-Action (VLA overview, speech recognition)
2. Setting Up OpenAI Whisper (Installation, model selection, real-time processing)
3. Voice Input ROS 2 Integration (Audio capture, Whisper node, action publishing)
4. Natural Language Command Parsing (Intent detection, slot filling)
5. Safety-Aware Voice Control (Command validation, safety boundaries, kill switch integration)

Constitutional Requirements:
- CEFR C2: Unlimited concepts, multi-modal AI integration
- Safety: MANDATORY command validation, reject dangerous actions, safety boundaries
- Can reference Chapter 12 (LLM planning) for context
- Three Roles framework + Layer 4 (orchestration) heavily emphasized
```

---

### CHAPTER 12: LLM-Driven Cognitive Planning

**Target**: C2 (Advanced) | Isaac Sim | 6 hours | 6 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 12: "LLM-Driven Cognitive Planning"

Course Context: Physical AI & Humanoid Robotics textbook, Module 4 (Vision-Language-Action).

Prerequisites: Students completed Chapters 1-11 (ROS 2 + Isaac Sim + Voice control).

Chapter Goals:
- Teach LLM integration for robot task planning (GPT-4, Claude)
- Map natural language goals to ROS 2 action sequences
- Implement cognitive planning (high-level reasoning → low-level actions)
- Target C2 (Advanced) CEFR level: Complex AI reasoning integration

Learning Outcomes:
1. Understand LLM capabilities for robot planning
2. Design prompt templates for task decomposition
3. Translate LLM output to ROS 2 actions
4. Implement closed-loop planning (plan → execute → observe → replan)
5. Test LLM-driven planning in Isaac Sim scenarios

Content Requirements:
- All examples tested in Isaac Sim with GPT-4/Claude API integration
- Safety: C2 level, MANDATORY plan validation (verify actions are safe before execution)
- 4-Layer pedagogy: All 4 layers (Layer 4 orchestration focus)
- Include troubleshooting for LLM integration (API failures, hallucinations)
- Self-assessment checklists

Lessons (Proposed):
1. LLMs for Robot Planning (Cognitive reasoning, task decomposition, action grounding)
2. Prompt Engineering for Robotics (Few-shot examples, action libraries, safety constraints)
3. LLM-ROS 2 Integration (API calls, action translation, execution monitoring)
4. Closed-Loop Planning (Observe-Plan-Act cycle, replanning on failure)
5. Safety Validation for LLM Plans (Action verification, simulation testing before execution)
6. Advanced: Multi-Agent LLM Planning (Coordination, task allocation)

Constitutional Requirements:
- CEFR C2: Unlimited concepts, complex LLM reasoning
- Safety: MANDATORY plan validation, simulate before execution, kill switch integration
- Can reference Chapter 13 (Capstone) for integration context
- Three Roles framework + Layer 4 (orchestration) heavily emphasized
```

---

### CHAPTER 13: Capstone - Autonomous Humanoid Integration

**Target**: C2 (Advanced) | Isaac Sim | 8-10 hours | 8 lessons

#### `/sp.specify` Prompt:

```
Create a specification for Chapter 13: "Capstone - Autonomous Humanoid Integration"

Course Context: Physical AI & Humanoid Robotics textbook, Module 4 (Vision-Language-Action), final capstone.

Prerequisites: Students completed all Chapters 1-12 (ROS 2 + Gazebo + Isaac + VLA).

Chapter Goals:
- Integrate all course concepts into a complete autonomous humanoid system
- Voice command → plan → navigate → manipulate pipeline
- Deploy to simulated humanoid robot (Unitree G1 or similar)
- Target C2 (Advanced): Complex multi-system integration

Learning Outcomes:
1. Design an end-to-end autonomous robot system
2. Integrate voice recognition (Whisper) with robot control
3. Use LLMs for cognitive planning (natural language → ROS 2 actions)
4. Navigate obstacles using Nav2 path planning
5. Manipulate objects using perception + grasping
6. Deploy and test the complete system in Isaac Sim

Content Requirements:
- Complete autonomous pipeline: voice → LLM → ROS 2 → Isaac Sim
- Safety: Full safety protocols (kill switch, movement boundaries, error handling)
- 4-Layer pedagogy: Focus on Layer 4 (orchestration, spec-driven assembly)
- Extensive troubleshooting for integration issues
- Project rubric and assessment criteria

Lessons (Proposed):
1. Capstone Overview (System architecture, requirements)
2. Voice Interface Setup (Whisper integration with ROS 2)
3. LLM Cognitive Planner (GPT-4 → action sequence translation)
4. Navigation Pipeline (Nav2, obstacle avoidance, path planning)
5. Perception and Grasping (Object detection, manipulation planning)
6. System Integration (Connect all components)
7. Testing and Deployment (Sim testing, real-world considerations)
8. Final Demo and Assessment

Constitutional Requirements:
- CEFR C2: Complex integration, no concept limits
- Safety: MANDATORY kill switch, safety boundaries, failure modes documented
- Simulation-first: Test in Isaac Sim before ANY hardware deployment
- Three Roles + Layer 4 (spec-driven orchestration) heavily emphasized
- Include "what would change for real hardware" sections
```

---

## Quick Reference Table

| # | Chapter Title | Module | CEFR | Simulator | Duration |
|---|---|---|---|---|---|
| 1 | Introduction to ROS 2 Fundamentals | M1 | A2 | Turtlesim | 4-5h |
| 2 | ROS 2 Advanced Concepts | M1 | A2-B1 | Turtlesim | 4-5h |
| 3 | Building ROS 2 Packages | M1 | B1 | Turtlesim/Gazebo | 5h |
| 4 | Gazebo Simulation Fundamentals | M2 | B1 | Gazebo | 5-6h |
| 5 | URDF and Robot Description | M2 | B1 | Gazebo | 5h |
| 6 | Sensor Simulation in Gazebo | M2 | B1 | Gazebo | 5h |
| 7 | Unity Integration for Visualization | M2 | B1-C2 | Unity | 5h |
| 8 | NVIDIA Isaac Sim Introduction | M3 | C2 | Isaac Sim | 6-7h |
| 9 | Isaac ROS and VSLAM | M3 | C2 | Isaac Sim | 6h |
| 10 | Nav2 Path Planning for Humanoids | M3 | C2 | Isaac Sim | 6h |
| 11 | Voice-to-Action with OpenAI Whisper | M4 | C2 | Isaac Sim | 5h |
| 12 | LLM-Driven Cognitive Planning | M4 | C2 | Isaac Sim | 6h |
| 13 | Capstone - Autonomous Humanoid | M4 | C2 | Isaac Sim | 8-10h |

**Total**: ~70-80 hours of content

---

## Execution Checklist

### For Each Chapter:

- [ ] Copy `/sp.specify` prompt from this guide
- [ ] Run `/sp.specify "[pasted prompt]"`
- [ ] Review `specs/chapter-N-slug/spec.md` → approve or provide feedback
- [ ] Run `/sp.plan` (auto-reads spec.md)
- [ ] Review `specs/chapter-N-slug/plan.md` → approve
- [ ] Run `/sp.tasks` (auto-reads plan.md)
- [ ] Review `specs/chapter-N-slug/tasks.md` → approve
- [ ] Run `/sp.implement` (auto-reads tasks.md)
- [ ] Review lesson files → approve
- [ ] Build Docusaurus: `cd book-source && pnpm run build`
- [ ] Test rendering and navigation
- [ ] Verify mobile responsiveness
- [ ] Mark chapter as complete

---

## Recommended Execution Sequence

**For Time Efficiency**:

1. **Quick Win (Hours 1-4)**: Complete Chapter 1 (ROS 2 Fundamentals)
   - Validates the approach early
   - Generates foundational content
   - Identifies any issues before scaling

2. **Foundation Module (Hours 5-8)**: Complete Chapters 2-3 (ROS 2 advanced + packages)
   - Builds on Chapter 1
   - Completes Module 1

3. **Digital Twin Module (Hours 9-15)**: Complete Chapters 4-7 (Gazebo + Unity)
   - Extends simulation capabilities
   - Completes Module 2

4. **AI-Robot Brain Module (Hours 16-22)**: Complete Chapters 8-10 (Isaac + VSLAM + Nav2)
   - Moves to advanced simulation and AI
   - Completes Module 3

5. **VLA Module (Hours 23-30)**: Complete Chapters 11-13 (Voice + LLM + Capstone)
   - Integrates all concepts
   - Completes the course

---

## Notes

- Each chapter's `/sp.plan`, `/sp.tasks`, and `/sp.implement` prompts are **auto-generated** from the spec.md file
- You only need to copy the `/sp.specify` prompts provided in this guide
- The subsequent commands (`/sp.plan`, `/sp.tasks`, `/sp.implement`) will automatically reference the generated files from previous steps
- All lessons follow the **4-Layer Method** with simulation-first safety principles
- Constitutional compliance is enforced throughout (CEFR limits, Three Roles framework, no forward references)

---

**Document Status**: Ready for execution
**Last Updated**: 2025-11-28
