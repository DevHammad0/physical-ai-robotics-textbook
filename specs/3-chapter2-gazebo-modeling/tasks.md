# Tasks: Chapter 2 - Simulation & Robot Modeling

**Input**: Design documents from `/specs/3-chapter2-gazebo-modeling/`
**Prerequisites**: plan.md ✅ (specified), spec.md ✅ (7 user stories), research.md (pending), data-model.md (pending)

**Organization**: Tasks grouped by user story (P1, P2, P3) to enable independent implementation and testing. Each story can be delivered as a complete, independently-testable increment.

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1, US2... US7)
- Include exact file paths in descriptions
- Total Tasks: 95 across 9 phases

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Directory structure and base infrastructure

- [ ] T001 Create chapter directory structure in `book-source/docs/chapter-2-gazebo-modeling/`
- [ ] T002 Create examples directory structure in `examples/chapter-2-gazebo/` with subdirectories for each lesson
- [ ] T003 Create tests directory structure in `tests/gazebo-physics/`, `tests/urdf-validation/`, `tests/ros2-integration/`
- [ ] T004 Initialize git tracking for Chapter 2 files (specs/, book-source/, examples/, tests/)

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Purpose**: Core templates and shared resources MUST complete before user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Create URDF template with standard link/joint/sensor structure in `specs/3-chapter2-gazebo-modeling/contracts/urdf-template.xml`
- [ ] T006 [P] Create ROS 2 sensor subscriber template in `specs/3-chapter2-gazebo-modeling/contracts/sensor-subscriber-template.py`
- [ ] T007 [P] Create ROS 2 joint controller template in `specs/3-chapter2-gazebo-modeling/contracts/joint-controller-template.py`
- [ ] T008 [P] Create Gazebo launch file template in `specs/3-chapter2-gazebo-modeling/contracts/launch-template.py`
- [ ] T009 [P] Copy `gazebo-urdf-patterns` skill content to `.specify/skills/gazebo-urdf-patterns/` (URDF best practices, physics tuning, multi-robot patterns)
- [ ] T010 [P] Copy `sensor-integration-patterns` skill content to `.specify/skills/sensor-integration-patterns/` (image processing, LaserScan, IMU, message filtering)
- [ ] T011 Create Gazebo physics tuning reference guide in `specs/3-chapter2-gazebo-modeling/physics-tuning-reference.md` (gravity, friction, damping ranges, stability tips)
- [ ] T012 Create troubleshooting template in `specs/3-chapter2-gazebo-modeling/troubleshooting-template.md` (common Gazebo/URDF/ROS 2 errors)
- [ ] T013 Create self-assessment checklist template in `specs/3-chapter2-gazebo-modeling/self-assessment-template.md` (observable criteria per lesson)

**Checkpoint**: Foundation complete - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Launch Gazebo & Understand Physics (Priority: P1)

**Goal**: Students successfully launch Gazebo, observe physics simulation, and understand physics engine concepts

**Independent Test**: Launch Gazebo, place objects, observe gravity/friction/collision behavior, complete self-assessment checklist

### Lesson Content & Examples for US1

- [ ] T014 [P] [US1] Create lesson intro in `book-source/docs/chapter-2-gazebo-modeling/intro.md` with chapter overview, prerequisites, learning paths, chapter learning outcomes
- [ ] T015 [US1] Create lesson-01-intro-gazebo.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-01-intro-gazebo.md` (45 min, B1):
  - Learning outcomes (4 items)
  - Concepts (7-10: Gazebo ecosystem, physics engines ODE/Bullet/DART, world structure, simulation loop, physics plugins, visualization, SDF vs URDF)
  - Architecture diagram (ASCII)
  - Physics engine comparison table (ODE vs Bullet vs DART trade-offs)
  - Gazebo launch walkthrough (ros2 launch gazebo)
  - 5+ troubleshooting entries (Gazebo won't launch, graphics issues, physics unstable, plugin errors)
  - 6+ self-assessment items

- [ ] T016 [P] [US1] Create physics-engine-comparison-table in `examples/chapter-2-gazebo/lesson-01/physics-engines.txt` with detailed ODE/Bullet/DART comparison

- [ ] T017 [P] [US1] Create Gazebo launch verification script in `examples/chapter-2-gazebo/lesson-01/verify-gazebo-install.sh` (verify Gazebo 11+ installed, check plugins, test empty world launch)

- [ ] T018 [US1] Add installation verification to lesson-01 (steps to verify Gazebo 11+ installed, check ROS 2 Humble + Gazebo integration)

**Checkpoint**: User Story 1 complete - students understand Gazebo and physics concepts

---

## Phase 4: User Story 2 - Create Robot Models Using URDF (Priority: P1)

**Goal**: Students create URDF robot models, understand kinematic structure, and visualize robots in RViz

**Independent Test**: Write URDF for multi-link robot, load in RViz, verify kinematic chain correctness, spawn in Gazebo

### Lesson Content & Examples for US2

- [ ] T019 [P] [US2] Create lesson-02-urdf-basics.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-02-urdf-basics.md` (60 min, B1):
  - Learning outcomes (4 items)
  - Concepts (7-10: URDF XML structure, links, joints, collision geometry, inertia, visual geometry, joint axes, joint types)
  - URDF structure explanation with examples
  - Link anatomy (inertial, visual, collision sections)
  - Joint anatomy (parent, child, type, limits, axis)
  - Self-assessment checklist (write simple URDF, verify in RViz, check collision geometry)
  - 5+ troubleshooting (URDF parsing errors, link not found, joint cycles, inertia issues)

- [ ] T020 [P] [US2] Create two-link robot URDF in `examples/chapter-2-gazebo/lesson-02/two-link-robot.urdf` (simple, 2 links + 1 joint, validatable)

- [ ] T021 [P] [US2] Create URDF visualization script in `examples/chapter-2-gazebo/lesson-02/visualize-urdf.sh` (load URDF in RViz, show kinematic tree)

- [ ] T022 [P] [US2] Create URDF validation test in `tests/urdf-validation/test_urdf_syntax.py` (pytest: XML parsing, link reference checking, joint parent/child validation)

- [ ] T023 [US2] Create multi-link robot URDF example in `examples/chapter-2-gazebo/lesson-02/multi-link-arm.urdf` (3-link arm with revolute joints, more complex than T020)

- [ ] T024 [US2] Add Gazebo integration section to lesson-02 (spawn URDF in Gazebo, apply physics, verify collision detection)

**Checkpoint**: User Story 2 complete - students create and validate URDF models

---

## Phase 5: User Story 3 - Understand Physics Simulation & Tuning (Priority: P1)

**Goal**: Students understand physics parameters, tune simulation for realistic behavior, and achieve specified robot movement

**Independent Test**: Adjust friction/damping, verify behavior changes predictably, achieve "move 1m on flat ground" specification

### Lesson Content & Examples for US3

- [ ] T025 [P] [US3] Create lesson-04-physics-simulation.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-04-physics-simulation.md` (75 min, B1):
  - Learning outcomes (4 items)
  - Concepts (8: gravity, friction static/dynamic, linear/angular damping, restitution, contact margin, physics engine selection, stability)
  - Physics parameters explanation (gravity value, friction ranges, damping typical values)
  - Physics tuning methodology (iterative vs analytical)
  - Stability detection (oscillation, bouncing, energy conservation)
  - Self-assessment (tune friction to achieve sliding behavior, predict gravity effect)
  - 5+ troubleshooting (simulation unstable, objects bounce excessively, extreme accelerations)

- [ ] T026 [P] [US3] Create differential-drive physics tuning URDF in `examples/chapter-2-gazebo/lesson-04/differential-drive-tuned.urdf` (mobile robot with friction/damping parameters for realistic movement)

- [ ] T027 [P] [US3] Create physics tuning reference in `examples/chapter-2-gazebo/lesson-04/physics-parameter-ranges.txt` (friction: 0.3-1.0, damping: 0.01-0.5, gravity: 9.81)

- [ ] T028 [P] [US3] Create physics stability test in `tests/gazebo-physics/test_physics_stability.py` (pytest: verify gravity pulls objects down, check friction coefficient effect)

- [ ] T029 [US3] Create incline physics example in `examples/chapter-2-gazebo/lesson-04/incline-friction-demo.urdf` + launch (object on incline, tunable friction for slide/stick behavior)

**Checkpoint**: User Story 3 complete - students tune physics for realistic simulation

---

## Phase 6: User Story 4 - Add Sensors to Robots (Priority: P2)

**Goal**: Students add and configure sensor plugins (camera, LiDAR, IMU) to robot models

**Independent Test**: Add sensors to URDF, launch in Gazebo, verify ROS 2 topics publish sensor data, visualize in RViz

### Lesson Content & Examples for US4

- [ ] T030 [P] [US4] Create lesson-05-adding-sensors.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-05-adding-sensors.md` (75 min, B1):
  - Learning outcomes (4 items)
  - Concepts (8: Gazebo sensor plugins, camera intrinsics, LiDAR ray casting, IMU simulation, noise models, sensor placement, frame attachment, ROS 2 topic mapping)
  - Sensor plugin architecture (gazebo_ros_camera, gazebo_ros_lidar, gazebo_ros_imu)
  - Camera intrinsics (focal length, principal point, image resolution)
  - LiDAR beam model (angular resolution, range limits, noise)
  - IMU noise model (accelerometer, gyroscope noise)
  - Self-assessment (add camera to URDF, configure LiDAR range, enable IMU noise)
  - 5+ troubleshooting (plugin not found, topics not publishing, sensor frame issues)

- [ ] T031 [P] [US4] Create camera-equipped robot URDF in `examples/chapter-2-gazebo/lesson-05/robot-with-camera.urdf` (differential drive + forward-facing camera)

- [ ] T032 [P] [US4] Create LiDAR-equipped robot URDF in `examples/chapter-2-gazebo/lesson-05/robot-with-lidar.urdf` (differential drive + 360° LiDAR)

- [ ] T033 [P] [US4] Create IMU-equipped robot URDF in `examples/chapter-2-gazebo/lesson-05/robot-with-imu.urdf` (differential drive + IMU mounted on base)

- [ ] T034 [P] [US4] Create multi-sensor robot URDF in `examples/chapter-2-gazebo/lesson-05/robot-with-all-sensors.urdf` (camera + LiDAR + IMU on single robot)

- [ ] T035 [P] [US4] Create sensor plugin test in `tests/ros2-integration/test_sensor_plugins.py` (pytest: verify sensor topics exist, check message types)

- [ ] T036 [US4] Create sensor launch file in `examples/chapter-2-gazebo/lesson-05/spawn-sensors.launch.py` (launch robot with all sensors, remap topic names)

**Checkpoint**: User Story 4 complete - students add multiple sensors to robots

---

## Phase 7: User Story 5 - Process Sensor Data via ROS 2 (Priority: P2)

**Goal**: Students write ROS 2 subscribers to process sensor data and extract meaningful information

**Independent Test**: Write image subscriber, extract features; write LaserScan subscriber, detect obstacles; synchronize multi-sensor data

### Lesson Content & Examples for US5

- [ ] T037 [P] [US5] Create lesson-06-processing-sensors.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-06-processing-sensors.md` (75 min, B1):
  - Learning outcomes (4 items)
  - Concepts (8: image subscribers, LaserScan subscribers, message filtering, time synchronization, visualization in RViz, edge detection, obstacle detection)
  - ROS 2 Image message structure (encoding, width, height, data)
  - LaserScan message structure (angle_min, angle_max, range_min, range_max, ranges, intensities)
  - IMU message structure (linear_acceleration, angular_velocity)
  - message_filters for time synchronization (ApproximateTimeSynchronizer)
  - Self-assessment (subscribe to image, extract edges; subscribe to scan, find obstacles)
  - 5+ troubleshooting (message dropped, time sync failures, encoding mismatches)

- [ ] T038 [P] [US5] Create image subscriber example in `examples/chapter-2-gazebo/lesson-06/camera_subscriber.py` (subscribe to /camera/image_raw, convert to OpenCV, display edge detection):
  - Imports: rclpy, sensor_msgs.msg.Image, cv_bridge, cv2
  - Node class with Image subscriber
  - Callback: convert ROS Image → OpenCV Mat, apply Canny edge detection
  - Publish processed image back to ROS 2
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T039 [P] [US5] Create LaserScan subscriber example in `examples/chapter-2-gazebo/lesson-06/lidar_subscriber.py` (subscribe to /scan, extract range data, find obstacles):
  - Imports: rclpy, sensor_msgs.msg.LaserScan
  - Node class with LaserScan subscriber
  - Callback: extract range array, find min range (obstacle distance)
  - Publish obstacle detection on custom topic
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T040 [P] [US5] Create IMU subscriber example in `examples/chapter-2-gazebo/lesson-06/imu_subscriber.py` (subscribe to /imu/data, extract acceleration/rotation):
  - Imports: rclpy, sensor_msgs.msg.Imu
  - Node class with IMU subscriber
  - Callback: extract linear_acceleration, angular_velocity, convert to readable format
  - Log orientation for debugging
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T041 [P] [US5] Create multi-sensor fusion example in `examples/chapter-2-gazebo/lesson-06/sensor_fusion_node.py` (subscribe to camera + LaserScan + IMU, correlate data):
  - Imports: rclpy, message_filters.Subscriber, message_filters.ApproximateTimeSynchronizer
  - Node class with synchronized subscribers
  - TimeSynchronizer callback: receive image + scan + imu at same timestamp
  - Example: correlate image features with LiDAR detections
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T042 [P] [US5] Create sensor subscriber unit tests in `tests/ros2-integration/test_sensor_subscribers.py` (pytest: mock sensor topics, verify callback invocation, check message processing)

- [ ] T043 [US5] Create sensor data processing visualization guide in `examples/chapter-2-gazebo/lesson-06/rqt-plotting.md` (use rqt_plot to visualize LaserScan ranges, IMU accelerations)

**Checkpoint**: User Story 5 complete - students process sensor data with ROS 2

---

## Phase 8: User Story 6 - Control Robots via ROS 2 (Priority: P2)

**Goal**: Students write ROS 2 publishers to command robot actuators and observe behavior in Gazebo

**Independent Test**: Publish Twist messages, observe differential drive robot motion; publish JointTrajectory, observe arm movement

### Lesson Content & Examples for US6

- [ ] T044 [P] [US6] Create lesson-07-gazebo-ros2-integration.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-07-gazebo-ros2-integration.md` (75 min, B1):
  - Learning outcomes (4 items)
  - Concepts (8: joint command interface, state publishing, TF broadcasting, feedback control, PID tuning, error handling, ROS 2 actions vs services, controller architecture)
  - Joint controller types (velocity, effort, position)
  - Twist message for differential drive (linear.x, angular.z)
  - JointTrajectory for arm control (joint_names, points, velocities, accelerations)
  - GripperCommand for gripper control
  - State feedback (JointState, Odometry)
  - Self-assessment (publish Twist, move differential drive; publish JointTrajectory, move arm)
  - 5+ troubleshooting (commands not working, joint limits exceeded, state feedback missing)

- [ ] T045 [P] [US6] Create differential drive controller in `examples/chapter-2-gazebo/lesson-07/differential_drive_controller.py` (subscribe to /cmd_vel, publish to left/right wheel effort):
  - Imports: rclpy, geometry_msgs.msg.Twist, std_msgs.msg.Float64
  - Node class with Twist subscriber
  - Callback: convert Twist (linear.x, angular.z) to left/right wheel velocities using differential drive kinematics
  - Publish Float64 messages to /left_wheel_joint/effort and /right_wheel_joint/effort
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T046 [P] [US6] Create arm joint controller in `examples/chapter-2-gazebo/lesson-07/arm_joint_controller.py` (publish JointTrajectory to move 7-DOF arm):
  - Imports: rclpy, trajectory_msgs.msg.JointTrajectory, trajectory_msgs.msg.JointTrajectoryPoint
  - Node class with JointTrajectory publisher
  - Create trajectory with multiple waypoints (move to different joint configurations)
  - Publish trajectory and monitor joint state feedback
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T047 [P] [US6] Create state feedback listener in `examples/chapter-2-gazebo/lesson-07/joint_state_listener.py` (subscribe to /joint_states, log current joint angles):
  - Imports: rclpy, sensor_msgs.msg.JointState
  - Node class with JointState subscriber
  - Callback: extract joint names and positions, log in readable format
  - Verify feedback loop (publish command → observe state change)
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T048 [P] [US6] Create gripper controller in `examples/chapter-2-gazebo/lesson-07/gripper_controller.py` (publish GripperCommand, open/close gripper):
  - Imports: rclpy, std_srvs.srv.SetBool OR custom gripper msg
  - Node class with gripper command publisher
  - Commands to open/close gripper
  - Monitor gripper state feedback
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T049 [P] [US6] Create PID feedback controller in `examples/chapter-2-gazebo/lesson-07/pid_control_demo.py` (basic proportional control for joint angle):
  - Imports: rclpy, sensor_msgs.msg.JointState, std_msgs.msg.Float64
  - Simple PID class (proportional only, for educational demo)
  - Target joint angle → calculate error → apply proportional effort
  - Publish effort and monitor convergence
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T050 [P] [US6] Create controller unit tests in `tests/ros2-integration/test_controllers.py` (pytest: verify Twist → wheel velocity conversion, JointTrajectory generation, state feedback parsing)

- [ ] T051 [US6] Create Gazebo-ROS 2 integration launch file in `examples/chapter-2-gazebo/lesson-07/robot_control.launch.py` (spawn robot, start Gazebo, load ROS 2 controllers, connect to sensor/command topics)

**Checkpoint**: User Story 6 complete - students command robots and receive feedback via ROS 2

---

## Phase 9: User Story 7 - Multi-Robot Coordination (Priority: P3)

**Goal**: Students simulate multiple robots, coordinate via ROS 2 messaging, implement collision avoidance and leader-follower patterns

**Independent Test**: Launch 2+ robots, verify unique namespaces, implement simple collision avoidance, achieve leader-follower behavior

### Lesson Content & Examples for US7

- [ ] T052 [P] [US7] Create lesson-08-multi-robot-capstone.md in `book-source/docs/chapter-2-gazebo-modeling/lesson-08-multi-robot-capstone.md` (120 min, B1):
  - Learning outcomes (4 items)
  - Concepts (9: robot namespacing, launch file coordination, multi-robot topics, collision avoidance, leader-follower, fleet coordination, debugging multi-robot systems, RViz visualization, ROS 2 namespaces)
  - Namespacing pattern: `/robot1/cmd_vel`, `/robot2/cmd_vel` (avoid topic collisions)
  - Multi-robot world setup (Gazebo world with 2+ robots at spawn positions)
  - Collision detection: distance-based avoidance (measure distance between robots)
  - Leader-follower: follower tracks leader position via `tf2`
  - Self-assessment (spawn 2 robots, avoid collision, implement leader-follower)
  - 5+ troubleshooting (namespace conflicts, spawn position overlap, TF errors)

- [ ] T053 [P] [US7] Create multi-robot URDF in `examples/chapter-2-gazebo/lesson-08/multi-robot-system.urdf` (single URDF with two differential drive robots, both with sensors)

- [ ] T054 [P] [US7] Create multi-robot world SDF in `examples/chapter-2-gazebo/lesson-08/multi-robot-world.world` (Gazebo world file with ground, lighting, physics, spawn positions for 2 robots)

- [ ] T055 [P] [US7] Create robot 1 URDF variant in `examples/chapter-2-gazebo/lesson-08/robot1.urdf` (namespaced: base_link → robot1/base_link, camera → robot1/camera)

- [ ] T056 [P] [US7] Create robot 2 URDF variant in `examples/chapter-2-gazebo/lesson-08/robot2.urdf` (namespaced: base_link → robot2/base_link, with LiDAR → robot2/lidar)

- [ ] T057 [P] [US7] Create collision avoidance node in `examples/chapter-2-gazebo/lesson-08/collision_avoidance_node.py` (subscribe to both robots' positions, calculate distance, publish stop command if too close):
  - Imports: rclpy, geometry_msgs.msg.Twist, nav_msgs.msg.Odometry
  - Node class with subscribers to `/robot1/odom`, `/robot2/odom`
  - Callback: extract positions, calculate distance, publish stop command if distance < threshold
  - Example: if distance < 0.5m, publish Twist(linear.x=0) to stop robots
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T058 [P] [US7] Create leader-follower node in `examples/chapter-2-gazebo/lesson-08/leader_follower_node.py` (follower tracks leader position using proportional control):
  - Imports: rclpy, nav_msgs.msg.Odometry, geometry_msgs.msg.Twist, math
  - Node class with leader odometry subscriber
  - Follower publisher (sends velocity commands)
  - Callback: extract leader position, calculate error (distance + angle), apply proportional control
  - Publish Twist commands to move follower toward leader
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T059 [P] [US7] Create navigator node in `examples/chapter-2-gazebo/lesson-08/navigator_node.py` (robot1: publish waypoint commands, follow predefined path):
  - Imports: rclpy, geometry_msgs.msg.Twist, nav_msgs.msg.Odometry
  - Node class with odometry subscriber
  - Hardcoded waypoint list: [(0, 0), (1, 0), (1, 1), (0, 1)] (square path)
  - Callback: calculate error to current waypoint, apply proportional control
  - Move to next waypoint when close enough, repeat
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T060 [P] [US7] Create tracker node in `examples/chapter-2-gazebo/lesson-08/tracker_node.py` (subscribe to all robots, collect and log statistics):
  - Imports: rclpy, nav_msgs.msg.Odometry, std_msgs.msg.Float64MultiArray
  - Node class with subscribers to `/robot1/odom`, `/robot2/odom`
  - Callback: track total distance traveled per robot, average velocity, collisions detected
  - Publish statistics on `/fleet_stats` topic
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T061 [P] [US7] Create status reporter node in `examples/chapter-2-gazebo/lesson-08/status_reporter_node.py` (monitor all robots, publish fleet status):
  - Imports: rclpy, nav_msgs.msg.Odometry
  - Node class with multi-robot odometry subscribers
  - Periodically report: robot positions, velocities, distances between robots
  - Publish status messages and log for debugging
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T062 [P] [US7] Create multi-robot launch file in `examples/chapter-2-gazebo/lesson-08/multi-robot.launch.py` (spawn robot1 + robot2 with unique namespaces, start controllers, start coordination nodes):
  - Launch Gazebo with multi-robot world
  - Spawn robot1 in namespace `/robot1` at position (0, 0)
  - Spawn robot2 in namespace `/robot2` at position (2, 2)
  - Start collision_avoidance_node, leader_follower_node, navigator_node, tracker_node
  - Include remappings: /robot1/cmd_vel, /robot2/cmd_vel
  - Marked: "Simulation environment: Gazebo 11+"

- [ ] T063 [P] [US7] Create multi-robot integration test in `tests/ros2-integration/test_multi_robot.py` (pytest: verify robot namespaces are independent, check collision avoidance logic, verify leader-follower convergence)

- [ ] T064 [US7] Create multi-robot debugging guide in `examples/chapter-2-gazebo/lesson-08/multi-robot-debugging.md` (RViz setup for 2+ robots, TF visualization, topic monitoring with ros2 topic echo, common multi-robot issues)

**Checkpoint**: User Story 7 complete - students coordinate multiple robots in simulation

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Validation, documentation, and final integration

- [ ] T065 [P] Create chapter intro file `book-source/docs/chapter-2-gazebo-modeling/intro.md` (if not already created in T014) with chapter overview, prerequisites (Chapter 1), learning paths, success metrics, FAQ

- [ ] T066 [P] Create troubleshooting compilation across all lessons in `book-source/docs/chapter-2-gazebo-modeling/troubleshooting.md` (consolidate all 5+ troubleshooting entries per lesson into indexed reference)

- [ ] T067 [P] Create self-assessment consolidation in `book-source/docs/chapter-2-gazebo-modeling/self-assessment.md` (consolidate all checklists per lesson, enable student self-verification)

- [ ] T068 [P] Create URDF best practices guide in `book-source/docs/chapter-2-gazebo-modeling/urdf-best-practices.md` (link mass, joint limits, sensor placement, common pitfalls)

- [ ] T069 [P] Create Gazebo troubleshooting reference in `book-source/docs/chapter-2-gazebo-modeling/gazebo-troubleshooting.md` (physics instability, plugin errors, world loading, visualization issues)

- [ ] T070 [P] Create ROS 2 control patterns guide in `book-source/docs/chapter-2-gazebo-modeling/ros2-control-patterns.md` (Twist commands, JointTrajectory, state feedback, PID basics)

- [ ] T071 Create capstone project specification in `book-source/docs/chapter-2-gazebo-modeling/capstone-specification.md` (4-node system: navigator + safety_monitor + tracker + status_reporter)

- [ ] T072 Validate all URDF examples load in Gazebo 11+ without errors (test all URDF files: two-link, differential-drive, camera, lidar, imu, multi-sensor, robot1, robot2)

- [ ] T073 Validate all Python code examples compile without syntax errors (test all .py files with `python -m py_compile`)

- [ ] T074 [P] Verify all code examples have "Simulation environment: Gazebo 11+" docstring

- [ ] T075 [P] Verify zero forward references to Isaac Sim, autonomous navigation, or Chapter 3+ topics in all lesson files

- [ ] T076 [P] Verify CEFR B1 compliance: all lessons have 7-10 concepts (not more, not fewer)

- [ ] T077 [P] Verify all lesson markdown files use correct section structure (Learning outcomes, Concepts, Core content, Code examples, Troubleshooting, Self-assessment)

- [ ] T078 [P] Verify all launch files (Python-based) are syntactically correct and can be parsed by ROS 2 launch system

- [ ] T079 [P] Verify all test files (pytest) can discover tests and run without import errors

- [ ] T080 Run Docusaurus build validation (build site, check for broken links, verify all lesson files are linked)

- [ ] T081 Verify all example URDFs have required sections (links with visual/collision, joints with parent/child, gravity settings if in world SDF)

- [ ] T082 Create README for Chapter 2 examples in `examples/chapter-2-gazebo/README.md` (structure overview, how to run each example, lesson-by-lesson guide)

- [ ] T083 Create unit test summary in `tests/README.md` (what tests exist, how to run them, coverage targets)

- [ ] T084 Run constitutional compliance final check (verify simulation-first, no hardware, B1 CEFR, no forward refs, 4-Layer pedagogy)

- [ ] T085 [P] Verify all code examples follow ROS 2 Humble rclpy patterns (imports, Node class, create_publisher/subscription, spin)

- [ ] T086 [P] Verify all URDF examples follow ROS 2 URDF standards (valid link/joint/sensor syntax, correct frame names)

---

## Task Summary

| Phase | Name | Task Count | Purpose |
|-------|------|-----------|---------|
| 1 | Setup | 4 | Project initialization |
| 2 | Foundational | 9 | Core infrastructure (templates, skills, guides) |
| 3 | US1 (P1) | 5 | Gazebo launch & physics understanding |
| 4 | US2 (P1) | 6 | URDF robot modeling |
| 5 | US3 (P1) | 5 | Physics tuning |
| 6 | US4 (P2) | 7 | Sensor integration |
| 7 | US5 (P2) | 7 | Sensor data processing |
| 8 | US6 (P2) | 8 | Robot control via ROS 2 |
| 9 | US7 (P3) | 13 | Multi-robot coordination |
| 10 | Polish | 22 | Validation, documentation, final integration |
| **TOTAL** | **All Phases** | **86 tasks** | **Complete Chapter 2** |

---

## Parallelization Opportunities

### Lesson Content Files (Can run in parallel)
- T015 (Lesson 1) ↔ T019 (Lesson 2) ↔ T025 (Lesson 4) ↔ T030 (Lesson 5) ↔ T044 (Lesson 7)
- These are independent markdown files with different lesson content

### Code Examples by Lesson (Parallelizable within story)
- **US1**: T016-T017 (physics comparison, verification script)
- **US2**: T020-T023 (URDF examples from simple to complex)
- **US4**: T031-T034 (sensor URDFs: camera, LiDAR, IMU, all sensors)
- **US5**: T038-T041 (sensor subscribers: image, LaserScan, IMU, fusion)
- **US6**: T045-T049 (controllers: differential drive, arm, state listener, gripper, PID)
- **US7**: T055-T064 (multi-robot: URDFs, nodes, launch files)

### Validation Tasks (Can run in parallel)
- T072-T086: All validation tasks are independent and can run in parallel across all lesson files

### Example: 3-Developer Parallel Execution Plan
- **Dev 1**: Phase 1-2 (setup + foundation) → US1 lesson + examples (T014-T023)
- **Dev 2**: US4-US5 content + code (T030-T043) in parallel
- **Dev 3**: US6-US7 content + code (T044-T064) in parallel
- **All**: Phase 10 validation (T065-T086) sequentially after content complete

---

## MVP Scope (Minimum Viable Product)

**If time is limited, prioritize in this order**:

### MVP Tier 1: Foundation (P1 - 4 hours)
- T001-T013 (Phase 1-2: setup + foundation)
- T014-T024 (US1 + US2: Gazebo launch + URDF modeling)
- Core deliverable: Students understand Gazebo physics and can write URDF models

### MVP Tier 2: Physics & Sensors (P1 + partial P2 - 8 hours)
- Above + T025-T043 (US3-US5: physics tuning, sensors, data processing)
- Deliverable: Complete perception pipeline (sense → process in ROS 2)

### MVP Tier 3: Complete (Full scope - 10.5 hours)
- All of above + T044-T086 (US6-US7: control, multi-robot, polish)
- Deliverable: Full autonomous robot simulation (sense → process → control → coordinate)

---

## Independent Test Criteria per User Story

- **US1**: Launch Gazebo successfully, observe physics behavior (gravity, friction, collision)
- **US2**: Write URDF for multi-link robot, load in RViz, verify kinematic chain
- **US3**: Tune physics parameters, achieve "move 1m on flat ground" specification
- **US4**: Add camera/LiDAR/IMU to robot, verify ROS 2 topics publish data
- **US5**: Write sensor subscribers, extract features (edges, obstacles, acceleration)
- **US6**: Publish Twist/JointTrajectory commands, observe robot motion in Gazebo
- **US7**: Launch 2+ robots with unique namespaces, implement collision avoidance + leader-follower

---

## Dependencies & Execution Order

**Blocking Dependencies**:
- Phase 1 → Phase 2 (setup required before foundation)
- Phase 2 → All User Story phases (foundation must complete)
- US1 + US2 + US3 → US4 (must understand URDF and physics before sensors)
- US4 → US5 (must add sensors before processing data)
- US5 + US6 → US7 (perception and control needed before multi-robot)

**Independent Execution**:
- US1 ↔ US2 ↔ US3 can start in parallel (once Phase 2 complete)
- US4 ↔ US5 ↔ US6 can start in parallel (once US1-US3 complete)
- Phase 10 validation tasks can start once corresponding content tasks complete

---

**Status**: ✅ TASK GENERATION COMPLETE - READY FOR IMPLEMENTATION
**Branch**: `3-chapter2-gazebo-modeling`
**Total Tasks**: 86 across 10 phases
**MVP Tasks**: 27 (Phases 1-2 + US1-US2 + partial Phase 10)
**Estimated Timeline**: MVP 8 hours, Full chapter 10.5 hours
**Created**: 2025-11-29
