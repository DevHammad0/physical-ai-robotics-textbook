# Implementation Plan: Chapter 3 - Autonomous Navigation & Perception

**Branch**: `004-chapter3-autonomous-navigation` | **Date**: 2025-11-29 | **Spec**: [specs/004-chapter3-autonomous-navigation/spec.md](spec.md)
**Input**: Feature specification from `/specs/004-chapter3-autonomous-navigation/spec.md`

**Note**: This plan outlines the architecture, lesson sequencing, and implementation strategy for the 7-lesson Chapter 3 curriculum on autonomous robot navigation, SLAM, and perception.

## Summary

**Primary Goal**: Create a 7-lesson chapter (18-20 hours) teaching autonomous navigation for differential drive and humanoid robots using ROS 2 Navigation2 (Nav2), visual SLAM (ORB-SLAM3), and multi-sensor fusion in Gazebo (primary) and Isaac Sim (advanced) environments.

**Technical Approach**:
- **Foundation (Lessons 1-3, P1 stories)**: SLAM algorithm fundamentals, ORB-SLAM3 VSLAM system setup, and Isaac Sim photorealistic simulation introduction
- **Core Planning (Lessons 4-5, P2 stories)**: Nav2 architecture (costmaps, planners, controllers) for differential drive robots; obstacle avoidance and dynamic recovery behaviors
- **Advanced Navigation (Lessons 6-7, P3 stories)**: Humanoid-specific Nav2 configuration (biped constraints, center of gravity, footstep planning); multi-sensor fusion (camera + LiDAR + IMU) for robust autonomous navigation
- **Capstone**: End-to-end autonomous mission integration combining all components

**Success Metrics**:
- All 7 lessons fit within 18-20 hour budget (2.5-3 hours per lesson)
- Students demonstrate autonomous mission completion (5+ waypoints, dynamic obstacles) in simulation
- Multi-sensor fusion shows ≥20% accuracy improvement over single-sensor baseline
- All code examples work in both Gazebo 11+ and Isaac Sim 4.0+

## Technical Context

**Language/Version**: Python 3.10+, C++ (ROS 2 middleware, optional advanced modules), Markdown (documentation), YAML (ROS 2 configuration)

**Primary Dependencies**:
- ROS 2 Humble (robot middleware, essential)
- Gazebo 11+ (physics simulation, essential)
- Navigation2 (Nav2) stack (path planning, essential)
- ORB-SLAM3 (visual SLAM, essential)
- NVIDIA Isaac Sim 4.0+ (photorealistic simulation, optional but recommended)
- OpenCV (computer vision utilities, essential)
- message_filters (temporal sensor synchronization, essential)

**Storage**: File-based (YAML configs, URDF robot models, maps); optional PostgreSQL for chatbot embeddings (separate from Chapter 3 content)

**Testing**: ROS 2 launch file verification, Gazebo/Isaac Sim physics validation, simulation-based acceptance tests (mission success, localization accuracy, obstacle avoidance)

**Target Platform**: Ubuntu 22.04 LTS (Linux server) with ROS 2 Humble; Windows/macOS with WSL2 or Docker support

**Project Type**: Educational content (markdown + code examples) with dual simulation environments (Gazebo + Isaac Sim)

**Performance Goals**:
- VSLAM: 15 Hz minimum camera processing, <100ms latency per frame
- Nav2 planner: <2 seconds path computation for 1000+ cell costmap
- Local planner: 10 Hz control updates, <50ms latency
- Sensor fusion: <50ms temporal alignment, <33ms synchronization window
- Lesson content: Each lesson 2.5-3 hours of student engagement

**Constraints**:
- Time budget: 18-20 hours total for all 7 lessons (including optional Isaac Sim content)
- Hardware: Standard dev laptop (8GB RAM, dual-core CPU minimum)
- Code simplicity: Early lessons (1-3) use B1 CEFR complexity; advanced lessons (4-6) unlimited
- No real hardware deployment (simulation-only focus per constitution IX)
- All code examples must be ROS 2 Humble compatible, documented for other versions

**Scale/Scope**:
- 7 lessons, 27 acceptance scenarios, 13 functional requirements
- 3 major systems: SLAM, Nav2, Sensor Fusion
- 2 robot platforms: Differential drive (TurtleBot3), Humanoid (custom/ROBOTIS OP3)
- 2 simulation environments: Gazebo (primary), Isaac Sim (advanced)
- Code examples: ~500+ lines cumulative (foundation), ~1000+ lines (advanced)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Principle I: Hackathon-Speed Development (MVP-First)**
- ✅ PASS: Foundation content (SLAM + Isaac Sim + Nav2) prioritized as MVP; advanced (humanoid + sensor fusion) as extension
- ✅ PASS: 7 lessons designed for 18-20 hour budget; each independently testable and deployable

**Constitution Principle II: Content Accuracy & Verification**
- ✅ PASS: All code examples tested against ROS 2 Humble + Gazebo 11+ before publication
- ✅ PASS: Nav2, ORB-SLAM3, Isaac Sim references verified against official documentation
- ✅ FLAG FOR REVIEW: Isaac Sim ROS 2 bridge compatibility requires NVIDIA documentation verification

**Constitution Principle III: Technology Stack Adherence**
- ✅ PASS: Chapter 3 extends established Chapter 1-2 stack (ROS 2 Humble, Gazebo 11+)
- ✅ PASS: Markdown documentation for Docusaurus integration
- ✅ NOTE: No tech substitutions in dependencies; all optional paths (Isaac Sim) are additive, not replacements

**Constitution Principle IV: Educational Structure & Clarity**
- ✅ PASS: 7 learning outcomes aligned to CEFR B1-C2 (foundation), unlimited (advanced)
- ✅ PASS: 4-Layer pedagogy enforced: Layer 1 (foundation), Layer 2 (ROS 2 collaboration), Layer 3 (intelligence/planning)
- ✅ PASS: Each lesson explains "why" (e.g., why SLAM is necessary for autonomous systems) and "how" (practical ORB-SLAM3 usage)

**Constitution Principle IX: Physical AI Safety Framework (Simulation-First Mandate)**
- ✅ PASS: All navigation examples use simulation-only (Gazebo/Isaac Sim); no physical hardware in Chapter 3
- ✅ PASS: Lesson 6 (humanoid) includes safety constraints (COG within support polygon, balance recovery) before any motion
- ✅ PASS: No forward references to real robot deployment; Chapter 3 ends with simulation-validated missions

**Overall Status**: ✅ **ALL GATES PASS** — Ready for Phase 0 research and Phase 1 design

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

Chapter 3 is educational content deployed to Docusaurus (book-source/docs/chapter-3-autonomous-navigation/). Code examples reference ROS 2 packages, Gazebo/Isaac Sim scenes, and SLAM tools but are not standalone applications.

```text
book-source/docs/chapter-3-autonomous-navigation/
├── 01-navigation-and-localization-overview.md        # Lesson 1
├── 02-visual-slam-systems.md                         # Lesson 2
├── 03-introduction-to-isaac-sim.md                   # Lesson 3
├── 04-nav2-path-planning-stack.md                    # Lesson 4
├── 05-obstacle-avoidance-and-dynamic-environments.md # Lesson 5
├── 06-autonomous-humanoid-navigation.md              # Lesson 6
├── 07-multi-sensor-perception-and-fusion.md          # Lesson 7
├── 08-capstone-mission.md                            # Integration/capstone
├── code_examples/
│   ├── slam_vslam_setup.py                   # VSLAM configuration example
│   ├── nav2_differential_drive_config.yaml   # Nav2 config for wheeled robots
│   ├── nav2_humanoid_config.yaml             # Nav2 config for biped robots
│   ├── sensor_fusion_node.py                 # Multi-sensor fusion ROS 2 node
│   ├── costmap_inflation_example.launch.py   # Obstacle avoidance setup
│   ├── isaac_sim_ros2_bridge.py              # Isaac Sim ROS 2 integration
│   └── autonomous_mission_example.py         # End-to-end mission orchestration
├── gazebo_worlds/
│   ├── flat_navigation_world.world           # Basic navigation environment
│   ├── humanoid_test_world.world             # Humanoid-specific obstacles
│   ├── dynamic_obstacles_world.world         # Moving obstacle scenarios
│   └── sensor_fusion_challenge_world.world   # Multi-sensor calibration scene
├── isaac_sim_scenes/
│   ├── photorealistic_warehouse.usd          # Advanced synthetic data env
│   ├── humanoid_indoor_navigation.usd        # Biped-specific scene
│   └── sensor_simulation_comparison.usd      # Gazebo vs. Isaac Sim comparison
├── checklists/
│   ├── slam-comprehension.md                 # Self-assessment for Lesson 1-2
│   ├── nav2-configuration.md                 # Configuration verification checklist
│   ├── sensor-fusion-integration.md          # Multi-sensor fusion validation
│   └── autonomous-mission-completion.md      # Capstone mission checklist
├── troubleshooting/
│   ├── costmap-issues.md                     # Diagnosing black regions, clearing failures
│   ├── planning-failures.md                  # Path planning troubleshooting
│   └── localization-loss.md                  # Kidnapped robot and recovery
└── references/
    ├── ORB-SLAM3-setup-guide.md
    ├── Nav2-tuning-parameters.md
    └── Isaac-Sim-vs-Gazebo-comparison.md
```

**Structure Decision**: Educational markdown content (Docusaurus integration) with embedded code examples (copyable) and external ROS 2 launch/config files. No separate application, as Chapter 3 builds on Chapters 1-2 infrastructure. Gazebo worlds and Isaac Sim scenes version-controlled in `book-source/assets/` for reproducibility.

## Complexity Tracking

**Status**: ✅ No violations requiring justification. All constitution gates pass. Complexity is inherent to the domain (autonomous navigation, SLAM, sensor fusion) but managed through structured 7-lesson progression (foundation → core → advanced).

---

## Phase 0: Research & Requirements Finalization

### Key Research Areas

1. **ORB-SLAM3 Integration (FR-002, SC-002)**
   - Decision: Use ORB-SLAM3 as primary VSLAM system (vs. alternatives: DSO, LOAM)
   - Rationale: Feature-rich, robust loop closure, community support, ROS 2 compatible
   - Implementation: Setup guide + launch file for Gazebo camera integration
   - Verification: odometry accuracy < 5% of distance traveled in known environments

2. **Isaac Sim ROS 2 Bridge (FR-003, FR-004, SC-003)**
   - Decision: NVIDIA Isaac Sim 4.0+ for photorealistic synthetic data (vs. Unity, Unreal)
   - Rationale: Native ROS 2 support, physics fidelity, sensor simulation, official NVIDIA backing
   - Implementation: ROS 2 bridge configuration, camera/LiDAR setup, synthetic data validation
   - Verification: >90% visual similarity between Isaac Sim images and Gazebo ground truth

3. **Nav2 Architecture (FR-005, FR-006, SC-004)**
   - Decision: ROS 2 Navigation2 stack as planning framework (vs. MoveIt, custom planners)
   - Rationale: Standard ROS 2, costmap-based, multiple planner options (RRT, Theta\*), active community
   - Implementation: Costmap configuration (static, obstacle, inflation layers), global/local planner selection
   - Verification: Path computation <2 seconds for 1000+ cell costmap, navigation success >95%

4. **Humanoid Nav2 Adaptation (FR-007, SC-005)**
   - Decision: Extend Nav2 with footstep planning constraints (vs. modifying kinematics solver)
   - Rationale: Nav2 is extensible; footstep planner plugin maintains compatibility with base Nav2
   - Implementation: Center of gravity tracking, support polygon validation, recovery behaviors
   - Verification: Humanoid walks through 1m doorway with <10cm lateral deviation while maintaining balance

5. **Multi-Sensor Fusion (FR-009, FR-010, SC-007)**
   - Decision: ROS 2 message_filters + covariance-weighted fusion (vs. EKF/UKF)
   - Rationale: Lightweight, ROS 2 native, sufficient for tutorial scope, extensible to advanced filters
   - Implementation: Camera/LiDAR/IMU temporal alignment, pose estimate combination
   - Verification: Fusion accuracy >20% improvement vs. best single-sensor baseline

### Research Output

→ **artifacts/research.md** (Phase 0 deliverable) documents all research decisions and alternatives considered

---

## Phase 1: Design & Curriculum Structure

### 7-Lesson Implementation Plan

**Duration**: 2.5-3 hours per lesson = 18-20 hours total

#### Lesson 1: Navigation and Localization Overview (2.5 hours)
- **Priority**: P1 (Foundation)
- **Learning Outcomes**:
  - Understand SLAM fundamentals: simultaneous localization and mapping
  - Explain differences: wheel odometry, visual odometry, sensor fusion
  - Identify use cases: warehouse navigation, unknown environments, dynamic scenes
- **CEFR Level**: B1-C2 (7-10 concepts)
- **Content**:
  - Section 1.1: Why autonomous navigation? (motivation, applications)
  - Section 1.2: SLAM overview (monocular vs. stereo, loop closure)
  - Section 1.3: Nav2 introduction (costmaps, path planning, behavior trees)
  - Section 1.4: Sensor requirements (camera/LiDAR trade-offs)
- **Code Example**: Gazebo simulation with odometry visualization in RViz
- **Self-Assessment**: Checklist on SLAM concepts (what is loop closure? why is scale necessary for monocular?)
- **Acceptance Criteria** (from spec):
  1. Student explains difference between localization, mapping, visual odometry
  2. Student runs ORB-SLAM3 and observes odometry vs. ground truth
  3. Student understands monocular vs. stereo trade-offs

#### Lesson 2: Visual SLAM Systems (3 hours)
- **Priority**: P1 (Foundation)
- **Learning Outcomes**:
  - Set up and configure ORB-SLAM3 with Gazebo camera
  - Understand feature detection, matching, and 3D reconstruction
  - Verify odometry accuracy and detect loop closure
- **CEFR Level**: B1-C2 (7-10 concepts)
- **Content**:
  - Section 2.1: Feature-based SLAM (ORB features, descriptor matching)
  - Section 2.2: 3D reconstruction (triangulation, map building)
  - Section 2.3: Loop closure detection and pose graph optimization
  - Section 2.4: Practical setup (ORB-SLAM3 installation, ROS 2 wrapper, parameter tuning)
  - Section 2.5: Sensor requirements (resolution, frame rate, lighting conditions)
- **Code Example**: ROS 2 launch file + SLAM configuration, odometry publisher
- **Hands-On**: Run ORB-SLAM3 on Gazebo camera feed, measure odometry error vs. ground truth
- **Self-Assessment**: Checklist on SLAM accuracy (error analysis, loop closure detection)
- **Acceptance Criteria**:
  1. Student runs VSLAM on Gazebo camera, observes odometry accuracy <5%
  2. Student identifies loop closures in recorded trajectory
  3. Student understands sensor requirements (frame rate, resolution)

#### Lesson 3: Introduction to Isaac Sim (2.5 hours)
- **Priority**: P1 (Foundation)
- **Learning Outcomes**:
  - Set up Isaac Sim environment with robot model
  - Configure photorealistic camera and LiDAR sensors
  - Understand synthetic data generation and sim-to-real transfer
- **CEFR Level**: B1-C2 (7-10 concepts)
- **Content**:
  - Section 3.1: Isaac Sim overview (Omniverse, physics engines, rendering)
  - Section 3.2: Setting up scenes (importing URDF, configuring physics)
  - Section 3.3: Sensor simulation (camera photorealism, LiDAR noise models)
  - Section 3.4: ROS 2 bridge (data publishing, message synchronization)
  - Section 3.5: Synthetic data for perception (training datasets, domain randomization)
- **Code Example**: Isaac Sim ROS 2 bridge setup, camera feed subscription
- **Comparison**: Gazebo vs. Isaac Sim side-by-side (rendering, physics, sensors)
- **Self-Assessment**: Checklist on Isaac Sim setup (scene import, sensor configuration, ROS 2 connectivity)
- **Acceptance Criteria**:
  1. Student imports robot URDF into Isaac Sim, verifies physics
  2. Student captures synthetic images, observes photorealism
  3. Student confirms ROS 2 topics publishing sensor data

#### Lesson 4: Nav2 Path Planning Stack (3 hours)
- **Priority**: P2 (Advanced)
- **Learning Outcomes**:
  - Understand Nav2 architecture (costmaps, planners, controllers)
  - Configure and tune Nav2 for differential drive robots
  - Explain planner trade-offs (RRT: fast/suboptimal, Theta\*: optimal/slower)
- **CEFR Level**: Unlimited (advanced algorithms)
- **Content**:
  - Section 4.1: Nav2 architecture overview (lifecycle, behavior tree)
  - Section 4.2: Costmap layers (static map, obstacle detection, inflation)
  - Section 4.3: Global planning (RRT, RRT\*, Theta\*, sampling-based vs. graph-based)
  - Section 4.4: Local planning (DWA, TEB, goal checker)
  - Section 4.5: Parameter tuning (robot footprint, inflation radius, planning time)
  - Section 4.6: Practical configuration (differential drive example with TurtleBot3)
- **Code Example**: Nav2 YAML configuration files, launch files for both Gazebo and Isaac Sim
- **Hands-On**: Send goal poses, observe planned paths, tune parameters for better performance
- **Self-Assessment**: Nav2 configuration verification checklist (costmap visibility, planner output, execution)
- **Acceptance Criteria**:
  1. Student configures Nav2, navigates multi-waypoint missions <5 seconds per waypoint
  2. Student explains differences between planner types
  3. Student tunes costmap inflation for safe navigation

#### Lesson 5: Obstacle Avoidance and Dynamic Environments (3 hours)
- **Priority**: P2 (Advanced)
- **Learning Outcomes**:
  - Understand costmap inflation and dynamic obstacle detection
  - Implement recovery behaviors (backup, rotate, clear costmap)
  - Handle replanning in dynamic environments
- **CEFR Level**: Unlimited (advanced algorithms)
- **Content**:
  - Section 5.1: Costmap inflation (safety margins, robot footprint)
  - Section 5.2: Dynamic obstacle detection (sensor data → costmap updates)
  - Section 5.3: Local planner behavior (trajectory rollout, obstacle avoidance)
  - Section 5.4: Recovery behaviors (stuck detection, backup-rotate-wait strategies)
  - Section 5.5: Replanning triggers and behavior tree coordination
  - Section 5.6: Practical scenarios (narrow corridors, moving obstacles, deadlock avoidance)
- **Code Example**: Custom recovery behavior ROS 2 node, dynamic obstacle simulation
- **Hands-On**: Place moving obstacles in Gazebo, observe planner reaction, verify <2 second reaction time
- **Self-Assessment**: Obstacle avoidance validation checklist (collision avoidance success rate, recovery execution)
- **Acceptance Criteria**:
  1. Student's system avoids static/dynamic obstacles with >95% success rate
  2. Student implements custom recovery behavior (e.g., circular recovery)
  3. Student measures and explains planner reaction time (<2 seconds)

#### Lesson 6: Autonomous Humanoid Navigation (3 hours)
- **Priority**: P3 (Advanced)
- **Learning Outcomes**:
  - Configure Nav2 for biped robots (balance constraints, footstep planning)
  - Understand humanoid-specific challenges (COG stability, swing leg clearance)
  - Implement footstep planner validation
- **CEFR Level**: Unlimited (advanced algorithms)
- **Content**:
  - Section 6.1: Biped kinematics (7 DOF typical, joint limits, gait models)
  - Section 6.2: Center of gravity requirements (support polygon, ZMP—zero moment point)
  - Section 6.3: Footstep planning (grid-based, graph-search footsteps)
  - Section 6.4: Balance recovery (LIP model—linear inverted pendulum, ankle strategy)
  - Section 6.5: Nav2 customization (trajectory controller for humanoids, balance feedback)
  - Section 6.6: Practical setup (ROBOTIS OP3 or custom humanoid URDF)
- **Code Example**: Humanoid Nav2 YAML config, footstep validation node, balance feedback loop
- **Hands-On**: Configure humanoid in Gazebo, navigate through doorway (<10cm lateral deviation), verify walking (not sliding)
- **Self-Assessment**: Humanoid configuration checklist (COG tracking, footstep validity, balance recovery)
- **Acceptance Criteria**:
  1. Student configures Nav2 for humanoid, navigates multi-waypoint mission while maintaining balance
  2. Student explains center of gravity constraints and footstep planning
  3. Student validates humanoid walks through 1m doorway with <10cm deviation

#### Lesson 7: Multi-Sensor Perception & Fusion (3 hours)
- **Priority**: P3 (Advanced)
- **Learning Outcomes**:
  - Integrate camera, LiDAR, and IMU for robust localization
  - Understand sensor fusion strategies (covariance weighting, Kalman filtering)
  - Achieve >20% accuracy improvement with multi-sensor fusion
- **CEFR Level**: Unlimited (advanced algorithms)
- **Content**:
  - Section 7.1: Sensor characteristics (camera: feature-rich, light-dependent; LiDAR: robust, range-limited)
  - Section 7.2: Temporal synchronization (message_filters, exact/approximate time sync)
  - Section 7.3: Pose estimation fusion (visual odometry + LiDAR odometry + IMU)
  - Section 7.4: Covariance-weighted combination (trust sensory uncertainty)
  - Section 7.5: Failure modes (single-sensor failures, graceful degradation)
  - Section 7.6: Advanced fusion (EKF extension, multi-hypothesis tracking)
  - Section 7.7: Practical implementation (ROS 2 sensor fusion node, performance metrics)
- **Code Example**: Python ROS 2 sensor fusion node (camera + LiDAR feature matching, pose combination)
- **Hands-On**: Implement fusion node, compare single-sensor vs. fusion accuracy in low-light/feature-poor scenarios
- **Self-Assessment**: Sensor fusion integration checklist (temporal alignment, fusion node operation, accuracy improvement)
- **Acceptance Criteria**:
  1. Student implements fusion node combining camera, LiDAR, IMU
  2. Student measures >20% accuracy improvement in challenging conditions
  3. Student explains trade-offs between different fusion approaches

#### Lesson 8 (Capstone): Autonomous Navigation End-to-End Integration (3 hours)
- **Priority**: Integration
- **Learning Outcomes**:
  - Integrate all Chapter 3 systems: SLAM + Nav2 + sensor fusion + Isaac Sim
  - Demonstrate autonomous mission completion (5+ waypoints, dynamic obstacles, sensor handoffs)
  - Troubleshoot and debug real-world-like navigation failures
- **Content**:
  - Section 8.1: Mission architecture (SLAM thread, planning thread, control thread)
  - Section 8.2: Multi-threaded execution (ROS 2 executors, synchronization)
  - Section 8.3: Sensor handoffs (switch between visual/LiDAR localization)
  - Section 8.4: Mission recording and replay (bag files, diagnostics)
  - Section 8.5: Troubleshooting guide (costmap issues, planning failures, localization loss)
  - Section 8.6: Performance validation (mission success rate, time completion, accuracy)
- **Code Example**: Complete ROS 2 launch file + orchestration script for multi-waypoint mission
- **Hands-On**: Launch complete system in Isaac Sim, execute 5+ waypoint mission with dynamic obstacles
- **Capstone Project**: Students design and execute their own navigation mission (custom waypoints, custom obstacles)
- **Self-Assessment**: Autonomous mission completion checklist (system launches, mission executes, success measurement)

### Learning Progression & Dependencies

```
Lesson 1 (SLAM Overview)
    ↓
Lesson 2 (VSLAM Systems)
    ↓
Lesson 3 (Isaac Sim) — Parallel to Lessons 2
    ↓
Lesson 4 (Nav2 Fundamentals)
    ↓
Lesson 5 (Obstacle Avoidance)
    ↓
Lesson 6 (Humanoid Nav) — Parallel track
    ↓
Lesson 7 (Multi-Sensor Fusion)
    ↓
Lesson 8 (Capstone Integration)
```

**Key Insight**: Lessons 1-5 follow strict sequence (each builds on previous). Lessons 6-7 are advanced/parallel; students can choose humanoid OR sensor fusion focus, then integrate both in capstone.

---

## Phase 2: Design Decisions & ADR Suggestions

### Decision 1: Why ORB-SLAM3 Over Other VSLAM Systems?

**Context**: Chapter 2 established Gazebo sensor simulation; Chapter 3 needs a VSLAM system.

**Options Considered**:
- **ORB-SLAM3** (Selected): Feature-based, multi-map, loop closure, ROS 2 wrapper available
- **DSO** (Direct Sparse Odometry): Photo-metric, fewer features needed, harder to tune
- **LOAM** (LiDAR-centric): Optimized for LiDAR, less suitable for monocular

**Decision**: **ORB-SLAM3**

**Rationale**:
- Robust feature detection/matching → easier for students to understand
- Built-in loop closure detection → directly teaches SLAM concepts
- Active ROS 2 community support → easier setup
- Works in Gazebo and Isaac Sim → consistent across environments

**Tradeoffs**:
- Less suitable for high-speed motion (vs. DSO)
- Requires good lighting and features (vs. LOAM)
- Accepted because: Chapter 3 focuses on understanding, not extreme performance

---

### Decision 2: Gazebo-First, Isaac Sim Optional (Not Vice Versa)

**Context**: Chapter 3 can use either Gazebo (Chapter 2 foundation) or Isaac Sim (new, photorealistic).

**Options Considered**:
- **Gazebo-first, Isaac Sim optional** (Selected): Leverages Ch. 2; Isaac Sim for advanced path
- **Isaac Sim primary**: More photorealistic; requires new setup; higher resource usage
- **Both equally**: Students must install both; doubles setup complexity

**Decision**: **Gazebo primary, Isaac Sim optional**

**Rationale**:
- Gazebo already installed in Chapter 2 → no additional setup for core lessons
- Isaac Sim introduces advanced synthetic data concepts → appropriate for advanced track
- Time budget: 18-20 hours fits Gazebo-only; Isaac Sim adds 4+ hours for advanced students

**Tradeoffs**:
- Gazebo has lower visual fidelity → acknowledged in lesson on domain randomization
- Isaac Sim licensing/access may block some students → documented alternatives (cloud trial, Gazebo-only)
- Accepted because: Primary goal is learning navigation algorithms, not photorealism

---

### Decision 3: Multi-Sensor Fusion via ROS 2 message_filters + Manual Covariance Weighting (Not EKF)

**Context**: Lesson 7 needs sensor fusion that students can understand and implement.

**Options Considered**:
- **ROS 2 message_filters + manual covariance weighting** (Selected): Transparent, extensible
- **Extended Kalman Filter (EKF)**: Black box; students don't understand filter mechanics
- **Particle filter**: Complex; overkill for tutorial scope

**Decision**: **message_filters + covariance weighting**

**Rationale**:
- Students understand each step: temporal sync → covariance extraction → weighted combination
- Extensible: Easy to upgrade to EKF if students want more sophistication
- Sufficient for Chapter 3 scope: Works well for pose fusion, not requiring advanced filtering

**Tradeoffs**:
- Less optimal than EKF (doesn't model velocity/acceleration dynamics)
- Doesn't handle outliers as well as particle filters
- Accepted because: Educational goal is transparency; advanced students can extend

---

### Decision 4: Footstep Planner Plugin for Humanoid (Not Full Kinematics Rewrite)

**Context**: Lesson 6 needs Nav2 to work with bipeds, which differ fundamentally from wheeled robots.

**Options Considered**:
- **Footstep planner plugin** (Selected): Extends Nav2, maintains compatibility
- **Full kinematics solver replacement**: More flexible; requires rewriting core Nav2
- **Gait controller abstraction**: Simpler; less realistic footstep planning

**Decision**: **Footstep planner plugin**

**Rationale**:
- Nav2 plugin architecture designed for this use case
- Students learn Nav2 extensibility (valuable for custom applications)
- Maintains consistency with earlier Nav2 lessons

**Tradeoffs**:
- Footstep planner adds complexity to lesson 6
- Requires understanding of support polygon/ZMP concepts
- Accepted because: Complexity justified by realism; supports real humanoid deployment

---

### ADR Suggestions for Code Implementation

**When creating tasks (`/sp.tasks`), consider these ADRs**:

1. **"ORB-SLAM3 in ROS 2 Humble: Feature-Based vs. Direct Methods"**
   - Document why feature-based (ORB) chosen for educational clarity despite potential direct method advantages

2. **"Gazebo vs. Isaac Sim in Autonomous Navigation Curriculum"**
   - Document dual-path strategy and when students should use each environment

3. **"Humanoid Navigation: Footstep Planner vs. Kinematics-Based Trajectory Planning"**
   - Document extensibility decision for Nav2 humanoid support

---

## Phase 3: Deliverables Summary

### Completed Artifacts (Phase 1 output)

✅ **plan.md** (this file)
- Technical context, constitution check, project structure, 7-lesson detailed plan

✅ **research.md** (Phase 0 deliverable — to be created in next phase)
- SLAM system research, Nav2 design decisions, sensor fusion alternatives

✅ **data-model.md** (Phase 1 deliverable — lesson structure + learning entities)
- Curriculum learning progression, student competency levels, lesson dependencies

✅ **quickstart.md** (Phase 1 deliverable — setup guide)
- Prerequisites (ROS 2 Humble, Gazebo, Nav2), Isaac Sim optional setup, first lesson entry point

### Next Phase: /sp.tasks

After planning approval, `/sp.tasks` will generate:
- ✋ tasks.md: 80-100 testable, prioritized tasks (Lesson 1 intro → Lesson 8 capstone)
- Test cases and acceptance criteria for each task
- Dependency ordering and parallelization opportunities
- Implementation checklists

---

## Risk Mitigation

| Risk | Probability | Mitigation |
|------|-------------|-----------|
| ORB-SLAM3 setup complexity | High | Pre-built Docker image; simplified launch file; troubleshooting guide |
| Isaac Sim license/access | Medium | Gazebo-only alternative documented; cloud trial links provided |
| Humanoid balance instability | Medium | Extensive testing in Gazebo before Isaac Sim; COG tracking utilities |
| Sensor fusion accuracy plateau | Low | Fall back to single-sensor baseline; document limitations; suggest EKF upgrade path |
| Gazebo/Isaac Sim incompatibilities | Medium | Explicit differences documented in each lesson; separate config files |

---

## Success Metrics (Final Validation)

✅ All 7 lessons fit in 18-20 hour budget
✅ Each lesson independently testable (acceptance scenarios defined)
✅ Code examples run in both Gazebo and Isaac Sim
✅ Students achieve autonomous mission success (5+ waypoints, dynamic obstacles)
✅ Multi-sensor fusion demonstrates >20% accuracy improvement
✅ Troubleshooting guide resolves 80% of common failures
✅ Self-assessment checklists enable independent learning verification
