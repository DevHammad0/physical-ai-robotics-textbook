# Implementation Tasks: Chapter 4 - AI Integration & Autonomous Systems

**Feature**: Chapter 4 - Vision-Language-Action (VLA) Autonomous Pipeline
**Branch**: `005-chapter4-ai-integration`
**Total Tasks**: 92
**Expected Duration**: 18-20 weeks (8 lessons × 18-20 hours)
**Test Approach**: Simulation-based acceptance tests (Isaac Sim)

---

## Implementation Strategy

**MVP Scope**: 4 P1 user stories (voice-controlled perception, LLM planning, manipulation, full-stack integration)
**Incremental Delivery**: Lesson-by-lesson (Lesson 1 → Lesson 8), each independently testable
**Parallelization Opportunities**: Within each lesson, nodes can be implemented in parallel (voice, vision, planning parallel to each other)
**Independent Test Criteria**: Each lesson has end-to-end scenario that can be tested independently

---

## Phase 1: Setup & Infrastructure (Weeks 1-2)

### Goal
Establish Docusaurus chapter structure, ROS 2 package scaffolding, and Isaac Sim environment

### Independent Test
Documentation structure created, empty lesson files ready, example ROS 2 packages initialized, Isaac Sim scenes loadable

---

#### Setup Tasks

- [ ] T001 Create book-source/docs/03-chapter-4-ai-integration/ directory structure and 00-intro.md file with chapter overview and 4-layer pedagogy explanation
- [ ] T002 Create empty lesson markdown files (01-lesson-1-vla-architecture.md through 08-lesson-8-capstone.md) with learning objectives placeholders and code section stubs
- [ ] T003 Create assessments/ directory with lesson rubric file stubs (lesson1-rubric.md through lesson8-rubric.md, capstone-rubric.md)
- [ ] T004 Initialize ros2_packages/ directory with 6 ROS 2 package scaffolds (chapter4_voice, chapter4_vision, chapter4_planning, chapter4_manipulation, chapter4_safety, chapter4_integration)
- [ ] T005 Create package.xml and setup.py for each ROS 2 package with correct dependencies (rclpy, sensor_msgs, std_msgs, etc.)
- [ ] T006 [P] Create Isaac Sim scenes directory (isaac_sim_scenes/) and design 3 scene YAML files (scene_1_simple_table.yaml, scene_2_manipulation_task.yaml, scene_3_complex_scene.yaml) with object definitions
- [ ] T007 [P] Create quickstart.md with environment setup instructions (Isaac Sim, ROS 2 Humble, Python 3.10+, OpenAI API key or Ollama)
- [ ] T008 Create troubleshooting.md guide with common issues (microphone not detected, API key invalid, Isaac Sim connection failed, etc.)
- [ ] T009 Create README.md for chapter with links to all lessons, assessment rubrics, and Isaac Sim scenes
- [ ] T010 [P] Set up GitHub Actions CI/CD to build Docusaurus, validate code examples, run ROS 2 package checks on every commit

---

## Phase 2: Foundational Components (Weeks 3-4)

### Goal
Implement shared ROS 2 message types, utility libraries, and simulation test harness

### Independent Test
ROS 2 messages compilable, utility functions testable, simulation test framework can spawn Isaac Sim scenes

---

#### Foundational Tasks

- [ ] T011 [P] Create ROS 2 message definitions (msg/) for all data structures: VoiceCommand.msg, DetectionResult.msg, DetectionArray.msg, TaskPlan.msg, GraspCandidate.msg, ExecutionStatus.msg, SafetyEvent.msg, RobotState.msg
- [ ] T012 [P] Create ROS 2 service definitions (srv/) for planning and validation: PlanTask.srv, ValidatePlan.srv, GenerateGrasps.srv
- [ ] T013 [P] Create ROS 2 action definitions (action/) for execution and motion: ExecuteTask.action, MoveArm.action
- [ ] T014 Create shared utility library (chapter4_common/utils.py) with helper functions: ROS 2 message conversion, Isaac Sim scene loading, logging utilities, error codes
- [ ] T015 Create shared configuration module (chapter4_common/config.py) with parameters: voice timeout (5s), voice confidence threshold (0.7), safety thresholds (force limits, workspace bounds), timeouts
- [ ] T016 [P] Create Isaac Sim test harness (chapter4_integration/sim_test_harness.py) for launching scenes, spawning robots, capturing sensor data, validating physics
- [ ] T017 Create ROS 2 launch file utilities (chapter4_integration/launch_utils.py) for composing launch files, managing nodes, handling startup/shutdown
- [ ] T018 Create documentation (ARCHITECTURE.md) describing ROS 2 node interfaces, topic/service contracts, message flow diagrams
- [ ] T019 Create GitHub Actions workflow (.github/workflows/test.yml) to run ROS 2 linter (ament_lint), compile messages, test utilities

---

## Phase 3: User Story 1 - Voice-Controlled Perception System (Weeks 5-7)

### Story Goal
Students learn to set up speech recognition (OpenAI Whisper) in ROS 2, integrate with object detection and semantic segmentation, create closed-loop perception pipeline

### User Stories Included
- **US1**: Build Voice-Controlled Perception System (P1)

### Independent Test
"Say 'find the red cube', robot detects objects via camera, responds with identified objects. Perception pipeline runs end-to-end in <5 seconds."

### Acceptance Criteria
- Voice transcription working with ≥90% accuracy on command vocabulary
- Object detection returns bounding boxes + confidence scores
- Semantic segmentation produces pixel-level labels
- System handles missing objects gracefully (requests clarification)

---

#### Lesson 1: VLA Architecture Overview (4-6 hours, 0 new code)

- [ ] T020 [US1] Write Lesson 1 markdown (01-lesson-1-vla-architecture.md) with learning objectives, VLA pipeline overview, real-world examples (Boston Dynamics, Tesla, OpenAI Robotics), comparison with traditional robot programming, 4-layer pedagogy applied to VLA
- [ ] T021 [US1] Create VLA architecture diagram (book-source/docs/03-chapter-4-ai-integration/images/vla_pipeline.svg) showing voice → perception → planning → action flow
- [ ] T022 [US1] Create real-world use case examples (book-source/docs/03-chapter-4-ai-integration/examples/use_cases.md): home robotics, manufacturing, logistics with scenarios
- [ ] T023 [US1] Write self-assessment rubric for Lesson 1 (assessments/lesson1-rubric.md) covering VLA architecture understanding, real-world application thinking, 4-layer pedagogy comprehension

---

#### Lesson 2: Voice Input with OpenAI Whisper (4-6 hours)

- [ ] T024 [US1] [P] Write Lesson 2 markdown (02-lesson-2-voice-whisper.md) with learning objectives, Whisper architecture, real-time vs batch processing tradeoffs, ROS 2 node design patterns, error handling, real hardware considerations
- [ ] T025 [US1] [P] Create Whisper setup guide (book-source/docs/03-chapter-4-ai-integration/02-lesson-2-voice-whisper/setup-whisper.md) with: pip install openai-whisper, model selection (base vs small vs medium), CPU vs GPU inference, audio device configuration
- [ ] T026 [US1] [P] Implement whisper_ros2_node.py (~100 lines) in chapter4_voice/src/: subscribes to audio stream, calls Whisper, publishes transcribed text to /robot/voice_command topic with confidence scores, logs all commands with timestamps
- [ ] T027 [US1] [P] Implement whisper_ros2_node_test.py (~50 lines) with pytest: tests successful transcription, confidence score validation, timeout handling, error logging
- [ ] T028 [US1] Create whisper setup script (chapter4_voice/setup-whisper.sh) automating pip install, model download, audio device detection
- [ ] T029 [US1] Write Lesson 2 exercise (02-lesson-2-voice-whisper/exercises/ex1-transcribe.md): "Set up Whisper node, say voice commands, verify transcription, adjust confidence threshold"
- [ ] T030 [US1] Create Lesson 2 troubleshooting section in 02-lesson-2-voice-whisper.md: microphone not detected, Whisper model download fails, timeout issues, API key errors (if using API)
- [ ] T031 [US1] Write self-assessment rubric for Lesson 2 (assessments/lesson2-rubric.md) covering Whisper setup, ROS 2 node design, error handling, real hardware awareness
- [ ] T032 [US1] Create Lesson 2 code example (02-lesson-2-voice-whisper/code/) with: whisper_ros2_node.py, setup-whisper.sh, requirements.txt (openai-whisper, rclpy, etc.)

---

#### Lesson 3: Vision Systems for Robotics (4-6 hours)

- [ ] T033 [US1] [P] Write Lesson 3 markdown (03-lesson-3-vision-systems.md) with learning objectives, object detection models (YOLO-v8, Detectron), semantic segmentation models (FCN, DeepLab), depth camera integration, tradeoffs, real hardware considerations
- [ ] T034 [US1] [P] Create object detection guide (03-lesson-3-vision-systems/guides/object_detection.md): YOLO-v8 installation, model selection, inference optimization, batch processing, confidence thresholding
- [ ] T035 [US1] [P] Implement yolo_detector_node.py (~150 lines) in chapter4_vision/src/: subscribes to /robot/camera/image_raw (Isaac Sim camera), runs YOLO-v8 inference, publishes detections to /robot/detections topic with bounding boxes, class labels, confidence scores
- [ ] T036 [US1] [P] Implement segmentation_node.py (~150 lines) in chapter4_vision/src/: subscribes to camera image, runs semantic segmentation (FCN or DeepLab), publishes segmentation mask to /robot/segmentation topic with class labels
- [ ] T037 [US1] [P] Implement yolo_detector_test.py and segmentation_test.py (~50 lines each) with pytest: test inference on sample images, validate bounding box formats, check segmentation mask dimensions
- [ ] T038 [US1] Create object detection fine-tuning guide (03-lesson-3-vision-systems/guides/fine_tuning.md): how to fine-tune YOLO on custom objects, create training dataset, validate on test set
- [ ] T039 [US1] Write Lesson 3 exercise (03-lesson-3-vision-systems/exercises/ex2-detect-objects.md): "Run object detection on Isaac Sim scene, tune confidence threshold, identify all objects in scene, compare detection vs ground truth"
- [ ] T040 [US1] Create Lesson 3 code examples (03-lesson-3-vision-systems/code/): yolo_detector_node.py, segmentation_node.py, sample_detector_usage.py (standalone usage without ROS 2)
- [ ] T041 [US1] Write Lesson 3 real hardware section (03-lesson-3-vision-systems.md): "Differences from simulation: lighting variation, occlusion, object size range affect real camera; simulation assumes ideal lighting and known object dimensions"
- [ ] T042 [US1] Write self-assessment rubric for Lesson 3 (assessments/lesson3-rubric.md) covering detection accuracy, segmentation quality, real hardware awareness
- [ ] T043 [US1] Create integration test (chapter4_vision/tests/integration_test_vision.py): launch Isaac Sim scene, spawn robot with camera, run detection + segmentation nodes, verify /robot/detections and /robot/segmentation topics publish within timeout

---

#### Completion Test for US1
- [ ] T044 Create end-to-end test scenario (chapter4_vision/tests/e2e_voice_perception.py): launch Lesson 2 (Whisper) + Lesson 3 (Vision) nodes, simulate voice command "find red cube", verify transcription published, verify detection published with red cube in results, verify execution time <5 seconds

---

## Phase 4: User Story 2 - LLM-Driven Planning & Task Decomposition (Weeks 8-10)

### Story Goal
Students integrate LLM (GPT-4 or Ollama) to translate high-level commands into executable robot plans, handle replanning when observations conflict expectations

### User Stories Included
- **US2**: Implement LLM-Driven Planning & Task Decomposition (P1)

### Independent Test
"Given 'pick up the cube and place on shelf', system produces executable action sequence (move → approach → grasp → lift → move to shelf → place), replan if grasp fails"

### Acceptance Criteria
- LLM planner successfully decomposes 90%+ of common commands
- Generated plans are valid (respect workspace, joint limits, gripper feasibility)
- Replanning triggered on execution failure
- Safety checker blocks unsafe plans

---

#### Lesson 4: LLM-Driven Task Planning (4-6 hours)

- [ ] T045 [US2] [P] Write Lesson 4 markdown (04-lesson-4-llm-planning.md) with learning objectives, prompt engineering for robotics, function calling / tool use, task representation (goals, subtasks, preconditions), error recovery and replanning, real hardware latency considerations
- [ ] T046 [US2] [P] Create prompt engineering guide (04-lesson-4-llm-planning/guides/prompt_engineering.md): principles for robot task planning prompts, context window management, output format specification (JSON task plans), few-shot examples, constraint specification
- [ ] T047 [US2] [P] Implement llm_planner_node.py (~200 lines) in chapter4_planning/src/: subscribes to /robot/voice_command and /robot/task_status, calls OpenAI GPT-4 API (or local Ollama) with robot state context, publishes TaskPlan to /robot/task_plan topic, includes retry logic and error handling
- [ ] T048 [US2] [P] Create prompt_templates.py (~100 lines) in chapter4_planning/src/: defines prompt templates for task decomposition, includes examples of safe decompositions, constraint templates (avoid obstacles, respect limits), few-shot examples
- [ ] T049 [US2] [P] Implement llm_planner_test.py (~50 lines) with pytest: test prompt generation, mock GPT-4 API calls, validate plan parsing, test error recovery
- [ ] T050 [US2] Create LLM provider configuration (chapter4_planning/llm_config.yaml): OpenAI API key setup, model selection (gpt-4, gpt-3.5-turbo), alternative Ollama configuration with local model path
- [ ] T051 [US2] Write Lesson 4 exercise (04-lesson-4-llm-planning/exercises/ex3-decompose-tasks.md): "Design prompt for simple task, call GPT-4 with prompt, parse response, verify plan is executable"
- [ ] T052 [US2] Create Lesson 4 code examples (04-lesson-4-llm-planning/code/): llm_planner_node.py, prompt_templates.py, example_gpt_calls.py (standalone test script)
- [ ] T053 [US2] Write Lesson 4 real hardware section: "LLM latency on real robot: GPT-4 API ~2-5 seconds, Ollama local ~1-10 seconds depending on model and GPU; consider real-time constraints"
- [ ] T054 [US2] Write self-assessment rubric for Lesson 4 (assessments/lesson4-rubric.md) covering prompt design, LLM integration, error handling, replanning logic

---

#### Completion Test for US2
- [ ] T055 Create end-to-end test scenario (chapter4_planning/tests/e2e_voice_planning.py): integrate Lessons 2, 3, 4: voice command → transcription → LLM planning → task plan generation, verify plan is valid (no constraint violations)

---

## Phase 5: User Story 3 - Manipulation & Grasping (Weeks 11-13)

### Story Goal
Students implement grasp planning and motion planning for manipulation, validate grasp stability in Isaac Sim physics

### User Stories Included
- **US3**: Execute Manipulation & Grasping in Simulation (P1)

### Independent Test
"Pick up cube from table, move 1 meter, place on shelf—physics validated (object doesn't slip), grasp successful"

### Acceptance Criteria
- Grasp planner generates 3+ candidates with success probability
- Motion planner produces collision-free trajectories <1 second
- Grasp stability validated via force/torque sensors
- Gripper force limited (no over-gripping)

---

#### Lesson 5: Manipulation & Grasping (4-6 hours)

- [ ] T056 [US3] [P] Write Lesson 5 markdown (05-lesson-5-manipulation.md) with learning objectives, grasp planning algorithms, grasp metrics, motion planning (RRT, trajectory optimization), force/torque monitoring, grasp validation, failure detection and recovery, real hardware friction and sensor noise considerations
- [ ] T057 [US3] [P] Create grasp planning guide (05-lesson-5-manipulation/guides/grasp_planning.md): grasp quality metrics, candidate generation, success probability estimation, handling objects of different shapes
- [ ] T058 [US3] [P] Implement grasp_planner_node.py (~150 lines) in chapter4_manipulation/src/: receives object pose from perception, generates grasp candidates using simple heuristics (top, side, front grasps), estimates success probability, publishes GraspCandidates to /robot/grasp_candidates topic
- [ ] T059 [US3] [P] Implement motion_planner_node.py (~200 lines) in chapter4_manipulation/src/: receives grasp target, generates collision-free trajectory from current end-effector pose to grasp pose via simple linear interpolation in joint space (or advanced RRT if MoveIt 2 available), validates joint limits and collision constraints, publishes trajectory to /robot/trajectory topic
- [ ] T060 [US3] [P] Implement grasp_validation_node.py (~100 lines) in chapter4_manipulation/src/: monitors gripper force/torque during grasp execution, validates grasp stability (object doesn't slip), detects grasp failure and triggers replanning
- [ ] T061 [US3] [P] Implement manipulation tests (chapter4_manipulation/tests/): grasp_planner_test.py, motion_planner_test.py, grasp_validation_test.py with unit + integration tests
- [ ] T062 [US3] Create Lesson 5 exercise (05-lesson-5-manipulation/exercises/ex4-pick-place.md): "Generate grasps for cube, plan motion, execute in Isaac Sim, validate physics (object stable)"
- [ ] T063 [US3] Create Lesson 5 code examples: grasp_planner_node.py, motion_planner_node.py, example_pick_place.py (complete scenario)
- [ ] T064 [US3] Write Lesson 5 real hardware section: "Actual gripper dynamics, object friction, sensor noise: simulation assumes perfect physics; real robots have stiction, deadbands, elastic deformation"
- [ ] T065 [US3] Write self-assessment rubric for Lesson 5 (assessments/lesson5-rubric.md) covering grasp quality, trajectory planning, force control, stability validation

---

#### Completion Test for US3
- [ ] T066 Create end-to-end manipulation test (chapter4_manipulation/tests/e2e_pick_place.py): launch Isaac Sim, spawn robot, object on table, run grasp planner, motion planner, execute in sim, validate object moved to target location

---

## Phase 6: User Story 4 - Full-Stack Integration (Weeks 14-16)

### Story Goal
Integrate voice → perception → planning → manipulation into unified closed-loop autonomous system

### User Stories Included
- **US4**: Integrate Voice→Perception→Planning→Manipulation Pipeline (P1)

### Independent Test
"Say 'organize the table', robot perceives objects, plans strategy, manipulates objects, reports completion"

### Acceptance Criteria
- End-to-end latency <10 seconds per subtask
- Error attribution clear (which component failed)
- Replanning on execution failure
- Safety checks block unsafe commands

---

#### Lesson 6: Full-Stack Integration in Isaac Sim (4-6 hours)

- [ ] T067 [US4] [P] Write Lesson 6 markdown (06-lesson-6-full-integration.md) with learning objectives, orchestration architectures (state machine, behavior tree, action server), topic/service interface design, asynchronous error handling, testing and validation strategies, sim-to-real transfer challenges, domain randomization discussion
- [ ] T068 [US4] [P] Implement orchestrator_node.py (~250 lines) in chapter4_integration/src/: orchestrates entire pipeline (voice → perception → planning → manipulation → observation), implements state machine (idle → listening → perceiving → planning → executing → observing → completed/failed), publishes /robot/task_status updates, handles transitions and error cases
- [ ] T069 [US4] [P] Create launch file (chapter4_integration/launch/vla_pipeline.launch.py, ~50 lines): launches all nodes in correct order (perception nodes first, then planning, then manipulation, then orchestrator and safety), configures parameters and remappings
- [ ] T070 [US4] [P] Implement task_status_monitor.py (~100 lines): subscribes to all component status topics (/robot/voice_command, /robot/detections, /robot/task_plan, /robot/execution_status), logs execution trace, detects failures
- [ ] T071 [US4] Implement error_handler.py (~100 lines): centralizes error handling, provides user feedback (text/audio), triggers safety node on critical errors, implements retry logic
- [ ] T072 [US4] Write Lesson 6 exercise (06-lesson-6-full-integration/exercises/ex5-end-to-end.md): "Run full pipeline, say task command, observe execution, debug failures using task status monitor"
- [ ] T073 [US4] Create Lesson 6 code examples: orchestrator_node.py, launch file, example_e2e_scenario.py (complete scenario)
- [ ] T074 [US4] Write Lesson 6 real hardware section: "Sim-to-real: domain randomization (lighting, textures, object dimensions, physics parameters) critical; latency changes (real sensor processing slower); network communication overhead"
- [ ] T075 [US4] Write self-assessment rubric for Lesson 6 (assessments/lesson6-rubric.md) covering orchestration design, asynchronous coordination, error handling, testing strategies
- [ ] T076 [US4] Create integration test (chapter4_integration/tests/e2e_full_pipeline.py): end-to-end scenario with all nodes, voice command → completion or failure, verify all status updates published, validate execution time

---

#### Completion Test for US4
- [ ] T077 Create comprehensive end-to-end test suite (chapter4_integration/tests/): multiple scenarios (simple pick, complex manipulation, error cases), all run in <20 seconds per scenario, all pass physics validation in Isaac Sim

---

## Phase 7: User Story 5 - Safety Protocols & Deployment (Weeks 17-18)

### Story Goal
Implement mandatory safety guardrails (kill switch, workspace boundaries, force limits, recovery)

### User Stories Included
- **US5**: Deploy Safety Protocols & Failure Recovery (P2)

### Independent Test
"Press kill switch during motion, motion halts instantly (<100ms). Exceed workspace boundary, motion rejected. Exceed force threshold, motion pauses."

### Acceptance Criteria
- Kill switch halts all motion within 100ms
- Workspace boundaries enforced before execution
- Force limits monitored and enforced
- Safe idle state after unplanned stop
- Safety node cannot be bypassed

---

#### Lesson 7: Safety Protocols & Deployment (3-4 hours)

- [ ] T078 [US5] [P] Write Lesson 7 markdown (07-lesson-7-safety.md) with learning objectives, safety node architecture (dedicated, independent), workspace bounds, joint limits, force limits, kill switch behavior, recovery strategies, hardware deployment checklist, regulatory considerations, liability
- [ ] T079 [US5] [P] Implement safety_validator_node.py (~150 lines) in chapter4_safety/src/: runs as ROS 2 action server, validates all TaskPlans before execution (checks workspace bounds, joint limits, gripper feasibility), rejects unsafe plans, implements kill switch (subscribes to /robot/kill_switch topic, halts all motion immediately)
- [ ] T080 [US5] [P] Create safety configuration (chapter4_safety/safety_config.yaml): defines workspace bounds (x_min/max, y_min/max, z_min/max), joint limits per joint, gripper force limits (min/max), kill switch timeout (100ms), safe idle state definition
- [ ] T081 [US5] [P] Implement force_monitor.py (~100 lines): monitors gripper force/torque during execution, triggers motion pause if limits exceeded, logs force violations
- [ ] T082 [US5] [P] Implement workspace_boundary_checker.py (~80 lines): checks trajectory points against workspace bounds, rejects trajectories that exceed boundaries
- [ ] T083 [US5] [P] Implement safety_validator_test.py (~50 lines) with pytest: test workspace checking, force limit validation, kill switch response time, safe idle state transition
- [ ] T084 [US5] Create Lesson 7 exercise (07-lesson-7-safety/exercises/ex6-safety-validation.md): "Test kill switch response time, verify workspace boundary enforcement, trigger force limit and verify motion pauses"
- [ ] T085 [US5] Write Lesson 7 hardware deployment section (07-lesson-7-safety.md): "Real hardware: E-stop button (electrical), emergency brake, tethered operation mode, supervisor oversight, liability insurance"
- [ ] T086 [US5] Write self-assessment rubric for Lesson 7 (assessments/lesson7-rubric.md) covering safety architecture, fail-safe design, deployment readiness, regulatory awareness
- [ ] T087 [US5] Create safety integration test (chapter4_safety/tests/e2e_safety.py): verify kill switch works during execution, verify boundaries enforced, verify force limits enforced

---

#### Completion Test for US5
- [ ] T088 Create safety test suite (chapter4_safety/tests/): all safety mechanisms independently testable, all pass within acceptable margins (kill switch <100ms, boundary checks <50ms)

---

## Phase 8: User Story 6 - Self-Assessment & Capstone (Weeks 19-20)

### Story Goal
Students complete self-assessment rubrics and deliver capstone project integrating all concepts

### User Stories Included
- **US6**: Self-Assessment & Capstone Project Completion (P2)

### Independent Test
Each self-assessment rubric independently checkable. Capstone project testable against student-defined success criteria.

### Acceptance Criteria
- 8 lesson rubrics cover all learning outcomes
- Capstone project uses ≥3 pipeline components
- Design document articulates tradeoffs
- Capstone presentation demonstrates mastery

---

#### Lesson 8: Capstone Project (8-10 hours, student-driven)

- [ ] T089 [US6] Write Lesson 8 markdown (08-lesson-8-capstone.md) with learning objectives, capstone project guidelines, project proposal template, system design document requirements, testing and validation approach, presentation guidelines, grading rubric
- [ ] T090 [US6] Create capstone project proposal template (08-lesson-8-capstone/templates/project_proposal.md): problem statement, system architecture, component list (must include ≥3 of: voice, vision, planning, manipulation, safety), success criteria, timeline
- [ ] T091 [US6] Create capstone design document template (08-lesson-8-capstone/templates/design_document.md): overview, architecture diagram, module descriptions, interfaces, failure modes, tradeoff analysis
- [ ] T092 [US6] Write comprehensive capstone rubric (assessments/capstone-rubric.md): design (architecture, tradeoffs), implementation (code quality, testing, error handling), validation (test results, demo video), presentation (clarity, depth, reflection)

---

#### Self-Assessment Rubrics (Created throughout phases)

- [ ] Lesson 1: VLA architecture understanding (T023)
- [ ] Lesson 2: Whisper integration (T031)
- [ ] Lesson 3: Vision systems (T042)
- [ ] Lesson 4: LLM planning (T054)
- [ ] Lesson 5: Manipulation (T065)
- [ ] Lesson 6: Integration (T075)
- [ ] Lesson 7: Safety (T086)
- [ ] Capstone: Complete system design (T092)

---

## Phase 9: Polish & Cross-Cutting Concerns (Weeks 20+)

### Goal
Documentation refinement, troubleshooting guides, deployment validation, CI/CD hardening

---

#### Polish Tasks

- [ ] T093 Create comprehensive troubleshooting guide (troubleshooting.md) covering all lessons: microphone issues, API key problems, Isaac Sim connection failures, common ROS 2 errors, physics simulation issues, debugging strategies
- [ ] T094 Write "Real Hardware" appendix (real_hardware_appendix.md) compiling all hardware deployment sections from lessons 1-7: what changes on real robots, hardware validation checklist, safety certification requirements
- [ ] T095 [P] Create example student capstone projects (08-lesson-8-capstone/examples/): 3-4 reference projects showing different architectures (simple object sorting, collaborative manipulation, mobile manipulation)
- [ ] T096 [P] Create Docusaurus sidebar configuration (book-source/sidebars.ts) with Chapter 4 content, ensure proper nesting of lessons and code examples
- [ ] T097 Create video production guide (video_guide.md) for students creating capstone demo videos: recording setup, demo script template, editing checklist
- [ ] T098 [P] Implement GitHub Actions CI/CD enhancements: ROS 2 linting on all packages, documentation build verification, code example syntax validation, dead link checking

---

## Dependencies & Parallelization Opportunities

### Execution Order
```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
Phase 3 (US1: Voice-Perception) [Can run in parallel with Phase 4-5 after foundational]
    ↓
Phase 4 (US2: LLM Planning) [Can run in parallel with Phase 5]
    ↓
Phase 5 (US3: Manipulation) [Can run in parallel with Phase 4]
    ↓
Phase 6 (US4: Full Integration) [Depends on 3, 4, 5]
    ↓
Phase 7 (US5: Safety) [Can run in parallel with Phase 6]
    ↓
Phase 8 (US6: Capstone) [Depends on all preceding]
    ↓
Phase 9 (Polish)
```

### Parallel Execution Examples

**Example 1: Weeks 8-13 (Lessons 4-5 parallel)**
- Developer A: Implement Lesson 4 (LLM planning) — T045-T054
- Developer B: Implement Lesson 5 (Manipulation) — T056-T065
- Both can run independently; integration in Phase 6 (Lesson 6)

**Example 2: Weeks 5-7 (Lesson 2-3 parallel)**
- Developer A: Implement Lesson 2 (Voice/Whisper) — T024-T032
- Developer B: Implement Lesson 3 (Vision) — T033-T043
- Both test independently; integrated in Lesson 6

**Example 3: Within Lesson 5**
- Task T058: Grasp planner (dev A)
- Task T059: Motion planner (dev B)
- Task T060: Grasp validator (dev C)
- All 3 can implement in parallel; integration in orchestrator (T068)

---

## Success Metrics

### Completeness
- ✅ All 92 tasks completed
- ✅ All 8 lessons delivered with content + code + exercises + rubrics
- ✅ All 6 ROS 2 packages functional and tested
- ✅ All safety features implemented and validated

### Quality
- ✅ Code examples run without errors
- ✅ All end-to-end tests pass
- ✅ Documentation clarity sufficient for students with Chapters 1-3 knowledge
- ✅ Troubleshooting guides resolve 90%+ of common issues

### Learning Outcomes
- ✅ Students can set up Whisper + ROS 2 in <30 minutes
- ✅ Students can implement complete grasp planning in <3 hours
- ✅ Students can integrate all components in <4 hours
- ✅ Capstone projects demonstrate mastery of VLA pipeline concepts

### MVP Scope Achievement
- ✅ 4 P1 user stories fully implemented (voice, perception, planning, manipulation, integration)
- ✅ Full-stack autonomous system operational in Isaac Sim
- ✅ All safety requirements non-negotiable and enforced
- ✅ Real-world deployment considerations explicit in every lesson
