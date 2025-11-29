# Implementation Plan: Chapter 4 - AI Integration & Autonomous Systems

**Branch**: `005-chapter4-ai-integration` | **Date**: 2025-11-29 | **Spec**: [specs/005-chapter4-ai-integration/spec.md](spec.md)
**Input**: Feature specification from `/specs/005-chapter4-ai-integration/spec.md`

## Summary

Chapter 4 is a comprehensive educational curriculum module integrating Voice-Language-Action (VLA) pipeline concepts into a complete autonomous system. Students build on Chapters 1-3 (ROS 2, Gazebo, navigation) to create:

1. **Voice-controlled perception** (Whisper ASR + object detection + semantic segmentation)
2. **LLM-driven task planning** (high-level command decomposition into executable subtasks)
3. **Manipulation & grasping** (grasp planning + motion planning + force/torque validation)
4. **Full-stack integration** (voice → perception → planning → manipulation → observation)
5. **Safety protocols** (kill switch, workspace boundaries, force limits, failure recovery)

**Technical approach**: Multi-lesson curriculum with progressive complexity (foundation → exploration → integration → orchestration). All examples in Isaac Sim. Code-first, theory-second pedagogy. Real-world deployment considerations in every lesson.

## Technical Context

**Language/Version**: Python 3.10+, ROS 2 Humble+
**Primary Dependencies**:
- **Speech**: OpenAI Whisper (ASR)
- **Vision**: PyTorch/TensorFlow (YOLO/Detectron for detection, FCN/DeepLab for segmentation)
- **LLM**: OpenAI API (GPT-4) or open-source (Llama via Ollama)
- **Robotics**: ROS 2 (rclpy), Isaac Sim 4.0+
- **Motion Planning**: MoveIt 2 (optional; simplified trajectory generation acceptable)
- **Simulation**: Isaac Sim 4.0+ (Gazebo noted as alternative)

**Storage**: Log files (disk), ROS 2 bag recordings (if needed), no persistent database required
**Testing**: pytest (Python unit tests), ROS 2 integration tests in Isaac Sim, acceptance tests via scenario replay
**Target Platform**: Linux (Ubuntu 22.04 + ROS 2 Humble), Isaac Sim on GPU-capable workstation
**Project Type**: Educational curriculum (8 lessons, code examples, exercises, rubrics)
**Performance Goals**:
- Voice-to-text latency: <500ms
- End-to-end pipeline latency: <10 seconds per subtask
- Object detection inference: <100ms per frame
- Motion planning: <1 second for trajectory generation
- Safety response: <100ms kill switch halt

**Constraints**:
- No real hardware deployment in Chapter 4 (simulation-only)
- Standalone lessons (can implement any lesson independently, though integration recommended)
- Kill switch / safety validation non-negotiable
- All code examples must run without errors in provided environment

**Scale/Scope**:
- 8 lessons, 18-20 hours instruction + exercises
- ~2000 lines of runnable example code
- 3-5 major ROS 2 nodes per lesson (voice, perception, planning, manipulation, orchestration)
- 100+ functional requirements across 6 categories
- Self-assessment rubrics for each lesson + capstone rubric

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **MVP-First**: 4 P1 stories (voice, perception, planning, manipulation, integration) are MVP. P2 stories (safety, assessment) are supporting. Scope is bounded (no advanced manipulation, no learning from feedback, no multi-robot).

✅ **Content Accuracy**: All technical content references official docs (OpenAI Whisper, ROS 2 docs, NVIDIA Isaac, PyTorch). Code examples must be tested before publication. No hallucinated APIs.

✅ **Technology Stack**: Chapter fits textbook stack (ROS 2 Humble, Isaac Sim, Python). OpenAI API + Ollama support provides flexibility (cloud + local options).

✅ **Educational Structure**: Clear learning outcomes defined. Content organized foundation → advanced. "Why" + "how" balance maintained. Real-world deployment in every lesson.

✅ **Safety Framework**: Simulation-first (C2 level advanced Isaac Sim use). Kill switch mandatory. Workspace boundaries enforced. Force limits monitored. Safe idle state after unplanned stop. Progressive autonomy discussed.

✅ **Spec-Driven**: Following Spec-Kit Plus (spec → plan → tasks). ADRs for significant decisions (LLM choice, simulation platform, safety architecture) will be created if plan identifies them.

✅ **Implementation Path**: 8 lessons + exercises + rubrics. Delivery via Docusaurus book (TypeScript/pnpm). No external infrastructure required (OpenAI API as service dependency). GitHub Pages deployment sufficient.

**No violations detected**. Constitutional checks pass. Proceed to Phase 0 research.

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

### Source Code (Docusaurus book structure)

Chapter 4 content is delivered as educational markdown + code examples within Docusaurus. No separate backend/frontend projects needed. Code examples are runnable ROS 2 packages referenced from lessons.

```text
book-source/docs/03-chapter-4-ai-integration/
├── 00-intro.md                          # Chapter overview, learning paths
├── 01-lesson-1-vla-architecture.md      # VLA pipeline overview
├── 02-lesson-2-voice-whisper.md         # Voice input with Whisper
│   ├── code/whisper_ros2_node.py        # Example ROS 2 node
│   ├── code/setup-whisper.sh            # Installation script
│   └── exercises/ex1-transcribe.md
├── 03-lesson-3-vision-systems.md        # Object detection + segmentation
│   ├── code/yolo_detector_node.py
│   ├── code/segmentation_node.py
│   └── exercises/ex2-detect-objects.md
├── 04-lesson-4-llm-planning.md          # LLM task planning
│   ├── code/llm_planner_node.py
│   ├── code/prompt_templates.py
│   └── exercises/ex3-decompose-tasks.md
├── 05-lesson-5-manipulation.md          # Grasp planning + motion planning
│   ├── code/grasp_planner_node.py
│   ├── code/motion_planner_node.py
│   └── exercises/ex4-pick-place.md
├── 06-lesson-6-full-integration.md      # Voice→perception→planning→manipulation
│   ├── code/orchestrator_node.py
│   ├── code/launch_files/vla_pipeline.launch.py
│   └── exercises/ex5-end-to-end.md
├── 07-lesson-7-safety.md                # Kill switch, boundaries, force limits
│   ├── code/safety_validator_node.py
│   ├── code/safety_config.yaml
│   └── exercises/ex6-safety-validation.md
├── 08-lesson-8-capstone.md              # Project design, implementation, presentation
│   ├── rubrics/capstone_rubric.md
│   ├── rubrics/design_checklist.md
│   └── templates/project_proposal.md

assessments/
├── lesson1-rubric.md                    # VLA architecture understanding
├── lesson2-rubric.md                    # Whisper integration
├── lesson3-rubric.md                    # Vision systems
├── lesson4-rubric.md                    # LLM planning
├── lesson5-rubric.md                    # Manipulation
├── lesson6-rubric.md                    # Full-stack integration
├── lesson7-rubric.md                    # Safety protocols
└── capstone-rubric.md                   # Capstone project evaluation

ros2_packages/                           # Example ROS 2 packages for students to reference/extend
├── chapter4_voice/                      # Whisper node + message definitions
├── chapter4_vision/                     # Detection + segmentation nodes
├── chapter4_planning/                   # LLM planner node
├── chapter4_manipulation/               # Grasp + motion planning nodes
├── chapter4_safety/                     # Safety validator node
└── chapter4_integration/                # Full-stack orchestrator

isaac_sim_scenes/                        # Pre-built Isaac Sim environments
├── scene_1_simple_table.yaml            # Single table with objects
├── scene_2_manipulation_task.yaml       # Table + shelves for pick-place
└── scene_3_complex_scene.yaml           # Multiple objects, obstacles
```

**Structure Decision**: Educational curriculum delivered in Docusaurus. Code examples as markdown-embedded + standalone files. ROS 2 packages provided for reference (students extend/adapt). No backend infrastructure (OpenAI API + local Ollama as external dependencies). Isaac Sim scenes provided for consistent testing environment.

## Complexity Tracking

No Constitution violations detected. Complexity justified by:
- **Multi-component curriculum** (8 lessons needed to cover VLA pipeline)
- **Multi-modal integration** (voice + vision + planning + manipulation = inherent complexity)
- **Safety mandatory** (non-negotiable for autonomous systems)
- **Real-world considerations** in every lesson (simulation-first but hardware-aware)

---

## Phase 0: Research & Clarification

**Objective**: Resolve unknowns and validate technology choices documented in Technical Context.

**Research Tasks**:

1. **Whisper ASR Integration with ROS 2**
   - Task: Best practices for real-time speech recognition in ROS 2 nodes
   - Finding: Whisper offers local inference (openai/whisper) and API-based (OpenAI API). Local inference better for real-time; API for cloud. ROS 2 integration via standard publisher-subscriber pattern.
   - Decision: Provide both options. Lesson emphasizes local inference (lower latency, no API key required for basic setup).

2. **Object Detection Model Selection (YOLO vs Detectron)**
   - Task: Evaluate object detection models for speed/accuracy tradeoff on simulated scenes
   - Finding: YOLO-v8 faster (<50ms inference), easier to fine-tune. Detectron more accurate but slower (~100ms). For educational use case, YOLO-v8 recommended. Detectron as advanced option.
   - Decision: Lead with YOLO-v8 for lessons 1-6. Lesson 3 mentions Detectron as alternative for higher accuracy.

3. **LLM Provider: OpenAI vs Open-Source (Ollama)**
   - Task: Evaluate cost, latency, accessibility for GPT-4 API vs local Llama via Ollama
   - Finding: GPT-4 API most capable but requires API key + internet. Ollama (Llama 2/3) local, free, but requires GPU, higher setup overhead. Recommend GPT-4 for initial learning, Ollama as advanced option for offline/cost-sensitive environments.
   - Decision: Lesson 4 leads with GPT-4 API examples (clear API contract). Appendix: "Running Llama locally with Ollama" for alternative deployments.

4. **Motion Planning: MoveIt 2 vs Custom Trajectory Generation**
   - Task: Evaluate complexity tradeoff for Lesson 5 manipulation examples
   - Finding: MoveIt 2 more powerful (collision checking, trajectory optimization) but steep learning curve. Custom trajectory generation (linear interpolation in joint space) sufficient for lessons 1-6. MoveIt 2 as P2 feature.
   - Decision: Lesson 5 uses simplified trajectory generation. Lesson 8 (capstone) notes "MoveIt 2 integration for advanced projects."

5. **Isaac Sim vs Gazebo: Which is Primary?**
   - Task: Confirm simulation platform for consistent examples
   - Finding: Spec states Isaac Sim primary, Gazebo alternative. Isaac Sim superior for humanoid robots + multi-modal sensors. Gazebo wider adoption in academia. Decision confirmed from spec: Isaac Sim primary.
   - Decision: All lessons assume Isaac Sim 4.0+. Gazebo equivalents documented in appendix for compatibility.

6. **Safety Validation Node: Dedicated vs Integrated**
   - Task: Architectural decision for safety enforcement
   - Finding: Dedicated safety node (separate from orchestrator) allows independent testing and fail-safe design. Spec already recommends dedicated node. Safety node runs in separate thread, has hard real-time guarantees (kill switch <100ms response).
   - Decision: Lesson 7 implements dedicated safety validator as ROS 2 action server.

**Output**: All unknowns resolved. No blocking clarifications remain. Proceed to Phase 1 design.

---

## Phase 1: Design & API Contracts

**Objective**: Design data models, API contracts, and generate quickstart guide.

### 1.1 Data Model (`data-model.md`)

**Key Entities**:

- **VoiceCommand**: Text (string), confidence (0-1), timestamp (ISO 8601), user_id (string)
- **DetectionResult**: objects (list of {class, bbox, confidence}), frame_id (int), timestamp
- **TaskPlan**: goal (string), subtasks (list), estimated_duration (float seconds), safety_constraints (dict)
- **GraspCandidate**: object_id (string), pose (6D position+orientation), success_probability (0-1), grasp_type (enum)
- **ExecutionStatus**: current_command (string), progress_percent (0-100), current_action (string), errors (list)
- **SafetyEvent**: event_type (enum: kill_switch, boundary_violation, force_limit), timestamp, action_taken (string)
- **RobotState**: position_cartesian (3D), position_joint (7D for humanoid arm), gripper_state (enum: open/closed/gripping), detected_objects (list)

Relationships:
- TaskPlan contains Subtasks (composition)
- TaskPlan references detected_objects (N:1 to DetectionResult)
- ExecutionStatus tracks GraspCandidate (1:1)
- SafetyEvent modifies RobotState (N:1)

State Transitions:
- RobotState: idle → moving → grasping → idle
- ExecutionStatus: commanded → recognized → perceived → planned → executing → completed/failed
- TaskPlan: generated → validated → executing → completed/failed/replanned

### 1.2 API Contracts (ROS 2 Interfaces)

All interfaces defined as ROS 2 topics, services, and actions (protobuf-based messages).

**Topics** (Publisher-Subscriber):
- `/robot/voice_command` (std_msgs/String) — Voice command transcribed text
- `/robot/detections` (custom DetectionArray) — Object detection results
- `/robot/segmentation` (sensor_msgs/Image) — Semantic segmentation mask
- `/robot/task_status` (std_msgs/String or custom) — Pipeline status updates
- `/robot/gripper_state` (std_msgs/Float32) — Current gripper force

**Services** (Request-Response):
- `/robot/plan_task` (request: TaskGoal, response: TaskPlan) — LLM planner service
- `/robot/validate_plan` (request: TaskPlan, response: IsValid+Reason) — Safety validation
- `/robot/generate_grasps` (request: Object, response: GraspCandidates) — Grasp planning

**Actions** (Stateful, with feedback):
- `/robot/execute_task` (goal: TaskPlan, feedback: ExecutionStatus, result: Success+Errors) — Task executor
- `/robot/move_arm` (goal: Pose+Speed, feedback: Progress, result: Success) — Arm motion action

All interfaces versioned. Error handling: standard ROS 2 error codes + domain-specific codes (e.g., "PERCEPTION_FAILED", "UNSAFE_PLAN").

### 1.3 Quickstart Guide (`quickstart.md`)

**Sections**:
1. Environment setup (ROS 2 Humble, Isaac Sim, OpenAI API key or Ollama)
2. Clone example ROS 2 packages
3. Run Lesson 1 (VLA architecture overview) — no code, just diagrams
4. Run Lesson 2 (Whisper node) — test voice-to-text
5. Run Lesson 3 (Vision detection) — test object detection
6. Run end-to-end example (voice → perception → logging)
7. Troubleshooting common issues (microphone not detected, API key invalid, etc.)

**Expected time**: <2 hours from zero to running first example.

### 1.4 Agent Context Update

Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude` to register new technologies:
- OpenAI Whisper (ASR)
- YOLO-v8 (object detection)
- GPT-4 API (LLM planning)
- ROS 2 custom messages (DetectionArray, etc.)

---

## Phase 2: Implementation Roadmap

**Objective**: Outline lesson-by-lesson implementation sequence and identify architectural decisions requiring ADRs.

### 2.1 Lesson Implementation Sequence

**Lesson 1: VLA Architecture Overview** (4-6 hours)
- Learning objectives: Conceptual understanding of VLA pipeline
- Deliverables: Architecture diagram, real-world examples, comparison with traditional programming
- Code: None (theory + diagrams)
- No new ADRs needed

**Lesson 2: Voice Input with Whisper** (4-6 hours)
- Learning objectives: Whisper setup, ROS 2 node design, error handling
- Deliverables: Runnable ROS 2 node, setup guide, exercises
- Code: whisper_ros2_node.py (~100 lines), setup-whisper.sh
- ADR: "Whisper Local vs API Inference Trade-offs" (latency vs API cost/dependency)

**Lesson 3: Vision Systems** (4-6 hours)
- Learning objectives: Object detection, semantic segmentation, depth integration
- Deliverables: Detection + segmentation nodes, exercises
- Code: yolo_detector_node.py (~150 lines), segmentation_node.py (~150 lines)
- ADR: "YOLO-v8 vs Detectron for Educational Use" (accuracy vs speed tradeoff)

**Lesson 4: LLM-Driven Planning** (4-6 hours)
- Learning objectives: Prompt engineering, LLM integration, error recovery
- Deliverables: LLM planner node, prompt templates, exercises
- Code: llm_planner_node.py (~200 lines), prompt_templates.py (~100 lines)
- ADR: "OpenAI GPT-4 vs Local Ollama for Task Planning" (capability vs infrastructure cost)

**Lesson 5: Manipulation & Grasping** (4-6 hours)
- Learning objectives: Grasp planning, motion planning, force/torque validation
- Deliverables: Grasp planner + motion planner nodes, exercises
- Code: grasp_planner_node.py (~150 lines), motion_planner_node.py (~200 lines)
- ADR: "Custom Trajectory Generation vs MoveIt 2" (simplicity vs functionality tradeoff)

**Lesson 6: Full-Stack Integration** (4-6 hours)
- Learning objectives: Component orchestration, asynchronous coordination, end-to-end testing
- Deliverables: Orchestrator node, launch files, integration exercises
- Code: orchestrator_node.py (~250 lines), launch_files/vla_pipeline.launch.py (~50 lines)
- ADR: "Orchestration Architecture: State Machine vs Behavior Tree vs Action Server" (complexity vs expressiveness)

**Lesson 7: Safety Protocols** (3-4 hours)
- Learning objectives: Kill switch, workspace boundaries, force limits, recovery
- Deliverables: Safety validator node, safety config, exercises
- Code: safety_validator_node.py (~150 lines), safety_config.yaml
- ADR: "Dedicated Safety Node vs Integrated Safety Checks" (separation of concerns vs performance)

**Lesson 8: Capstone Project** (8-10 hours)
- Learning objectives: System design, implementation, testing, presentation
- Deliverables: Project proposals, rubrics, example projects
- Code: Student projects (not authored, evaluated against rubric)
- No new ADRs (capstone applies earlier decisions)

### 2.2 Architectural Decisions (ADRs)

**ADR-001: VLA Pipeline Architecture**
- **Decision**: Decoupled ROS 2 nodes (voice → perception → planning → manipulation → observation) communicating via topics/services
- **Rationale**: Enables independent testing, fault isolation, modularity for learning
- **Alternatives Rejected**: Monolithic node (too complex for education), shared memory (tight coupling)

**ADR-002: Whisper Integration (Local vs API)**
- **Decision**: Lead with local Whisper inference; document OpenAI API option
- **Rationale**: <500ms latency, no API key required for basics, works offline
- **Alternatives Rejected**: API-only (introduces cost + dependency)

**ADR-003: LLM Provider (OpenAI GPT-4 vs Ollama)**
- **Decision**: Lead with GPT-4 API; provide Ollama as alternative deployment
- **Rationale**: GPT-4 most capable for task planning, clean API contract, well-documented
- **Alternatives Rejected**: Ollama-only (high setup overhead for beginners)

**ADR-004: Safety Validation**
- **Decision**: Dedicated safety node running asynchronously, validates all plans before execution
- **Rationale**: Independent testing, fail-safe architecture, enforces safety non-negotiable
- **Alternatives Rejected**: Integrated checks (harder to test independently, safety can be bypassed)

**ADR-005: Motion Planning (Custom vs MoveIt 2)**
- **Decision**: Lesson 5-6 use simplified trajectory generation; MoveIt 2 as P2 advanced feature
- **Rationale**: Simplicity for learners, sufficient for chapter scope, MoveIt 2 can be integrated later
- **Alternatives Rejected**: MoveIt 2-first (steep learning curve, detracts from core concepts)

---

## Summary & Next Steps

✅ **Phase 0 (Research)**: All clarifications resolved. Technology choices validated.
✅ **Phase 1 (Design)**: Data models, API contracts, and quickstart guide prepared.
✅ **Phase 2 (Roadmap)**: Lesson implementation sequence and ADR decisions documented.

**Constitution Check (Post-Design)**: All principles still satisfied. No new violations introduced.

**Next Steps**:
1. Create `research.md` with Phase 0 findings
2. Create `data-model.md` with entity definitions and state transitions
3. Create contract files in `contracts/` directory
4. Create `quickstart.md` with setup instructions
5. Generate `tasks.md` via `/sp.tasks` command (breaks plan into testable, prioritized tasks)
6. Begin implementation in lesson order (Lesson 1 → Lesson 8)
