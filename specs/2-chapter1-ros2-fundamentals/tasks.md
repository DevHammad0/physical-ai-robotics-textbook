# Tasks: Chapter 1 - ROS 2 Fundamentals & Communication

**Input**: Design documents from `/specs/2-chapter1-ros2-fundamentals/`
**Prerequisites**: plan.md ✅, spec.md ✅
**Status**: Ready for implementation
**Branch**: `2-chapter1-ros2-fundamentals`

---

## Overview

Chapter 1 implementation tasks organized by 6 user stories (P1-P3 priority) covering ROS 2 communication patterns. Tasks include content generation, code examples, testing, and Docusaurus integration. **Total: 45 tasks** across 7 phases.

**MVP Scope**: User Stories 1-2 (Lessons 1-4: Understanding ROS 2 + Creating Nodes)

---

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Parallelizable (different files, no dependencies)
- **[Story]**: User story label (US1-US6)
- Include exact file paths

---

## Phase 1: Setup & Project Initialization

**Purpose**: Create directory structure and establish Docusaurus chapter framework

**Deliverable**: Chapter skeleton with lesson placeholders and example directories ready

- [ ] T001 Create chapter directory structure in `book-source/docs/chapter-1-ros2-fundamentals/`
- [ ] T002 Create examples directory structure in `examples/chapter-1-ros2/` with subdirectories for each lesson
- [ ] T003 Create test framework directory in `tests/` with unit/ and integration/ subdirectories
- [ ] T004 [P] Create chapter intro template in `book-source/docs/chapter-1-ros2-fundamentals/intro.md`
- [ ] T005 [P] Create lesson template file (for reference) in `specs/2-chapter1-ros2-fundamentals/lesson-template.md`

**Checkpoint**: Directory structure ready for lesson content

---

## Phase 2: Foundational Content (Blocking Prerequisites)

**Purpose**: Create foundation material required by all lessons

**⚠️ CRITICAL**: These must complete before lesson-specific work begins

- [ ] T006 Create ROS 2 architecture diagram (text/ASCII) in `examples/chapter-1-ros2/lesson-01/architecture-diagram.txt`
- [ ] T007 Create installation verification script in `examples/chapter-1-ros2/lesson-02/installation-checklist.sh`
- [ ] T008 Create README.md for examples directory in `examples/chapter-1-ros2/README.md` with instructions for running all examples
- [ ] T009 [P] Create pytest configuration in `tests/conftest.py` for ROS 2 test support
- [ ] T010 [P] Create test utilities module in `tests/test_utils.py` for common ROS 2 testing helpers
- [ ] T011 Create capstone project specification in `specs/2-chapter1-ros2-fundamentals/capstone-spec.md`

**Checkpoint**: Foundation ready - lesson implementation can begin

---

## Phase 3: User Story 1 - Understand ROS 2 Architecture (Priority: P1)

**Goal**: Students understand what ROS 2 is, why it exists, and the 4 communication patterns

**Independent Test**: Student can explain ROS 2 components, identify them in architecture diagrams, and select correct patterns for scenarios

### Content Generation for US1

- [ ] T012 [P] [US1] Create Lesson 1 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-01-what-is-ros2.md`
  - Content: ROS 2 overview, architecture, ecosystem, use cases
  - Include learning outcomes, concepts (5-6 CEFR A2), and explanations

- [ ] T013 [P] [US1] Create Lesson 2 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-02-setup.md`
  - Content: Installation steps, Turtlesim first run, environment verification
  - Include prerequisites, setup instructions, troubleshooting

- [ ] T014 [P] [US1] Create Lesson 3 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-03-nodes-communication.md`
  - Content: Node structure, topics, pub/sub pattern, services, actions, messages, naming conventions
  - Include diagrams, theory, and comparison table of patterns

### Deliverables for US1

- [ ] T015 [US1] Add self-assessment checklist to Lesson 1 (students verify understanding of architecture)
- [ ] T016 [US1] Add self-assessment checklist to Lesson 2 (verify Turtlesim installation)
- [ ] T017 [US1] Add self-assessment checklist to Lesson 3 (verify concept understanding)
- [ ] T018 [US1] Create common errors section for Lessons 1-3 (e.g., "ROS_DOMAIN_ID not set", "Turtlesim fails to launch")
- [ ] T019 [US1] Add CLI debugging tools section to Lesson 3 (`ros2 topic list`, `ros2 node list`, `rqt_graph`)

**Checkpoint**: User Story 1 complete - students can understand ROS 2 fundamentals

---

## Phase 4: User Story 2 - Create and Control ROS 2 Nodes (Priority: P1)

**Goal**: Students can write ROS 2 publisher nodes in Python that control Turtlesim

**Independent Test**: Student writes publisher → publishes Twist messages → turtle moves in response

### Content Generation for US2

- [ ] T020 [P] [US2] Create Lesson 4 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-04-publisher.md`
  - Content: rclpy.Node structure, publisher creation, Twist messages, message publishing, message rate control
  - Include 4 code examples (MWE → with params → with error handling → with testing)

### Code Examples for US2

- [ ] T021 [P] [US2] Create simple publisher example in `examples/chapter-1-ros2/lesson-04/simple_publisher.py`
  - MWE level: ~20 lines, publishes Twist to /turtle1/cmd_vel every 0.1s
  - Includes docstring marking "Simulation environment: Turtlesim"

- [ ] T022 [P] [US2] Create publisher with parameters in `examples/chapter-1-ros2/lesson-04/publisher_with_params.py`
  - Production-aware level: ~45 lines
  - Accepts parameters for velocity, frequency, topic name

- [ ] T023 [P] [US2] Create publisher with error handling in `examples/chapter-1-ros2/lesson-04/publisher_with_error_handling.py`
  - Production-aware level: ~50 lines
  - Handles node shutdown, parameter validation, exception handling

- [ ] T024 [P] [US2] Create unit tests in `examples/chapter-1-ros2/lesson-04/test_publisher.py`
  - pytest suite: Test publisher creation, message publishing rate, parameter handling
  - 80%+ code coverage

### Validation for US2

- [ ] T025 [US2] Validate all publisher examples with `simulation-code-validation` skill
  - Check Python syntax, ROS 2 patterns, Turtlesim compatibility, safety checks

- [ ] T026 [US2] Create how-to-run section in Lesson 4 with step-by-step instructions
  - Instructions for running Turtlesim, launching publisher, observing turtle movement

- [ ] T027 [US2] Add self-assessment checklist to Lesson 4 (5+ items validating publisher skills)

- [ ] T028 [US2] Create common errors section for Lesson 4 (e.g., "Node not publishing", "Turtle doesn't move", "Import errors")

**Checkpoint**: User Story 2 complete - students can write and run publisher nodes

---

## Phase 5: User Story 3 - Process Sensor Data Asynchronously (Priority: P2)

**Goal**: Students can subscribe to topics and process data asynchronously with callbacks

**Independent Test**: Student writes subscriber → listens to /turtle1/pose → callback processes data without blocking

### Content Generation for US3

- [ ] T029 [P] [US3] Create Lesson 5 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-05-subscriber.md`
  - Content: Subscriber creation, callback functions, asynchronous processing, data extraction, message types
  - Include 3 code examples (MWE → data processing → error handling)

### Code Examples for US3

- [ ] T030 [P] [US3] Create simple subscriber in `examples/chapter-1-ros2/lesson-05/simple_subscriber.py`
  - MWE level: ~25 lines, subscribes to /turtle1/pose, prints messages

- [ ] T031 [P] [US3] Create subscriber with data processing in `examples/chapter-1-ros2/lesson-05/data_processor.py`
  - Production-aware level: ~50 lines
  - Calculates distance traveled, speed, logs to file

- [ ] T032 [P] [US3] Create subscriber with error handling in `examples/chapter-1-ros2/lesson-05/subscriber_with_error_handling.py`
  - Production-aware level: ~55 lines
  - Timeout handling, graceful shutdown, exception handling

- [ ] T033 [P] [US3] Create unit tests in `examples/chapter-1-ros2/lesson-05/test_subscriber.py`
  - pytest suite: Test callback invocation, data extraction, error scenarios
  - Mock ROS 2 topics, verify callback execution

### Validation for US3

- [ ] T034 [US3] Validate all subscriber examples with `simulation-code-validation` skill

- [ ] T035 [US3] Create how-to-run section in Lesson 5 with instructions
  - Launch Turtlesim, publisher (from Lesson 4), subscriber, observe data processing

- [ ] T036 [US3] Add self-assessment checklist to Lesson 5

- [ ] T037 [US3] Create common errors section for Lesson 5 (e.g., "Callback not firing", "Data not extracted", "Deadlock issues")

**Checkpoint**: User Story 3 complete - students can write subscribers with callbacks

---

## Phase 6: User Story 4 - Implement Services & Actions (Priority: P2)

**Goal**: Students understand and implement request-response (services) and long-running tasks (actions)

**Independent Test**: Student implements service server + action server → both respond to client requests correctly

### Content Generation for US4

- [ ] T038 [P] [US4] Create Lesson 6 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-06-services-actions.md`
  - Content: Services vs topics, service client/server, actions, goals, feedback, results, state machines
  - Include 4 code examples (service MWE → action MWE → combined → testing)
  - CEFR B1: 9-10 concepts

### Code Examples for US4

- [ ] T039 [P] [US4] Create service server example in `examples/chapter-1-ros2/lesson-06/service_server.py`
  - MWE level: ~30 lines, implements freeze_robot service

- [ ] T040 [P] [US4] Create action server example in `examples/chapter-1-ros2/lesson-06/action_server.py`
  - MWE level: ~40 lines, implements rotate_360 action with feedback

- [ ] T041 [P] [US4] Create combined service+action example in `examples/chapter-1-ros2/lesson-06/combined_server.py`
  - Production-aware level: ~70 lines
  - Both service and action in one node with parameter configuration

- [ ] T042 [P] [US4] Create unit tests in `examples/chapter-1-ros2/lesson-06/test_services.py`
  - pytest suite: Test service request/response, action goal/feedback/result
  - Mock clients and verify server behavior

### Validation for US4

- [ ] T043 [US4] Validate all service/action examples with `simulation-code-validation` skill

- [ ] T044 [US4] Create how-to-run section in Lesson 6
  - Instructions for running servers and clients, testing with ros2 service/action CLI

- [ ] T045 [US4] Add self-assessment checklist to Lesson 6

- [ ] T046 [US4] Create common errors section for Lesson 6 (e.g., "Service not responding", "Action timeout", "Feedback not received")

**Checkpoint**: User Story 4 complete - students understand services and actions

---

## Phase 7: User Story 5 - Orchestrate Multi-Node Systems (Priority: P2)

**Goal**: Students create launch files that orchestrate multiple nodes and configure parameters

**Independent Test**: Student creates launch file → all nodes start → nodes communicate → capstone project runs

### Content Generation for US5

- [ ] T047 [P] [US5] Create Lesson 7 markdown in `book-source/docs/chapter-1-ros2-fundamentals/lesson-07-launch-files.md`
  - Content: Launch files (.launch.py), parameters, node lifecycle, namespacing, multi-node debugging, coordination
  - Include 5 code examples (launch file MWE → with parameters → multi-node → capstone orchestration)

### Code Examples for US5

- [ ] T048 [P] [US5] Create simple launch file in `examples/chapter-1-ros2/lesson-07/simple.launch.py`
  - MWE level: ~15 lines, launches Turtlesim and one publisher node

- [ ] T049 [P] [US5] Create launch file with parameters in `examples/chapter-1-ros2/lesson-07/launch_with_params.launch.py`
  - Production-aware level: ~30 lines
  - Configurable node names, frequencies, topics

- [ ] T050 [P] [US5] Create multi-node publisher in `examples/chapter-1-ros2/lesson-07/publisher_node.py`
  - For capstone orchestration: ~50 lines, publishes velocity commands

- [ ] T051 [P] [US5] Create multi-node subscriber in `examples/chapter-1-ros2/lesson-07/subscriber_node.py`
  - For capstone orchestration: ~50 lines, listens to position and logs

- [ ] T052 [P] [US5] Create coordinator node in `examples/chapter-1-ros2/lesson-07/coordinator_node.py`
  - For capstone orchestration: ~60 lines
  - Coordinates pub/sub nodes, implements simple state machine

- [ ] T053 [P] [US5] Create capstone launch file in `examples/chapter-1-ros2/lesson-07/turtle_control.launch.py`
  - Orchestrates publisher, subscriber, coordinator with parameters

- [ ] T054 [P] [US5] Create integration tests in `examples/chapter-1-ros2/lesson-07/test_integration.py`
  - pytest suite: Test multi-node communication, parameter passing, state transitions
  - Full system integration testing

### Validation for US5

- [ ] T055 [US5] Validate all launch files and orchestration code with `simulation-code-validation` skill

- [ ] T056 [US5] Create how-to-run section in Lesson 7
  - Instructions for launching capstone system, observing multi-node coordination

- [ ] T057 [US5] Add self-assessment checklist to Lesson 7

- [ ] T058 [US5] Create common errors section for Lesson 7 (e.g., "Node not starting", "Parameter not found", "Communication fails")

**Checkpoint**: User Story 5 complete - students can orchestrate multi-node systems

---

## Phase 8: User Story 6 - Capstone Project Integration (Priority: P3)

**Goal**: Students complete capstone project demonstrating all lessons 1-7

**Independent Test**: Student runs capstone → all nodes launch → Turtlesim responds → system behaves as designed

### Capstone Content

- [ ] T059 [US6] Create capstone markdown in `book-source/docs/chapter-1-ros2-fundamentals/capstone.md`
  - Project specification: "Multi-Node Turtlesim Control System"
  - Requirements: 3+ coordinated nodes, publisher + subscriber + service, launch file
  - Success criteria: Turtle responds to coordinated commands, system runs for 60+ seconds without errors
  - Estimated duration: 2-3 hours

- [ ] T060 [US6] Create capstone reference solution in `examples/chapter-1-ros2/capstone/solution.launch.py`
  - Complete working example students can reference if stuck
  - Includes publisher_node.py, subscriber_node.py, coordinator_node.py

- [ ] T061 [US6] Create capstone rubric in `specs/2-chapter1-ros2-fundamentals/capstone-rubric.md`
  - Grading criteria (if instructors want to grade submissions)
  - 5-10 criteria covering functionality, code quality, documentation

- [ ] T062 [US6] Add capstone hints/troubleshooting section in capstone.md
  - "Stuck? Try this..." debugging guidance

**Checkpoint**: Capstone project complete - students demonstrate mastery

---

## Phase 9: Polish, Integration & Deployment

**Purpose**: Final validation, documentation, and Docusaurus integration

### Code Validation & Documentation

- [ ] T063 Run all code examples through `ros2-node-patterns` skill for review
  - Verify node structure follows best practices
  - Check naming conventions, docstrings, error handling

- [ ] T064 Create comprehensive examples README in `examples/chapter-1-ros2/README.md`
  - How to run each lesson's examples
  - System requirements (ROS 2 Humble, Python 3.10+)
  - Troubleshooting guide

- [ ] T065 Create test documentation in `tests/README.md`
  - How to run test suite
  - Test coverage report methodology
  - Continuous integration guidance

### Docusaurus Integration

- [ ] T066 Update book-source/docusaurus.config.js to include Chapter 1 in sidebar navigation
  - Add chapter and lesson links in correct order
  - Ensure all markdown files are referenced

- [ ] T067 Create chapter landing page by updating `book-source/docs/chapter-1-ros2-fundamentals/intro.md`
  - Chapter overview (30-second summary)
  - Prerequisites list
  - Learning outcomes (7 high-level outcomes for chapter)
  - Lesson breakdown with estimated durations
  - Links to each lesson

- [ ] T068 [P] Validate all Docusaurus markdown syntax
  - Check for broken links, missing images, formatting issues
  - Verify code blocks render correctly

### Final Quality Checks

- [ ] T069 Verify CEFR compliance for all lessons
  - Lessons 1-3: Confirm 5-7 concepts each
  - Lessons 4-7: Confirm 8-10 concepts each
  - Document concept list for each lesson

- [ ] T070 Verify constitutional compliance
  - Confirm simulation-first (no hardware, no forward references)
  - Verify 4-Layer pedagogy included (Layer 1: manual + Layer 2: AI notes)
  - Confirm troubleshooting sections present
  - Verify self-assessment checklists complete

- [ ] T071 [P] Create final summary document in `specs/2-chapter1-ros2-fundamentals/implementation-summary.md`
  - List all generated files (lessons, code examples, tests)
  - Document line counts, concept counts, example counts
  - Success metrics met (90% student success, all tests pass, etc.)

- [ ] T072 Update `specs/2-chapter1-ros2-fundamentals/README.md` with implementation status
  - What's complete, what's tested, deployment status

**Checkpoint**: Chapter 1 complete and ready for deployment

---

## Dependency Graph & Parallel Opportunities

### Critical Path (Sequential)

1. **Phase 1** (Setup) - must complete first
2. **Phase 2** (Foundation) - must complete before user stories
3. **Phases 3-8** - can run in parallel within phases:
   - T012-T019 (US1 content) - parallel
   - T020-T028 (US2 code) - parallel
   - T029-T037 (US3 code) - parallel (independent from US2)
   - T038-T046 (US4 code) - parallel (independent from US2/US3)
   - T047-T058 (US5 orchestration) - can start after US2/US3/US4 examples exist
   - T059-T062 (US6 capstone) - depends on US5 completion
4. **Phase 9** (Polish) - final phase, cannot start until all lessons complete

### Parallelization Opportunities

**Lesson Content Generation** (can run in parallel):
- T012 (Lesson 1) ↔ T013 (Lesson 2) ↔ T014 (Lesson 3) - independent markdown files

**Code Examples by Lesson** (can run in parallel):
- T021-T024 (Lesson 4 examples) ↔ T030-T033 (Lesson 5) ↔ T039-T042 (Lesson 6) ↔ T048-T054 (Lesson 7)

**Validation by Lesson** (can run in parallel):
- T025 (Validate US2) ↔ T034 (Validate US3) ↔ T043 (Validate US4) ↔ T055 (Validate US5)

**Docusaurus Tasks** (can run in parallel):
- T068 (Markdown validation) ↔ T069 (CEFR verification) ↔ T070 (Constitutional check)

---

## MVP Scope Recommendation

**Minimum Viable Product** for Chapter 1 (can deliver in 4-5 hours):

**Phase 3: User Story 1 (Understanding)** - REQUIRED
- Lessons 1-3: ROS 2 overview, setup, communication patterns
- Learning outcome: Students understand ROS 2 architecture

**Phase 4: User Story 2 (Publisher Nodes)** - REQUIRED
- Lesson 4: Publisher node implementation
- Learning outcome: Students write and run publisher nodes
- 4 code examples with testing

**These two user stories form a complete, functional MVP** that teaches the foundational ROS 2 concepts.

**Optional additions** (if time permits):
- Phase 5 (Subscribers) - adds asynchronous processing skill
- Phase 6 (Services/Actions) - complete communication toolkit
- Phase 7 (Launch Files) - orchestration and production patterns
- Phase 8 (Capstone) - integration project

---

## Implementation Strategy

### Parallel Execution Plan (Recommended)

**If 2 people available**:
- **Person A**: Lessons 1-3 content (T012-T019) + Lesson 4 code (T021-T028)
- **Person B**: Lessons 5-7 code (T030-T058) + Docusaurus integration (T066-T072)

**Timeline**: ~5-6 hours for MVP + full chapter

### Testing Strategy

- Each lesson: 5+ common errors documented
- Each code example: Unit tests with 80%+ coverage
- Capstone: Full system integration test
- Constitution compliance: Automated checks (simulation-only, no forward refs)

### Quality Metrics

- **Success rate**: 90%+ of students pass self-assessment checklists
- **Test coverage**: 80%+ for all code examples
- **Documentation**: 100% of concepts explained, all code examples runnable
- **Deployment**: Docusaurus build succeeds, no broken links, GitHub Pages ready

---

## Task Summary

| Phase | Count | Focus |
|-------|-------|-------|
| 1 Setup | 5 | Directory structure |
| 2 Foundation | 6 | Shared infrastructure |
| 3 US1 | 8 | Understanding ROS 2 |
| 4 US2 | 8 | Publisher nodes |
| 5 US3 | 9 | Subscriber nodes |
| 6 US4 | 9 | Services/actions |
| 7 US5 | 12 | Orchestration |
| 8 US6 | 4 | Capstone project |
| 9 Polish | 10 | Validation & deployment |
| **TOTAL** | **72 tasks** | **Complete chapter** |

---

**Status**: ✅ TASKS COMPLETE - READY FOR IMPLEMENTATION
**Branch**: `2-chapter1-ros2-fundamentals`
**Last Updated**: 2025-11-29
**Created By**: /sp.tasks command
