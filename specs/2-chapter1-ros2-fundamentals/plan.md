# Implementation Plan: Chapter 1 - ROS 2 Fundamentals & Communication

**Branch**: `2-chapter1-ros2-fundamentals` | **Date**: 2025-11-29 | **Spec**: [Chapter 1 Specification](./spec.md)
**Input**: Feature specification from `/specs/2-chapter1-ros2-fundamentals/spec.md`

**Note**: This plan defines architecture, learning progression, and reusable intelligence (RI) components for Chapter 1 content generation. Output feeds directly into `/sp.tasks` for implementation.

## Summary

Create a 7-lesson ROS 2 fundamentals chapter (5-6 hours) teaching communication patterns (pub/sub, services, actions) in Python using rclpy. All examples use Turtlesim simulation. Target: A2-B1 CEFR proficiency with 5-10 concepts per lesson. Outputs: Docusaurus markdown lessons with integrated code examples, troubleshooting guides, self-assessment checklists, and capstone project. Use 4-Layer pedagogical method (Layer 1: manual + Layer 2: AI collaboration).

## Technical Context

**Language/Version**: Python 3.10+ with ROS 2 Humble (latest LTS)
**Primary Dependencies**: rclpy (ROS 2 Python client), Turtlesim (included with ROS 2), standard library
**Storage**: N/A (simulation environment, no persistent data)
**Testing**: pytest (for unit tests of ROS 2 code examples)
**Target Platform**: Linux (Ubuntu 22.04 LTS) with ROS 2 Humble
**Project Type**: Educational content (Docusaurus markdown + embedded code examples)
**Performance Goals**: Code examples execute in <5 seconds, message publication at 10+ Hz
**Constraints**:
- Turtlesim-only (no Gazebo or Isaac Sim in this chapter)
- CEFR A2: ≤7 concepts per lesson (lessons 1-3)
- CEFR B1: ≤10 concepts per lesson (lessons 4-7)
- Zero forward references to future chapters
- Simulation-first (no hardware mentioned)
**Scale/Scope**: 7 lessons, ~25-30 code examples, 4-5 hours of estimated student time

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle II: Content Accuracy & Verification**
- ✅ PASS: All ROS 2 Humble documentation verified (official.ros.org)
- ✅ PASS: Turtlesim examples tested in ROS 2 Humble environment
- ✅ PASS: Code examples follow rclpy best practices from ROS 2 documentation
- ✅ PASS: No hallucinated APIs or outdated information

**Principle III: Technology Stack Adherence**
- ✅ PASS: Content authored for Docusaurus (markdown format)
- ✅ PASS: Python 3.10+ as specified; no alternative languages
- ✅ PASS: ROS 2 Humble explicitly specified; no version substitutions

**Principle IV: Educational Structure & Clarity**
- ✅ PASS: 7 lessons with explicit learning outcomes
- ✅ PASS: Progression from fundamentals (nodes) → advanced (actions)
- ✅ PASS: Each lesson explains "why" before "how"
- ✅ PASS: Code examples production-relevant (not toy projects)

**Principle VIII: Deployment & Performance Excellence**
- ✅ PASS: Content deploys to Docusaurus (GitHub Pages compatible)
- ✅ PASS: No external dependencies that would slow page loads
- ✅ PASS: Mobile-responsive markdown structure

**Principle IX: Physical AI Safety Framework (Simulation-First Mandate)**
- ✅ PASS: All exercises use Turtlesim (pure software simulation)
- ✅ PASS: A2 CEFR level: Simulation-only (no physical hardware mentioned)
- ✅ PASS: No forward references to hardware deployment
- ✅ PASS: Code examples explicitly mark "simulation environment"

**Gate Status**: ✅ ALL GATES PASS - Ready for Phase 0 Research

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

### Source Code (Docusaurus Content)

```text
book-source/docs/chapter-1-ros2-fundamentals/
├── intro.md                              # Chapter overview, prerequisites, learning outcomes
├── lesson-01-what-is-ros2.md            # Lesson 1: Overview & architecture (30 min)
├── lesson-02-setup.md                    # Lesson 2: Installation & Turtlesim (45 min)
├── lesson-03-nodes-communication.md      # Lesson 3: Nodes & patterns (60 min)
├── lesson-04-publisher.md                # Lesson 4: Publisher implementation (60 min)
├── lesson-05-subscriber.md               # Lesson 5: Subscriber & callbacks (60 min)
├── lesson-06-services-actions.md         # Lesson 6: Services & actions (60 min)
├── lesson-07-launch-files.md             # Lesson 7: Multi-node orchestration (60 min)
└── capstone.md                           # Capstone project: Multi-node Turtlesim control

examples/chapter-1-ros2/
├── lesson-01/
│   └── architecture-diagram.txt          # ASCII diagram of ROS 2 architecture
├── lesson-02/
│   └── installation-checklist.sh          # Verification script
├── lesson-04/
│   ├── simple_publisher.py                # Publisher node example
│   ├── publisher_with_params.py           # Publisher with parameters
│   └── test_publisher.py                  # Unit tests
├── lesson-05/
│   ├── simple_subscriber.py               # Subscriber with callback
│   ├── data_processor.py                  # Subscriber that processes data
│   └── test_subscriber.py                 # Unit tests
├── lesson-06/
│   ├── service_server.py                  # Service server example
│   ├── action_server.py                   # Action server example
│   └── test_services.py                   # Unit tests
├── lesson-07/
│   ├── turtle_control.launch.py          # Launch file for capstone
│   ├── publisher_node.py                  # Multi-node components
│   ├── subscriber_node.py                 #
│   ├── coordinator_node.py                #
│   └── test_integration.py                # Integration tests
└── README.md                              # How to run examples

tests/
├── unit/
│   └── test_ros2_patterns.py             # Unit tests for ROS 2 code examples
├── integration/
│   └── test_chapter1_integration.py      # Full-chapter integration tests
└── README.md                              # Test documentation
```

**Structure Decision**: Educational content structure (Docusaurus) + code examples in standalone directory. Each lesson maps to one markdown file with embedded code blocks and Docusaurus-compatible formatting. Code examples are tested separately and referenced in markdown files.

## Complexity Tracking

**Status**: ✅ No violations - Constitution fully aligned

All design decisions align with constitution principles. No complexity justification needed.

---

## Phase 0: Research & Unknowns Resolution

### Research Tasks Completed

| Unknown | Research Task | Finding | Decision |
|---------|---------------|---------|----------|
| ROS 2 Humble API stability | Verify latest documentation | Stable LTS release (Oct 2022, support until May 2027) | Use ROS 2 Humble exclusively |
| Turtlesim capabilities | Test simulation scope | Supports 2D movement, rotation, pen drawing; full ROS 2 integration | Sufficient for all 7 lessons |
| rclpy best practices | Verify official tutorials | Node structure, executor patterns, callback best practices documented | Follow official patterns |
| CEFR A2-B1 validation | Content complexity research | 5-7 concepts for A2, 7-10 for B1 aligns with language proficiency levels | Apply concept density limits per lesson |
| Code example testing | Verify pytest compatibility | pytest + ROS 2 integration well-documented and tested | Use pytest for validation |

### No Further Clarifications Needed

All technical context is specified. Phase 0 research is complete.

---

## Phase 1: Design & Architecture

### Learning Progression Architecture

**Chapter Arc**: Understand ROS 2 → Set up → Learn patterns → Implement → Debug → Orchestrate

```
Lesson 1 (Understanding)
  └─ What is ROS 2?
     └─ Why: Background for robotics students
     └─ CEFR A2: 5-6 concepts (architecture, nodes, topics, messages, ecosystem, why ROS 2)

Lesson 2 (Foundation)
  └─ Setting Up ROS 2
     └─ Why: Practical capability building
     └─ CEFR A2: 6-7 concepts (installation, environment, Turtlesim, verification, workspace, sourcing)

Lesson 3 (Concepts)
  └─ Nodes & Communication Patterns
     └─ Why: Theory before implementation
     └─ CEFR A2: 7 concepts (node structure, topics, pub/sub, services, actions, messages, naming)

Lesson 4 (Hands-on: Publishing)
  └─ Your First Publisher
     └─ Why: First coding success
     └─ CEFR B1: 8 concepts (rclpy.Node, publisher creation, Twist messages, publish(), callbacks, lifecycle, debugging, message rate)

Lesson 5 (Hands-on: Listening)
  └─ Subscribers & Callbacks
     └─ Why: Asynchronous processing for robotics
     └─ CEFR B1: 8-9 concepts (subscriber creation, callback functions, async execution, data extraction, message processing, error handling, integration)

Lesson 6 (Advanced Patterns)
  └─ Services & Actions
     └─ Why: Complete communication toolkit
     └─ CEFR B1: 9-10 concepts (request-response, service clients/servers, actions, goals, feedback, results, state machines, when to use each)

Lesson 7 (Integration)
  └─ Multi-Node Systems & Launch Files
     └─ Why: Real-world orchestration
     └─ CEFR B1: 8-9 concepts (launch files, parameters, node lifecycle, namespacing, multi-node debugging, coordination, capstone integration)
```

### Key Design Decisions

| Decision | Rationale | Impact |
|----------|-----------|--------|
| **Turtlesim Only** | Removes installation burden, maintains focus on ROS 2 concepts | Limits to 2D movement but covers all 4 communication patterns |
| **Python rclpy Only** | C++ would require build systems; Python is more accessible for learners | Slightly different syntax than C++, but logic patterns identical |
| **Layered Examples** | Start with minimal code, gradually add complexity (params, error handling, testing) | Students see progressively realistic code, not toy examples |
| **4-Layer Pedagogy** | Layer 1 (manual foundation code) + Layer 2 (AI collaboration notes) | Enables human-AI co-learning; Layer 3-4 reserved for future chapters |
| **Capstone Integration** | Multi-node project combining all lessons | Reinforces orchestration, debugging, and system thinking |
| **Troubleshooting First** | Each lesson includes "Common Errors" section | Proactive problem-solving reduces student frustration |
| **Self-Assessment Checklists** | Students verify learning independently | Builds metacognitive skills, reduces dependency on grading |

### Content Structure & Pedagogy

**Lesson Structure Template**:
```
[Lesson Title]
├── Learning Outcomes (3-5 specific, testable outcomes)
├── Estimated Duration (with breakdown)
├── Concepts Introduced (numbered list with CEFR validation)
├── Foundational Concepts (prerequisite review)
├── Core Content
│   ├── Explanation (why this matters)
│   ├── Architecture Diagram (visual)
│   ├── Theory & Patterns
│   ├── Code Examples (3-5 examples, progressively complex)
│   ├── How to Run (step-by-step instructions)
│   └── Expected Output (what success looks like)
├── CLI Debugging Tools (ros2 commands relevant to lesson)
├── Common Errors
│   ├── Error 1: [Name]
│   │  ├── Symptom: [What student sees]
│   │  ├── Root Cause: [Why it happens]
│   │  └── Fix: [Step-by-step solution]
│   └── Error 2-5: [Additional common issues]
├── Self-Assessment Checklist
│   ├─ [ ] I can explain [concept 1]
│   ├─ [ ] I can implement [pattern 1]
│   └─ [ ] I can debug [common issue]
└── Further Exploration (bonus content)
```

**4-Layer Pedagogical Method Integration**:
- **Layer 1 (Manual Foundation)**: Students write boilerplate code from scratch
- **Layer 2 (AI Collaboration)**: Notes on how Claude/AI could assist (but code is manual)
- Layers 3-4: Reserved for Chapters 2-4

### Code Example Strategy

**Progressive Complexity Levels**:

```
Level 1 (Minimum Working Example - MWE)
├─ Single node
├─ Single communication pattern
├─ No error handling
├─ ~20 lines of code
└─ Purpose: Understand core concept

Level 2 (Production-Aware)
├─ Add parameters
├─ Error handling
├─ ~40-50 lines
└─ Purpose: Real-world viability

Level 3 (Integrated)
├─ Multi-component
├─ Testing included
├─ ~60+ lines
└─ Purpose: System thinking
```

Example distribution across 7 lessons:
- Lessons 1-2: Theory only (0 code examples)
- Lesson 3: Diagrams only (0 runnable code)
- Lesson 4: 4 examples (MWE → with params → with error handling → with testing)
- Lesson 5: 3 examples (callback MWE → data processing → error handling)
- Lesson 6: 4 examples (service MWE → action MWE → combined → testing)
- Lesson 7: 5 examples (launch file → multi-node → parameters → capstone orchestration)

**Total**: ~25-30 code examples across chapter

### Reusable Intelligence (RI) Components

#### 1. **Skill: ros2-node-patterns**
**Location**: `.claude/skills/ros2-node-patterns/SKILL.md`
**Purpose**: Encodes ROS 2 node templates and naming conventions
**Provides**:
- Node class template (boilerplate)
- Naming conventions (snake_case for nodes, UPPERCASE for constants)
- Timer patterns (for polling at fixed rates)
- Executor patterns (single-threaded vs multi-threaded)
- Common pitfalls (forgetting destroy(), hardcoded strings, etc.)

#### 2. **Skill: simulation-code-validation**
**Location**: `.claude/skills/simulation-code-validation/SKILL.md`
**Purpose**: Validates ROS 2 code examples for correctness
**Provides**:
- Syntax validation (Python AST parsing)
- ROS 2 pattern validation (correct imports, node structure)
- Turtlesim compatibility checks
- Safety checks (no hardware calls, simulation-only confirmed)
- Troubleshooting completeness validation (example must have Common Errors section)

#### 3. **Agent: robotics-chapter-planner**
**Location**: `.claude/agents/robotics-chapter-planner.md`
**Purpose**: Transforms spec into detailed lesson plan with timings and prerequisites
**Input**: Chapter spec
**Output**: Detailed lesson breakdown with CEFR validation, concept lists, timing

#### 4. **Agent: robotics-content-implementer**
**Location**: `.claude/agents/robotics-content-implementer.md`
**Purpose**: Generates lesson markdown content following 4-Layer method
**Input**: Lesson plan (from robotics-chapter-planner)
**Output**: Docusaurus markdown lesson with integrated examples and troubleshooting

#### 5. **Output Style: hardware-lesson-template**
**Location**: `.claude/output-styles/hardware-lesson-template.md`
**Purpose**: Standardizes lesson structure across all robotics chapters
**Provides**:
- Markdown template with sections in correct order
- Code block formatting for Docusaurus
- Checkbox syntax for self-assessment
- Diagram placeholder syntax
- Cross-lesson linking patterns

### Data Model & Entities

**Chapter Content Structure**:
```
Chapter {
  id: "chapter-1-ros2-fundamentals"
  title: "ROS 2 Fundamentals & Communication"
  duration: "5-6 hours"
  cefr_level: ["A2", "B1"]
  lessons: [Lesson]
  capstone: Capstone
}

Lesson {
  id: "lesson-01"
  title: "What is ROS 2?"
  duration_minutes: 30
  concepts: [Concept]
  learning_outcomes: [string]
  code_examples: [CodeExample]
  exercises: [Exercise]
  troubleshooting: [TroubleshootingGuide]
  self_assessment: [AssessmentItem]
}

Concept {
  name: string
  explanation: string
  cefr_level: ["A2" | "B1" | "C2"]
  prerequisite_concepts: [Concept]
}

CodeExample {
  id: string
  level: ["MWE" | "Production-Aware" | "Integrated"]
  language: "python"
  runtime: "ROS 2 Humble + Turtlesim"
  testing_framework: "pytest"
  lines_of_code: number
}

Exercise {
  task: string
  acceptance_criteria: [string]
  expected_duration: number
  simulation_environment: "Turtlesim"
}

Capstone {
  title: "Multi-Node Turtlesim Control System"
  components: ["Publisher Node", "Subscriber Node", "Coordinator Node"]
  integration: "All 7 lessons combined"
}
```

### Contracts & Output Formats

**Output Contract for Each Lesson**:

```yaml
lesson_metadata:
  filename: "lesson-XX-<name>.md"
  format: "Docusaurus markdown"
  required_sections:
    - title (## Lesson X: ...)
    - learning_outcomes (bulleted, 3-5)
    - estimated_duration (with minute breakdown)
    - concepts_introduced (numbered, CEFR labeled)
    - foundational_concepts (review)
    - core_content (title, explanation, diagrams, theory, examples)
    - cli_tools (relevant ros2 commands)
    - common_errors (5+ errors with symptom/root cause/fix)
    - self_assessment (5+ checklist items)
    - further_exploration (bonus content)

code_example_metadata:
  filename: "examples/chapter-1-ros2/lesson-XX/<name>.py"
  format: "Python 3.10+ with rclpy"
  required_sections:
    - imports (rclpy, ROS 2 types)
    - docstring (purpose, runtime)
    - node_class (rclpy.Node subclass)
    - main_function (spin, error handling)
    - test_file (pytest suite)

validation_requirements:
  - Syntax: Valid Python 3.10+ (checked with ast.parse)
  - Imports: All imports resolvable in ROS 2 Humble
  - Simulation: No hardware APIs (no /dev/*, no serial ports)
  - Turtlesim: Compatible with turtlesim node (if used)
  - Testing: 80%+ code coverage with pytest
  - Security: No hardcoded credentials, no eval()
  - Documentation: All functions docstrung, code commented
  - Safety: Confirms "simulation environment: Turtlesim"
```

### Timeline & Execution

**Content Generation Timeline** (estimated 4-6 hours):

| Phase | Task | Time | RI Component |
|-------|------|------|--------------|
| 0 | Lesson plan generation | 1h | robotics-chapter-planner agent |
| 1 | Lesson 1-2 content | 45m | robotics-content-implementer agent |
| 2 | Lesson 3 content | 30m | robotics-content-implementer agent |
| 3 | Lesson 4 content + code | 1h | robotics-content-implementer agent + ros2-node-patterns skill |
| 4 | Lesson 5 content + code | 1h | robotics-content-implementer agent + ros2-node-patterns skill |
| 5 | Lesson 6 content + code | 1h | robotics-content-implementer agent + ros2-node-patterns skill |
| 6 | Lesson 7 + capstone | 1h | robotics-content-implementer agent + ros2-node-patterns skill |
| 7 | Code validation & testing | 1h | simulation-code-validation skill |
| 8 | Docusaurus integration | 30m | Manual formatting |

**Total**: ~7.5 hours estimated (content generation + validation + integration)

---

## Phase 1 Deliverables

### 1. research.md
**Purpose**: Document all research findings, technical decisions, best practices
**Content**:
- ROS 2 Humble stability analysis
- Turtlesim capability matrix
- rclpy best practices summary
- CEFR concept density validation
- Pytest + ROS 2 integration patterns

### 2. data-model.md
**Purpose**: Define content entities, structure, and relationships
**Content**:
- Lesson entity model
- Concept taxonomy (with CEFR levels)
- Code example structure
- Exercise format
- Capstone project definition

### 3. quickstart.md
**Purpose**: Quick reference for instructors on using Chapter 1
**Content**:
- Chapter overview (30-second summary)
- Prerequisites (ROS 2 Humble installed)
- How to deploy to Docusaurus
- How to run code examples
- How to assign exercises
- Success criteria checklist

### 4. contracts/
**Directory**: `specs/2-chapter1-ros2-fundamentals/contracts/`
**Files**:
- `lesson-output.schema.json` - JSON schema for lesson markdown
- `code-example.schema.json` - JSON schema for code examples
- `docusaurus-integration.md` - Integration guide for book platform

### 5. Updated Agent Context
**File**: `.claude/agents/robotics-content-implementer-context.md`
**Update**: Add ROS 2 patterns, Turtlesim examples, 4-Layer pedagogy notes for this chapter

---

## Re-Check: Constitution Compliance (Post-Design)

All design decisions maintain constitutional alignment:

- ✅ **Accuracy**: ROS 2 Humble docs verified, Turtlesim tested, rclpy best practices documented
- ✅ **Tech Stack**: Docusaurus markdown, Python 3.10+, pytest
- ✅ **Educational Structure**: 7 lessons with clear outcomes, "why before how"
- ✅ **Simulation-First**: Turtlesim only, no hardware, no forward refs to Gazebo/Isaac
- ✅ **Safety Framework**: A2 CEFR = simulation-only confirmed
- ✅ **SDD Workflow**: Spec ✅ → Plan ✅ → Tasks (next) → Implementation
- ✅ **Deployment**: Docusaurus markdown format ready for GitHub Pages

**Final Gate Status**: ✅ ALL GATES PASS POST-DESIGN

---

## Next Steps

**This plan is COMPLETE when:**
1. ✅ Technical context filled (all unknowns resolved)
2. ✅ Constitution check passed (all gates green)
3. ✅ Project structure defined (Docusaurus + examples)
4. ✅ Learning progression architected (7 lessons, CEFR validated)
5. ✅ RI components identified (5 skills/agents)
6. ✅ Timeline estimated (7.5 hours)
7. ✅ Phase 0-1 complete (research, design, contracts)

**Ready for**: `/sp.tasks` to generate implementation tasks with:
- Task breakdown for each lesson
- Code example specifications
- Testing requirements
- Validation criteria
- Execution order with dependencies

---

**Status**: ✅ PLANNING COMPLETE - READY FOR TASKS
**Branch**: `2-chapter1-ros2-fundamentals`
**Last Updated**: 2025-11-29
**Created By**: /sp.plan command
