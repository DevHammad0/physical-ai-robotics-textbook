# /sp.robotics-chapter

Autonomous workflow command for generating complete robotics chapters using Spec-Driven Development with Reusable Intelligence (SDD-RI).

## Purpose

This command orchestrates the full chapter generation pipeline:

```
User Request → /sp.robotics-chapter [chapter_num] "[chapter_title]"
                    ↓
                [SPECIFY PHASE]
                Clarify requirements
                    ↓
                [PLAN PHASE]
                robotics-chapter-planner agent
                Lesson-by-lesson architecture
                    ↓
                [TASKS PHASE]
                Break into implementation tasks
                    ↓
                [GATES]
                User approval at each stage
                    ↓
                [IMPLEMENT PHASE]
                robotics-content-implementer agent
                Generate all lessons
                    ↓
                [VALIDATE PHASE]
                Docusaurus rendering + quality checks
                    ↓
                Chapter Complete ✅
```

## Usage

```bash
# Generate a new chapter
/sp.robotics-chapter 2 "Gazebo Simulation and Physics"

# Generate chapter with custom duration
/sp.robotics-chapter 3 "Sensor Integration with ROS 2" --duration 6 --difficulty intermediate
```

## Workflow Stages

### STAGE 1: SPECIFY (Clarify Requirements)

**Trigger**: User invokes `/sp.robotics-chapter`

**Agent**: (Claude interviewer, no specialized agent)

**Action**: Ask 3-5 clarifying questions:
1. **Hardware Scope**: What hardware will be simulated? (sensors, actuators, robot structure)
2. **Simulation Environment**: Which simulator(s)? (Turtlesim, Gazebo, Isaac Sim, MuJoCo)
3. **Pedagogical Prerequisites**: What prior knowledge assumed? (Chapter 1 ROS 2 fundamentals? Specific skills?)
4. **Learning Outcomes**: What should students be able to DO by end?
5. **Bonus Features**: AI collaboration prompts? Debugging exercises? Multi-node systems?

**Output**: Specification summary (draft spec.md ready for review)

**Gate**: User approves specification before moving to planning

---

### STAGE 2: PLAN (Architecture Design)

**Trigger**: User says "OK" or "approved"

**Agent**: `robotics-chapter-planner` (specialized agent)

**Agent Instructions**:
- Load constitution Section IX (Physical AI Safety Framework)
- Apply robotics curriculum design questions (6 critical questions from agent instructions)
- Validate CEFR cognitive load
- Map 4-Layer pedagogical progression
- Determine simulation environment(s)
- Identify hardware prerequisites
- Suggest lesson breakdown (5-8 lessons, 45-90 min each)

**Agent Outputs**:
- `specs/chapter-N-slug/plan.md` with:
  - Chapter planning analysis (simulation tier, hardware prereqs, safety progression, CEFR validation, 4-Layer mapping)
  - Lesson-by-lesson architecture (title, duration, layer, concepts, content outline)
  - Skill dependencies (what prior chapters needed)
  - Constitutional compliance checklist

**Gate**: User reviews plan and confirms lesson structure before tasking

---

### STAGE 3: TASKS (Break Into Actionable Items)

**Trigger**: User approves plan

**Agent**: (Claude task-breaker, structured prompt)

**Action**: Transform plan into tasks.md with:
- Phase-by-phase breakdown (Phase 1: Foundation, Phase 2: Collaboration, etc.)
- Per-lesson tasks with acceptance criteria
- Per-task content checklist
- Validation steps
- Timeline estimates
- Risk mitigation

**Output**: `specs/chapter-N-slug/tasks.md` with all implementation work itemized

**Gate**: User confirms tasks are clear and complete before implementation

---

### STAGE 4: IMPLEMENT (Generate Lessons)

**Trigger**: User says "implement" or "go"

**Agent**: `robotics-content-implementer` (specialized agent)

**Agent Instructions**:
- For each lesson in tasks.md:
  1. Load constitution Section IX (Physical AI Safety)
  2. Load `ros2-node-patterns` skill (if ROS 2 content)
  3. Load `simulation-code-validation` skill
  4. Generate lesson file (hardware-lesson-template.md)
  5. Run simulation-code-validation on all code examples
  6. Validate pedagogical safety (no forward refs, CEFR OK, Three Roles present)
  7. Create lesson file at correct path
- After all lessons:
  1. Create chapter README.md
  2. Update Docusaurus sidebars.ts
  3. Run Docusaurus build (pnpm run build)
  4. Verify no broken links

**Skills Used**:
- `ros2-node-patterns` (for ROS 2 code generation)
- `simulation-code-validation` (for code quality + pedagogical safety)
- `hardware-lesson-template` (for lesson structure + YAML frontmatter)

**Output**:
- `book-source/docs/chapter-N-slug/` directory with:
  - `README.md` (chapter overview)
  - `01-lesson-name.md` (Lesson 1)
  - `02-lesson-name.md` (Lesson 2)
  - ... (all lessons)
- Updated `book-source/sidebars.ts`

**Gate**: User checks Docusaurus rendering before publishing

---

### STAGE 5: VALIDATE (Quality Assurance)

**Trigger**: Agent finishes implementation

**Validation Checklist**:
- [ ] All lesson files exist and are readable
- [ ] All YAML frontmatter valid (no missing fields)
- [ ] All code examples have expected outputs
- [ ] All code syntax valid (Python linter passed)
- [ ] All ROS 2 patterns correct (rclpy.init, spin, shutdown)
- [ ] All troubleshooting sections complete (3+ errors per code block)
- [ ] No forward references to future lessons
- [ ] CEFR concept count OK for each lesson
- [ ] Three Roles framework present (💬 CoLearning prompts)
- [ ] Docusaurus builds without errors (`pnpm run build`)
- [ ] Chapter renders in sidebar navigation
- [ ] All links internal (to other lessons) work
- [ ] Mobile-responsive (tested on phone viewport)
- [ ] Constitutional compliance verified (simulation-first, safety progression, no meta-commentary)

**Output**: Validation report with pass/fail for each item

**Gate**: If all pass → Chapter ready for students. If fail → Identify issues and re-implement.

---

## Example Invocation: Chapter 2 (Gazebo)

```
User: /sp.robotics-chapter 2 "Gazebo Simulation and Physics"

[SPECIFY STAGE]
Claude: I have a few questions before we start planning Chapter 2:
1. Hardware Scope: Will we simulate robots with multiple joints? Sensors like Lidar/Camera?
2. Simulation: Gazebo is the obvious choice. Should we compare to Isaac Sim?
3. Prerequisites: Should students have completed Chapter 1 (ROS 2 Fundamentals)?
4. Learning Outcomes: By end, can students write URDF, launch Gazebo, read sensor data?
5. Bonus: Should we include AI collaboration exercises for debugging Gazebo physics?

User: All yes except #2 - stick with Gazebo only. And yes to debugging exercises.

[PLAN STAGE]
robotics-chapter-planner agent analyzes:
- Hardware: Simple 2-DOF arm (2 joints, Lidar, camera) - medium complexity
- Simulation: Gazebo with physics
- Prerequisites: Chapter 1 ROS 2 (nodes, topics, publishers/subscribers)
- Safety: B1 level (benchtop testing tier, can mention hardware setup but not deployment)
- CEFR: B1 - 7-10 concepts per lesson (URDF, SDF, joints, sensors, controllers, physics parameters, etc.)
- 4-Layer: L1 (URDF manual explanation), L2 (AI collaboration on Gazebo debugging)
- Lesson Structure:
  * Lesson 1: Understanding URDF (45 min, L1)
  * Lesson 2: Building Your First Robot (60 min, L1→L2)
  * Lesson 3: Physics and Joints (60 min, L2)
  * Lesson 4: Adding Sensors to Your Robot (60 min, L2)
  * Lesson 5: Gazebo Controllers and Actuation (60 min, L2)
  * Lesson 6: Multi-Robot Systems in Gazebo (90 min, L2→L3)

Output: specs/chapter-2-gazebo/plan.md

[TASKS STAGE]
Breakdown into tasks.md:
- Task 2.1: Lesson 1 - Understanding URDF (no code, conceptual)
- Task 2.2: Lesson 2 - Build robot with simple URDF
- Task 2.3: Lesson 3 - Physics parameters and joint dynamics
- Task 2.4: Lesson 4 - Add Lidar/Camera sensors
- Task 2.5: Lesson 5 - Gazebo controllers (joint position control)
- Task 2.6: Lesson 6 - Multi-robot coordination
- Task 2.7: Chapter README and Docusaurus integration

Output: specs/chapter-2-gazebo/tasks.md

[IMPLEMENT STAGE]
robotics-content-implementer agent generates:
- For each lesson:
  * Load robot-urdf-patterns skill (NEW - created from Ch2 patterns)
  * Load simulation-code-validation skill
  * Generate lesson markdown with YAML frontmatter
  * Validate all code examples (URDF syntax, launch files, Python scripts)
  * Validate pedagogical safety (no forward refs, CEFR 7-10 concepts, Three Roles)
  * Create file at book-source/docs/chapter-2-gazebo/NN-lesson-name.md

Output:
- book-source/docs/chapter-2-gazebo/README.md
- book-source/docs/chapter-2-gazebo/01-understanding-urdf.md
- book-source/docs/chapter-2-gazebo/02-building-first-robot.md
- ... (all 6 lessons)
- Updated book-source/sidebars.ts

[VALIDATE STAGE]
Check:
- [ ] All 6 lessons exist
- [ ] All code examples have expected outputs
- [ ] URDF syntax valid
- [ ] Gazebo physics explained
- [ ] No forward references
- [ ] CEFR concepts OK (7-10 per lesson)
- [ ] Three Roles framework visible (💬 prompts)
- [ ] Docusaurus builds
- [ ] Mobile responsive
- [ ] Constitutional compliance ✅

Output: Validation report - PASS ✅

User sees: Chapter 2 is ready for students!
```

---

## Command Options

```bash
/sp.robotics-chapter [chapter_num] "[chapter_title]"
  --duration [hours]           # Estimated total chapter duration (default: 4)
  --difficulty [beginner|intermediate|advanced]  # CEFR level (default: depends on chapter)
  --simulator [turtlesim|gazebo|isaac|mujoco]   # Primary simulator
  --skip-implement             # Only do SPECIFY+PLAN+TASKS (no implementation)
  --no-validate                # Skip final validation stage
  --output-dir [path]          # Override default specs/chapter-N-slug
```

---

## Integration with Existing Workflow

This command is designed to chain with other `/sp.*` commands:

```
/sp.robotics-chapter 2 "Gazebo"     # Generate Chapter 2
  → Creates spec.md, plan.md, tasks.md
  → Implements all lessons
  → Generates phr automatically

/sp.phr                             # (Optional) Record this planning session
  → Creates history/prompts/chapter-2-gazebo/001-gazebo-planning.spec.prompt.md

/sp.adr "Gazebo vs Isaac Sim"       # (Optional) If architectural decision needed
  → Documents why Gazebo chosen over Isaac for B1
```

---

## Exit Conditions

**Command completes successfully when**:
- All lesson files generated
- All code examples validated
- Docusaurus builds without errors
- Constitutional compliance verified
- Validation checklist all green ✅

**Command stops with error if**:
- Code syntax invalid (fixable by agent, retry)
- Forward references detected (requires user input on what to remove)
- CEFR concept count exceeded (requires user guidance on scope)
- Docusaurus build fails (requires investigation of markdown syntax)

**User should re-invoke with clarifications**:
```
/sp.robotics-chapter 2 "Gazebo" --skip-lesson 6  # Skip advanced multi-robot lesson
```

---

## Success Criteria

Chapter generated by this command meets:

- [ ] Complete spec.md with requirements
- [ ] Detailed plan.md with lesson architecture
- [ ] Actionable tasks.md with acceptance criteria
- [ ] All lessons generated and valid
- [ ] All code examples tested and verified
- [ ] Constitutional compliance verified
- [ ] Docusaurus renders correctly
- [ ] Mobile-responsive design confirmed
- [ ] Zero broken links or missing assets
- [ ] Ready for student use

---

## Related Commands

- `/sp.specify` - Manual specification workflow (if you want full control)
- `/sp.plan` - Manual planning workflow
- `/sp.tasks` - Manual task generation
- `/sp.implement` - Manual implementation workflow
- `/sp.phr` - Record this work session
- `/sp.adr` - Document architectural decisions

---

## Notes for Users

1. **Automation with Guardrails**: This command automates most of the work but pauses at gates for user approval. You control the direction; the agents execute.

2. **Reusable Skills**: As chapters are generated, new skills are extracted (robot-urdf-patterns from Gazebo, sensor-reading-patterns from sensor chapter, etc.). These skills speed up future chapters.

3. **Constitution-First**: Every action checked against constitution Section IX (Physical AI Safety). You can't accidentally generate unsafe content.

4. **Fast Iteration**: If a chapter doesn't meet your standards, you can retry with `--skip-implement` and adjust the plan before implementation.

5. **Bonus Points**: Each reusable skill extracted earns bonus points. Track extracted skills in a SKILLS_MANIFEST.md file.

