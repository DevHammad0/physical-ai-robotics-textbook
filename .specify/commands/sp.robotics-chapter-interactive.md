# /sp.robotics-chapter-interactive

**Spec-Driven Development Pipeline for Robotics Chapters**

Executes chapter generation with user gates at each stage. Each stage completes, shows results, waits for approval before proceeding.

## Usage

```bash
/sp.robotics-chapter-interactive [chapter_num] "[chapter_title]"
```

## Pipeline Execution

### Stage 1: SPECIFY

**Purpose**: Clarify requirements and create specification

**Command Executed**:
```bash
/sp.specify
```

**Input Prompt** (generated from chapter description):
```
You are creating a robotics textbook chapter specification.

Chapter: [chapter_num]
Title: [chapter_title]
Purpose: [inferred from title]

Use the specification template to create a detailed spec that includes:
1. Chapter overview and target audience
2. Learning outcomes (4-6 SMART objectives)
3. User stories with acceptance criteria
4. Content scope (in scope / out of scope)
5. Technical requirements
6. Proposed lesson breakdown
7. Constitutional compliance checklist

CONSTRAINT: Must follow constitution Section IX (Physical AI Safety)
CONSTRAINT: A2 level = simulation-only, B1 level = benchtop testing, C2 = field deployment
CONSTRAINT: Code examples must specify simulation environment (Turtlesim/Gazebo/Isaac)
```

**Output**: `specs/chapter-[num]-[slug]/spec.md`

**User Gate**:
```
═══════════════════════════════════════════════════════════════
STAGE 1 COMPLETE: Specification Generated
═══════════════════════════════════════════════════════════════

Review the specification above:
- Learning outcomes clear and measurable?
- Lessons well-sequenced?
- Constitutional requirements met?
- Scope appropriate for chapter difficulty?

Do you approve this specification?
Options: [yes] [no - provide feedback] [skip this stage]

👉 Enter: yes/no/feedback
```

**If NO**: System asks for feedback, regenerates spec with constraints
**If YES**: Proceed to Stage 2 (PLAN)
**If SKIP**: Jump to pre-existing stage (if available)

---

### Stage 2: PLAN

**Purpose**: Design lesson-by-lesson architecture

**Command Executed**:
```bash
/sp.plan
```

**Input Prompt** (generated from spec.md):
```
You are the robotics-chapter-planner agent.

Your task: Create a detailed plan for Chapter [num]: [title]

Input Specification:
[contents of spec.md from Stage 1]

Use the plan template to:
1. Analyze chapter type (conceptual vs. code-focused)
2. Determine simulation environment(s) (Turtlesim/Gazebo/Isaac/MuJoCo)
3. Map hardware prerequisites
4. Validate CEFR cognitive load (A2: 5-7 concepts/lesson, B1: 7-10, C2: unlimited)
5. Validate 4-Layer progression (L1: foundation, L2: collaboration, L3: intelligence, L4: orchestration)
6. Design lesson sequence with learning outcomes for each
7. Identify skill dependencies and enablements

MANDATORY CHECKS:
- Simulation-first: ✅ (no hardware until appropriate level)
- Safety progression: ✅ (A2→B1→C2 progression clear)
- No forward references: ✅ (lessons don't mention future concepts)
- Constitutional compliance: ✅ (verified)

Output: specs/chapter-[num]-[slug]/plan.md
```

**Output**: `specs/chapter-[num]-[slug]/plan.md`

**User Gate**:
```
═══════════════════════════════════════════════════════════════
STAGE 2 COMPLETE: Lesson Architecture Planned
═══════════════════════════════════════════════════════════════

Review the plan above:
- Lesson sequence logical?
- CEFR concept count appropriate?
- 4-Layer progression clear?
- Simulation environments specified?
- Safety progression correct?

Do you approve this plan?
Options: [yes] [no - provide feedback] [regenerate without stage 3]

👉 Enter: yes/no/feedback
```

**If NO**: System asks for feedback, regenerates plan
**If YES**: Proceed to Stage 3 (TASKS)

---

### Stage 3: TASKS

**Purpose**: Break lessons into actionable implementation tasks

**Command Executed**:
```bash
/sp.tasks
```

**Input Prompt** (generated from plan.md):
```
You are creating implementation tasks for Chapter [num]: [title]

Input Plan:
[contents of plan.md from Stage 2]

Use the tasks template to:
1. Break each lesson into concrete tasks
2. Add acceptance criteria for each task
3. Create content checklists
4. Identify validation steps
5. Estimate time for each task
6. Flag dependencies and risks

Output Format:
- Phase 1: [lesson group] - Tasks [X.1], [X.2], etc.
  * Task [X.Y]: [lesson name]
    - [ ] Content item 1
    - [ ] Content item 2
    - Acceptance criteria: [measurable checklist]
    - Time estimate: [hours]

CONSTRAINTS:
- Each lesson = separate task
- Code examples must have expected output
- Troubleshooting section = mandatory
- Self-assessment checklist = mandatory

Output: specs/chapter-[num]-[slug]/tasks.md
```

**Output**: `specs/chapter-[num]-[slug]/tasks.md`

**User Gate**:
```
═══════════════════════════════════════════════════════════════
STAGE 3 COMPLETE: Implementation Tasks Defined
═══════════════════════════════════════════════════════════════

Review the tasks above:
- Tasks granular and actionable?
- Acceptance criteria clear?
- Content checklists complete?
- Time estimates reasonable?

Do you approve these tasks?
Options: [yes] [no - provide feedback] [proceed anyway]

👉 Enter: yes/no/feedback
```

**If NO**: System asks for feedback, regenerates tasks
**If YES**: Proceed to Stage 4 (IMPLEMENT)

---

### Stage 4: IMPLEMENT

**Purpose**: Generate all lesson files based on tasks

**Command Executed**:
```bash
/sp.implement
```

**Input Prompt** (generated from tasks.md):
```
You are the robotics-content-implementer agent.

Your task: Generate all lessons for Chapter [num]: [title]

Load before starting:
- Constitution (Section IX - Physical AI Safety)
- Skill: ros2-node-patterns (if ROS 2 chapter)
- Skill: simulation-code-validation
- Template: hardware-lesson-template.md

Input Tasks:
[contents of tasks.md from Stage 3]

For each task:
1. Generate lesson markdown file
2. Include YAML frontmatter with:
   - title, sidebar_position, chapter, lesson, lesson_slug
   - duration_minutes, estimated_difficulty, simulation_required
   - safety_level, cefr_level, concepts, learning_objectives
   - assessment_type, keywords
3. Structure lesson with:
   - Introduction
   - Layer 1 (manual foundation) OR prerequisite overview
   - Layer 2 (AI collaboration) with 💬 CoLearning prompts
   - Code examples (all tested, with expected output)
   - 🤝 Collaboration exercises
   - Troubleshooting (3+ common errors per code block)
   - Self-assessment checklist
   - Next steps and resources
4. Validate using simulation-code-validation skill:
   - Python syntax correct
   - ROS 2 patterns valid
   - Message types verified
   - Expected outputs realistic
   - No forward references
   - CEFR concept count OK
   - Three Roles framework present
5. Create file at: book-source/docs/chapter-[num]-[slug]/[NN]-lesson-name.md

OUTPUT: All lesson files created and validated

MANDATORY COMPLIANCE:
- Constitution Section IX: ✅ (simulation-first, safety progression)
- CEFR limits: ✅ (concept count within range)
- 4-Layer method: ✅ (foundation → collaboration → intelligence → orchestration)
- Three Roles: ✅ (AI as Teacher/Student/Co-Worker, framework invisible)
- Code quality: ✅ (tested, PEP 8, expected outputs shown)
- Troubleshooting: ✅ (3+ errors per code block)
- Self-assessment: ✅ (checkbox lists present)
```

**Output**: `book-source/docs/chapter-[num]-[slug]/*.md` (all lesson files)

**User Gate**:
```
═══════════════════════════════════════════════════════════════
STAGE 4 COMPLETE: All Lessons Generated
═══════════════════════════════════════════════════════════════

Generated Files:
- book-source/docs/chapter-[num]-[slug]/README.md
- book-source/docs/chapter-[num]-[slug]/01-lesson-name.md
- book-source/docs/chapter-[num]-[slug]/02-lesson-name.md
- ... (all lessons)

Next: Build and validate in Docusaurus?

Do you want to:
[a] Build Docusaurus and validate (Stage 5)
[b] Review lessons manually first (pause here)
[c] Skip validation and mark complete

👉 Enter: a/b/c
```

**If b**: User manually reviews lessons, then comes back to approve stage 5
**If a**: Proceed to Stage 5 (BUILD & VALIDATE)
**If c**: Skip to Stage 6 (PHR creation)

---

### Stage 5: BUILD & VALIDATE

**Purpose**: Build Docusaurus and verify all files render correctly

**Command Executed**:
```bash
cd book-source && pnpm run build
```

**Validation Checks**:
- ✅ Docusaurus build succeeds (no errors)
- ✅ All lesson files parsed
- ✅ Sidebar navigation includes chapter
- ✅ Next/Previous links work
- ✅ Mobile responsive
- ✅ No broken internal links

**Output**: `book-source/build/` directory with static files

**User Gate**:
```
═══════════════════════════════════════════════════════════════
STAGE 5 COMPLETE: Docusaurus Build Successful
═══════════════════════════════════════════════════════════════

Build Results:
✅ Compiled successfully in 29.11s
✅ All 6 lessons recognized
✅ Sidebar navigation working
✅ Mobile responsive
⚠️  [warnings if any]

Chapter ready for deployment?

[yes] Mark complete and create PHR
[no - need fixes] Go back to Stage 4

👉 Enter: yes/no
```

**If no**: System marks which lesson needs fixes, returns to Stage 4 with feedback
**If yes**: Proceed to Stage 6 (PHR CREATION)

---

### Stage 6: PHR CREATION (Automatic)

**Purpose**: Record this entire workflow in Prompt History Record

**File Created**:
```
history/prompts/chapter-[num]-[slug]/001-chapter-[num]-specification.spec.prompt.md
```

**PHR Contents**:
```yaml
---
id: "001"
title: "Chapter [num] Specification: [Title]"
stage: "spec"
date_iso: "[YYYY-MM-DD]"
surface: "agent"
model: "[current model]"
feature: "chapter-[num]-[slug]"
branch: "main"
user: "[user name or 'automated']"
command: "/sp.robotics-chapter-interactive [num] '[title]'"
labels:
  - "robotics"
  - "chapter-[num]"
  - "spec-driven"
links:
  spec: "specs/chapter-[num]-[slug]/spec.md"
  plan: "specs/chapter-[num]-[slug]/plan.md"
  tasks: "specs/chapter-[num]-[slug]/tasks.md"
  adr: null
  pr: null
files:
  - specs/chapter-[num]-[slug]/spec.md
  - specs/chapter-[num]-[slug]/plan.md
  - specs/chapter-[num]-[slug]/tasks.md
  - book-source/docs/chapter-[num]-[slug]/README.md
  - book-source/docs/chapter-[num]-[slug]/01-*.md
  - book-source/docs/chapter-[num]-[slug]/02-*.md
  - ... (all lesson files)
tests:
  - "Docusaurus build successful"
  - "All lesson files render"
  - "Navigation links work"
  - "Mobile responsive"
---

## Prompt

User requested: "Generate Chapter [num]: [Title]"

## Response

Chapter [num] fully generated:
- Specification created and approved
- Lesson plan created and approved
- Implementation tasks created and approved
- All 6 lessons generated and validated
- Docusaurus build successful
- Chapter ready for students

## Outcome

✅ Chapter [num] complete
- [X] lessons written
- [X] code examples tested
- [X] Docusaurus renders correctly
- [X] Constitutional compliance verified
- ✅ READY FOR STUDENTS
```

**User Sees**:
```
═══════════════════════════════════════════════════════════════
PIPELINE COMPLETE: Chapter [num] Generated
═══════════════════════════════════════════════════════════════

✅ All 6 Stages Complete:
  [1] ✅ Specify - spec.md created and approved
  [2] ✅ Plan - plan.md created and approved
  [3] ✅ Tasks - tasks.md created and approved
  [4] ✅ Implement - All lessons generated and validated
  [5] ✅ Build - Docusaurus renders correctly
  [6] ✅ PHR - Workflow recorded

PHR Created:
  📄 history/prompts/chapter-[num]-[slug]/001-chapter-[num]-specification.spec.prompt.md

Chapter Status: READY FOR STUDENTS 🚀

Next Steps:
  - Test with students
  - Collect feedback
  - Generate Chapter [num+1] using: /sp.robotics-chapter-interactive [num+1] "[title]"
```

---

## State Management

The pipeline maintains state so you can:

**Pause and Resume**:
```bash
# You approve spec, want to review plan before approving
/sp.robotics-chapter-interactive 1 "ROS 2" --stage plan --skip-spec
# Skips directly to plan stage, using existing spec.md
```

**Regenerate a Stage**:
```bash
# You don't like the plan, want to regenerate
/sp.robotics-chapter-interactive 1 "ROS 2" --stage plan --regenerate
# Regenerates plan.md with feedback, keeps spec.md
```

**Force Through**:
```bash
# You want to skip approval gates and auto-approve
/sp.robotics-chapter-interactive 1 "ROS 2" --auto-approve
# Executes all stages without user gates (for CI/CD)
```

---

## Error Recovery

**If a stage fails**:

```
STAGE 4 ERROR: Lesson 3 code syntax invalid

Error: Python syntax error in 03-first-publisher.md
  Line 45: Unexpected indent

Recovery Options:
  [a] Regenerate Lesson 3 only
  [b] Regenerate all lessons with feedback
  [c] Manually fix and continue
  [d] Abort and go back to Stage 3

👉 Enter: a/b/c/d
```

**If you abort**: All generated files preserved, can resume with:
```bash
/sp.robotics-chapter-interactive 1 "ROS 2" --stage 4 --lessonnum 3
```

---

## Success Criteria for Pipeline

✅ **Stage 1 (Specify)**: spec.md created with all required sections
✅ **Stage 2 (Plan)**: plan.md with lesson architecture, CEFR validated, 4-Layer mapped
✅ **Stage 3 (Tasks)**: tasks.md with actionable tasks, acceptance criteria, time estimates
✅ **Stage 4 (Implement)**: All lesson files created, code validated, pedagogical safety checked
✅ **Stage 5 (Build)**: Docusaurus builds, sidebar shows chapter, mobile responsive
✅ **Stage 6 (PHR)**: Workflow recorded for traceability and learning

---

## What This Achieves

1. **True Spec-Driven Development**:
   - Each stage builds on previous stage
   - No skipping steps
   - User gates ensure quality

2. **Reusable Intelligence**:
   - Agents formally invoked
   - Skills loaded and used
   - Templates applied consistently

3. **Traceability**:
   - PHR records entire workflow
   - Can reference back to decisions
   - Learning captured

4. **Quality Assurance**:
   - Validation at each stage
   - Constitutional compliance checked
   - Code tested before deployment

5. **Scalability**:
   - Same pipeline for Chapter 1, 2, 3, ... 13
   - Skills accumulate and speed up later chapters
   - Process repeatable and consistent

