# Lesson 8: Capstone Project - Your VLA System Design

## Learning Objectives

By the end of this lesson, you will:
1. Design a complete Vision-Language-Action system from requirements to deployment
2. Integrate ≥3 pipeline components (voice, vision, planning, manipulation, safety)
3. Validate your system against well-defined success criteria
4. Document architectural decisions and tradeoffs
5. Deploy and demonstrate your system in Isaac Sim
6. Present your design to technical audience

## Capstone Project Overview

This is the **culminating project** where you apply everything learned in Lessons 1-7.

### Project Structure

Your project should include:

1. **Problem Statement** (1-2 pages)
   - What problem are you solving?
   - Why is it interesting?
   - What are the constraints?

2. **System Architecture** (2-3 pages)
   - High-level diagram (voice → perception → planning → action)
   - Which components are you using?
   - How do they communicate?
   - What are failure modes?

3. **Implementation** (code + docs)
   - ROS 2 nodes implementing your design
   - Launch files
   - Configuration files
   - Unit tests for key components

4. **Validation** (test results + analysis)
   - Success metrics (latency, accuracy, robustness)
   - Test scenarios (happy path + failure cases)
   - Performance benchmarks
   - Comparison to baseline

5. **Presentation** (live demo + slides)
   - 10-minute demo showing system working
   - 5-minute presentation on design decisions
   - 5-minute Q&A

## Project Ideas

### Idea 1: Autonomous Object Organization

**Problem**: Sort and organize objects on a table by color/type

**Components**: Voice + Vision + Planning + Manipulation + Safety

**Pipeline**:
```
User: "Organize the table"
   ↓
Vision: Detect all objects, classify by color
   ↓
Planning: Generate sequence of moves (sort by color zones)
   ↓
Manipulation: Execute pick-and-place for each object
   ↓
Safety: Monitor gripper force, workspace bounds
   ↓
Result: Objects organized into color groups
```

**Success Criteria**:
- ✓ Detect ≥5 objects in scene
- ✓ Classify color with ≥85% accuracy
- ✓ Sort 5 objects in &lt;30 seconds
- ✓ Zero gripper force violations
- ✓ Complete without human intervention

**Difficulty**: Medium (uses all 5 components)

### Idea 2: Collaborative Manipulation

**Problem**: Robot assists human with two-hand manipulation task (e.g., holding object while human works)

**Components**: Vision + Manipulation + Safety (high priority)

**Pipeline**:
```
Vision: Track human hand position
   ↓
Planning: Maintain stable grip while human works
   ↓
Manipulation: Adjust grip dynamically
   ↓
Safety: Detect human proximity, reduce force
   ↓
Result: Safely assist human task
```

**Success Criteria**:
- ✓ Track human hand at 10 Hz
- ✓ Maintain object position ±5cm
- ✓ Reduce force when human nearby
- ✓ Release immediately on E-stop

**Difficulty**: Hard (requires real-time force control + human interaction)

### Idea 3: Mobile Manipulation Task

**Problem**: Robot navigates to location and manipulates object

**Components**: Voice + Vision + Planning + Manipulation

**Pipeline**:
```
User: "Go get the cup from the kitchen table"
   ↓
Planning: Plan navigation path + pick-and-place
   ↓
Navigation: Move to kitchen table
   ↓
Vision: Locate cup
   ↓
Manipulation: Pick up cup
   ↓
Navigation: Return to user
   ↓
Result: Cup delivered to user
```

**Success Criteria**:
- ✓ Navigate to target location
- ✓ Locate and identify cup
- ✓ Pick and carry object
- ✓ Avoid obstacles

**Difficulty**: Hard (requires navigation + manipulation integration)

### Idea 4: Learning from Demonstration

**Problem**: Robot learns to perform new task from human demonstration

**Components**: Voice + Vision + Planning (custom implementation)

**Pipeline**:
```
Human demonstrates task 3x
   ↓
Vision + IMU: Record demonstration trajectory
   ↓
Learning: Extract common pattern
   ↓
Planning: Generate executable plan
   ↓
Execution: Execute learned task
   ↓
Feedback: Human validates
   ↓
Result: Robot learned new task
```

**Success Criteria**:
- ✓ Record 3 demonstrations
- ✓ Extract generalizable pattern
- ✓ Execute with similar success rate
- ✓ Adapt to object location variation

**Difficulty**: Very Hard (machine learning component)

## Capstone Project Template

Use this template to structure your project:

### 1. Project Proposal

```markdown
# Project Proposal: [Your Project Name]

## Problem Statement
[1-2 paragraphs describing the problem you're solving and why it matters]

## Solution Overview
[High-level description of your approach]

## Components Used
- [ ] Voice Input (Whisper)
- [ ] Vision (YOLO + Segmentation)
- [ ] LLM Planning
- [ ] Manipulation (Grasp + Motion)
- [ ] Safety Validation

## Success Criteria
1. [Measurable criterion 1]
2. [Measurable criterion 2]
3. [Measurable criterion 3]

## Timeline
- Week 1: [Milestone 1]
- Week 2: [Milestone 2]
- Week 3: [Final integration + testing]
```

### 2. System Design Document

```markdown
# System Design Document

## Architecture Overview
[Diagram showing all components and data flow]

## Components
### Component 1: [Name]
- **Responsibility**: [What does it do?]
- **Inputs**: [What topics/services does it subscribe to?]
- **Outputs**: [What does it publish?]
- **Failure modes**: [What can go wrong?]

... (repeat for each component)

## Data Structures
[JSON examples of key message types]

## Error Handling
[How does system handle failures?]

## Performance Targets
| Metric | Target | Measured |
|--------|--------|----------|
| [Metric 1] | X | Y |
| [Metric 2] | X | Y |

## Trade-offs
[What design decisions did you make and why?]
- **Choice A vs B**: [Pros and cons]
- **Accuracy vs Speed**: [How did you balance?]
```

### 3. Implementation Code Structure

```
your_capstone_project/
├── src/
│   ├── node1.py        # Custom ROS 2 node
│   ├── node2.py
│   └── utils.py        # Helper functions
├── launch/
│   └── main.launch.py  # Launch all nodes
├── config/
│   └── config.yaml     # Configuration parameters
├── tests/
│   ├── test_node1.py   # Unit tests
│   └── test_e2e.py     # End-to-end test
└── README.md
```

### 4. Test Plan

```markdown
# Test Plan

## Unit Tests
- [ ] Node 1 logic test
- [ ] Node 2 logic test
- [ ] Message parsing test

## Integration Tests
- [ ] Component A ↔ B communication
- [ ] Component B ↔ C communication
- [ ] Full pipeline (happy path)

## Failure Tests
- [ ] Timeout handling
- [ ] Invalid message format
- [ ] Missing sensor data

## Performance Tests
- [ ] Latency benchmark
- [ ] Throughput test
- [ ] Memory usage profile
```

## Capstone Grading Rubric

| Category | Excellent (90-100) | Good (80-89) | Satisfactory (70-79) | Needs Improvement (&lt;70) |
|----------|-------------------|--------------|----------------------|------------------------|
| **Design** | Well-thought-out architecture, clear tradeoffs documented, integrates ≥3 components effectively | Good design with most tradeoffs explained, ≥3 components integrated | Basic design, some tradeoffs explained, ≥3 components present | Incomplete design, unclear integration |
| **Implementation** | Code is clean, well-tested, handles errors gracefully, follows ROS 2 best practices | Code works, some tests present, minor error handling | Code works for happy path, minimal testing | Code incomplete or doesn't work |
| **Validation** | Comprehensive tests (unit + integration + performance), all tests pass, quantified improvements | Good test coverage, most tests pass, some metrics measured | Basic testing, some tests pass | Minimal or no testing |
| **Presentation** | Clear 10-min demo + slides, answers questions well, shows deep understanding | Good demo + slides, mostly clear explanations | Demo works, slides present but unclear | Demo broken or presentation incomplete |
| **Documentation** | Complete design doc, architecture diagrams, API docs, troubleshooting guide | Good documentation, mostly complete | Basic documentation, missing some details | Incomplete documentation |

## Presentation Guidelines

### Demo (10 minutes)
1. **Setup** (1 min): Launch Isaac Sim, explain scene
2. **Normal operation** (5 min): Show system working (happy path)
3. **Failure handling** (2 min): Trigger failure, show recovery
4. **Performance** (2 min): Show metrics, explain tradeoffs

### Slides (5 minutes)
1. **Problem** (1 slide): What you're solving
2. **Solution** (1 slide): Your approach
3. **Implementation** (1 slide): Key components
4. **Results** (1 slide): Did you achieve success criteria?
5. **Lessons learned** (1 slide): What went well, what's hard

### Q&A (5 minutes)
Be prepared to answer:
- "Why did you choose this architecture?"
- "What would you do differently?"
- "How does this scale to real robots?"
- "What are the failure modes?"

## Example Capstone Project: "Autonomous Table Clearing"

### Problem Statement
Robots should help humans with tedious cleanup tasks. A robot that can autonomously clear a cluttered table (sorting objects by type/location) would be valuable in homes, restaurants, or offices.

### Solution
Use voice commands ("Clear the table") + vision to detect objects + LLM planning to generate sequence + manipulation to execute pick-and-place + safety to prevent accidents.

### Architecture
```
Voice Command: "Clear the table"
         ↓
Vision Node: Detect red cup, blue plate, napkin, fork
         ↓
LLM Planner: "First pick red cup, place in sink area. Then pick blue plate..."
         ↓
Grasp Planner: Generate grasps for cup (top approach, 30N force)
         ↓
Motion Planner: Plan collision-free path to sink area
         ↓
Executor: Close gripper, lift cup, move to sink area, open gripper
         ↓
Safety Monitor: Ensure no collisions, gripper force <100N
         ↓
Result: Cup moved to sink area
```

### Implementation Summary
- **Voice Node** (100 LOC): Listen for "clear" command
- **Vision Node** (150 LOC): YOLO detection + object classification
- **Planning Node** (150 LOC): Custom state machine (pick item → place → return → repeat)
- **Grasp Node** (120 LOC): Grasp strategy for cups, plates, utensils
- **Safety Node** (80 LOC): Force and workspace monitoring
- **Total**: ~600 LOC

### Success Criteria
1. Detect ≥5 objects in scene ✓
2. Clear cluttered table in &lt;2 minutes ✓
3. Zero gripper force violations ✓
4. Recover from failed grasp (retry) ✓
5. Stop on E-stop signal &lt;100ms ✓

### Performance Results
| Metric | Target | Achieved |
|--------|--------|----------|
| Detection accuracy | ≥85% | 92% |
| Grasp success rate | ≥80% | 87% |
| Motion planning time | &lt;2s | 1.2s |
| Total task time | &lt;2 min | 105s |
| Force violations | 0 | 0 |

### Design Tradeoffs
- **Accuracy vs Speed**: Used yolov8n (fast, 85% accurate) instead of yolov8l (slower, 93% accurate) to keep latency low
- **Generality vs Specificity**: Designed grasps specifically for cups/plates (works well) rather than generic grasp planner (slower)
- **Replanning vs Committing**: If grasp fails once, retry with same grasp rather than full replan (faster)

## Getting Started

1. **Choose your idea** (see ideas above or propose your own)
2. **Write problem statement** (understand what you're building)
3. **Design architecture** (how will components interact?)
4. **Implement incrementally** (start with single component, add others)
5. **Test thoroughly** (unit tests → integration tests → performance tests)
6. **Document everything** (others should understand your design)
7. **Prepare presentation** (practice 10-min demo)
8. **Submit** (code + design doc + presentation slides)

## What Happens Next?

After you complete this capstone, you've learned:
- ✓ How to build autonomous systems (voice → perception → planning → action)
- ✓ How to integrate multiple ML/AI components (Whisper, YOLO, LLM)
- ✓ How to make systems robust (error handling, replanning, safety)
- ✓ How to validate and measure performance
- ✓ How to communicate technical ideas to others

**This is professional robotics engineering.**

### Paths Forward
1. **Research**: Extend your project into research paper
2. **Real robot**: Deploy to real hardware (with proper safety)
3. **Commercialization**: Turn your system into product
4. **Teaching**: Help teach this course to others
5. **Open source**: Share your code with community

## Key Takeaways

1. **System design** is about making tradeoffs
2. **Integration** is harder than individual components
3. **Testing** catches problems early
4. **Safety** is non-negotiable for real robots
5. **Documentation** makes your work valuable to others
6. **Presentation** skills matter for getting ideas approved

## Final Thoughts

Congratulations on completing Chapter 4! You've learned:
- **Lesson 1**: VLA pipeline conceptually
- **Lesson 2**: Voice recognition (Whisper)
- **Lesson 3**: Vision systems (YOLO + segmentation)
- **Lesson 4**: LLM planning
- **Lesson 5**: Manipulation & grasping
- **Lesson 6**: System orchestration
- **Lesson 7**: Safety protocols
- **Lesson 8**: System design & integration (capstone)

You now understand how to build complete autonomous systems. Whether you pursue robotics as a career or hobby, you have the foundational knowledge to tackle complex problems.

**Now go build something amazing!** 🚀

---

**Have questions?**
- Review earlier lessons for component details
- Check troubleshooting guide (Chapter README)
- Ask instructors in office hours
- Share designs on class forum

**Ready to deploy on real hardware?**
See Appendix: Real Hardware Deployment for safety checklist and hardware considerations.

See you in the next chapter! 👋
