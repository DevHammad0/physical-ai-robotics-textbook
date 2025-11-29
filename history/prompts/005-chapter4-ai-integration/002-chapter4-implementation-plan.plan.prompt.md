---
id: 002
title: Chapter 4 Implementation Plan
stage: plan
date: 2025-11-29
surface: agent
model: claude-haiku-4-5-20251001
feature: 005-chapter4-ai-integration
branch: 005-chapter4-ai-integration
user: hammad
command: /sp.plan
labels: ["chapter4", "vla-pipeline", "implementation-plan", "architecture", "phase0-phase2"]
links:
  spec: specs/005-chapter4-ai-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-chapter4-ai-integration/plan.md
tests:
  - Constitution check: PASS (all 7 principles satisfied)
  - Phase 0 research: All unknowns resolved (6 technology decisions validated)
  - Phase 1 design: Data models, API contracts, quickstart outlined
  - Phase 2 roadmap: 8-lesson implementation sequence + 5 ADRs identified
---

## Prompt

Execute planning workflow for Chapter 4 (Vision-Language-Action autonomous systems curriculum).

Specification: 8 lessons, 100+ functional requirements, 6 user stories (4 P1, 2 P2), safety-mandatory architecture.

Deliver implementation plan with Phase 0 (research), Phase 1 (design), Phase 2 (roadmap) sections.

## Response snapshot

**Implementation Plan Created** (`plan.md` — 369 lines)

### Sections Completed:

**Summary**: Chapter 4 integrates voice-controlled perception, LLM-driven planning, manipulation, and full-stack integration into 8-lesson curriculum. Technical approach: progressive complexity (foundation → orchestration), simulation-first (Isaac Sim), code-first pedagogy, real-world considerations in every lesson.

**Technical Context**:
- Language: Python 3.10+, ROS 2 Humble+
- Dependencies: Whisper (ASR), YOLO-v8 (detection), GPT-4 API (planning), ROS 2, Isaac Sim 4.0+
- Performance goals: <500ms voice latency, <10s end-to-end pipeline, <100ms kill switch
- Scale: 8 lessons, ~2000 LOC examples, 100+ requirements

**Constitution Check**: ✅ All 7 principles satisfied (MVP-first, content accuracy, tech stack adherence, educational structure, safety framework, spec-driven, implementation path clear)

**Phase 0: Research & Clarification** (6 research tasks resolved):
1. Whisper ASR: Local inference (primary) + API option
2. Object Detection: YOLO-v8 (default) vs Detectron (advanced)
3. LLM Provider: GPT-4 API (primary) vs Ollama (alternative)
4. Motion Planning: Custom trajectory generation (lessons 1-6) + MoveIt 2 (P2)
5. Simulation Platform: Isaac Sim (primary) + Gazebo (alternative)
6. Safety Validation: Dedicated safety node (non-negotiable)

**Phase 1: Design & API Contracts**:
- **Data Model**: 7 entities (VoiceCommand, DetectionResult, TaskPlan, GraspCandidate, ExecutionStatus, SafetyEvent, RobotState) with relationships + state transitions
- **ROS 2 Interfaces**: 5 topics, 3 services, 2 actions (all specified with message types)
- **Quickstart**: <2 hours from zero to running first example
- **Agent Context**: 4 new technologies registered (Whisper, YOLO-v8, GPT-4, ROS 2 custom messages)

**Phase 2: Implementation Roadmap**:
- **8 Lessons**: Lesson 1 (overview, 0 code) through Lesson 8 (capstone, student projects) with hours, deliverables, ADRs
- **5 ADRs Identified**:
  1. VLA Pipeline Architecture (decoupled ROS 2 nodes)
  2. Whisper Integration (local vs API)
  3. LLM Provider (GPT-4 vs Ollama)
  4. Safety Validation (dedicated node)
  5. Motion Planning (custom vs MoveIt 2)

**Next Steps**: Create research.md, data-model.md, contracts/, quickstart.md (Phase 1 artifacts), then `/sp.tasks` for testable implementation tasks.

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation. All technology decisions justified. Architecture decisions documented as ADRs. Constitution remains satisfied (post-design check).
- 🧪 Tests: Constitution check PASS. Phase 0 research complete (6/6 unknowns resolved). Phase 1 design outlined (data model, API contracts, quickstart). Phase 2 roadmap complete (lesson sequence + ADRs).
- 📁 Files: specs/005-chapter4-ai-integration/plan.md (369 lines, comprehensive).
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks; ADR documentation for significant decisions.
- 🧠 Reflection: Plan successfully transforms 8-lesson curriculum specification into actionable implementation roadmap. Design validates all constitution principles. Phase 0 research resolves technology choices with clear rationale. ADRs provide decision transparency. Ready for task generation.

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan completed comprehensively. All decisions justified with rationale and alternatives.
- Graders run and results (PASS/FAIL): Constitution check PASS. Phase 0 research PASS. Phase 1 design outlined PASS. Phase 2 roadmap PASS.
- Prompt variant (if applicable): Original plan command. No variants needed.
- Next experiment (smallest change to try): Proceed to task generation via `/sp.tasks` to break implementation into testable, prioritized, dependency-ordered tasks.
