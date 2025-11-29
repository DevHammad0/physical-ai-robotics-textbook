---
description: "Task list for RAG Chatbot Integration feature implementation"
---

# Tasks: RAG Chatbot Integration

**Input**: Design documents from `/specs/006-rag-chatbot/`  
**Prerequisites**: plan.md (tech stack, architecture), spec.md (user stories with priorities)

**Tests**: Not explicitly requested - focusing on implementation and manual validation per spec.md acceptance criteria

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` directory with FastAPI structure
- **Frontend**: `book-source/src/` directory with Docusaurus/React
- **Scripts**: `backend/scripts/` for indexing and utilities

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, API accounts, and basic backend structure

- [ ] T001 Create API accounts (OpenAI, Qdrant Cloud, Neon Postgres, Vercel)
- [ ] T002 Create backend directory structure (backend/, backend/routers/, backend/services/, backend/models/, backend/scripts/)
- [ ] T003 Initialize Python virtual environment in backend/
- [ ] T004 Create backend/requirements.txt with dependencies: fastapi, uvicorn[standard], openai, qdrant-client, asyncpg, pydantic-settings, pyyaml, tiktoken, python-multipart
- [ ] T005 [P] Create backend/.env.example with all required environment variables
- [ ] T006 [P] Create backend/.gitignore for Python project (venv/, __pycache__/, *.pyc, .env)
- [ ] T007 [P] Create backend/vercel.json for Vercel deployment configuration
- [ ] T008 Create backend/scripts/test_connections.py to validate API connectivity
- [ ] T009 Run connection tests to verify OpenAI, Qdrant, and Neon are accessible

**Checkpoint**: Backend project structure ready, all external services accessible

---

## Phase 2: Foundational (Backend Core Infrastructure)

**Purpose**: Core backend infrastructure that BLOCKS all user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T010 [P] Implement backend/config.py with pydantic-settings for environment management
- [ ] T011 Create backend/scripts/schema.sql with all database tables (conversations, messages, chat_analytics, users, agent_messages)
- [ ] T012 Implement backend/scripts/init_db.py to create database schema in Neon Postgres
- [ ] T013 Run init_db.py script to initialize database tables and indexes
- [ ] T014 [P] Implement backend/services/db_service.py with AsyncPG connection pooling (init_db_pool, close_db_pool, get_pool)
- [ ] T015 [P] Implement backend/models/schemas.py with Pydantic request/response models (ChatRequest, ChatResponse, Source, HealthResponse)
- [ ] T016 Implement backend/routers/health.py with GET /api/health endpoint
- [ ] T017 Implement backend/main.py with FastAPI app, CORS middleware, lifespan management, and router includes
- [ ] T018 Test local backend startup with uvicorn main:app --reload and verify /api/health responds

**Checkpoint**: Backend foundation ready - user story implementation can now begin

---

## Phase 3: User Story 4 - Content Indexing and Search (Priority: P1) 🎯 FOUNDATIONAL

**Goal**: Index all 35+ markdown lessons into Qdrant vector database for semantic search

**Independent Test**: Run indexing script, verify ~600 chunks created, search for "URDF" returns top 5 relevant chunks with metadata

**Why First**: This is foundational infrastructure - User Story 1 (Q&A) cannot work without indexed content

### Implementation for User Story 4

- [ ] T019 [P] [US4] Create backend/services/embedding_service.py with EmbeddingGenerator class (async OpenAI embeddings)
- [ ] T020 [P] [US4] Implement backend/services/vector_search.py with VectorSearchService class (Qdrant async client)
- [ ] T021 [US4] Implement MarkdownParser class in backend/scripts/index_content.py (parse YAML frontmatter, extract sections)
- [ ] T022 [US4] Implement ContentChunker class in backend/scripts/index_content.py (heading-based, 800 token max, 100 overlap)
- [ ] T023 [US4] Implement URL generation logic in backend/scripts/index_content.py (convert file paths to Docusaurus URLs)
- [ ] T024 [US4] Implement Qdrant collection creation in backend/scripts/index_content.py (1536 dimensions, cosine distance)
- [ ] T025 [US4] Implement batch embedding generation in backend/scripts/index_content.py (100 texts per batch)
- [ ] T026 [US4] Implement chunk upload to Qdrant in backend/scripts/index_content.py (with metadata: chapter, lesson, heading, url)
- [ ] T027 [US4] Add CLI arguments to backend/scripts/index_content.py (--docs-dir, --collection, --incremental)
- [ ] T028 [US4] Run indexing script on book-source/docs/ directory and verify ~600 chunks created
- [ ] T029 [US4] Create backend/scripts/test_search.py to validate vector search with sample queries
- [ ] T030 [US4] Test vector search performance (<200ms for top-5 results)

**Checkpoint**: Content indexed and searchable - Core Q&A (US1) can now be implemented

---

## Phase 4: User Story 1 - Ask Questions About Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Students can ask questions and receive accurate answers with source citations in <2s

**Independent Test**: Open any Docusaurus page, type "What is ROS 2 Humble?" in chat widget, verify accurate answer with chapter citations appears within 2 seconds

### Backend Implementation for User Story 1

- [ ] T031 [US1] Implement backend/services/llm_service.py with LLMService class (build_system_prompt, build_context, generate_response, generate_embedding methods)
- [ ] T032 [US1] Implement ConversationService class in backend/services/db_service.py (create_or_get_conversation, add_message, get_conversation_history)
- [ ] T033 [US1] Implement AnalyticsService class in backend/services/db_service.py (log_query method)
- [ ] T034 [US1] Implement extract_sources utility function in backend/services/llm_service.py (top-3 unique chapter/lesson pairs)
- [ ] T035 [US1] Implement POST /api/chat endpoint in backend/routers/chat.py (query embedding → vector search → LLM generation → response)
- [ ] T036 [US1] Add rate limiting middleware to backend/main.py (30 requests per minute per session)
- [ ] T037 [US1] Add error handling for OpenAI rate limits in backend/services/llm_service.py (exponential backoff, max 3 retries)
- [ ] T038 [US1] Add fallback to retrieval-only mode if LLM fails in backend/routers/chat.py
- [ ] T039 [US1] Test /api/chat endpoint with curl: verify response includes message, sources, conversation_id, response_time_ms

### Frontend Implementation for User Story 1

- [ ] T040 [P] [US1] Create book-source/src/components/ChatWidget/ChatWidget.tsx with state management (isOpen, messages, conversationId, sessionId, isLoading)
- [ ] T041 [P] [US1] Create book-source/src/components/ChatWidget/MessageList.tsx component
- [ ] T042 [P] [US1] Create book-source/src/components/ChatWidget/MessageInput.tsx component
- [ ] T043 [P] [US1] Create book-source/src/components/ChatWidget/SourceCitation.tsx component
- [ ] T044 [P] [US1] Create book-source/src/components/ChatWidget/ChatWidget.module.css with light/dark mode support
- [ ] T045 [US1] Implement book-source/src/utils/api.ts with sendChatMessage and checkHealth functions
- [ ] T046 [US1] Implement book-source/src/utils/storage.ts with saveConversation and loadConversation functions (7-day expiry)
- [ ] T047 [US1] Create book-source/src/theme/Root.tsx to inject ChatWidget globally in Docusaurus
- [ ] T048 [US1] Add REACT_APP_CHAT_API_URL to book-source/.env.example
- [ ] T049 [US1] Test ChatWidget locally: open chat, send message, verify response displays with sources

**Checkpoint**: Core Q&A functionality complete - Students can ask questions and get answers with citations

---

## Phase 5: User Story 2 - Get Context-Specific Help from Highlighted Text (Priority: P2)

**Goal**: Students can highlight text on any page and ask questions about it without retyping

**Independent Test**: Highlight "Behavior trees enable hierarchical decision-making" on any page, click "Ask AI about this", type "Explain this simply", verify response references the highlighted text

### Implementation for User Story 2

- [ ] T050 [P] [US2] Create book-source/src/utils/textSelection.ts with initTextSelectionHandler function
- [ ] T051 [US2] Add text selection event listeners (mouseup, touchend) in textSelection.ts
- [ ] T052 [US2] Implement "Ask AI about this" button creation and positioning logic in textSelection.ts
- [ ] T053 [US2] Integrate text selection handler with ChatWidget.tsx (pass selectedText to API)
- [ ] T054 [US2] Update ChatWidget.tsx to display selected text banner when context is provided
- [ ] T055 [US2] Update backend/routers/chat.py to include selected_text in LLM context
- [ ] T056 [US2] Test text selection flow: highlight text → click button → chat opens → send query → response references selection

**Checkpoint**: Text selection feature complete - Students can ask about highlighted passages

---

## Phase 6: User Story 3 - Have Multi-Turn Conversations (Priority: P3)

**Goal**: Students can have context-aware conversations without repeating previous questions

**Independent Test**: Ask "What is URDF?", wait for response, ask "Show me an example", verify chatbot understands context

### Implementation for User Story 3

- [ ] T057 [US3] Update ConversationService.get_conversation_history in backend/services/db_service.py to return last 10 messages
- [ ] T058 [US3] Update LLMService.build_conversation_context in backend/services/llm_service.py to format last 5 messages for LLM
- [ ] T059 [US3] Update POST /api/chat in backend/routers/chat.py to retrieve and include conversation history
- [ ] T060 [US3] Update ChatWidget.tsx to load conversation from localStorage on mount
- [ ] T061 [US3] Update ChatWidget.tsx to save conversation to localStorage after each message
- [ ] T062 [US3] Implement "Clear History" button in ChatWidget.tsx with confirmation dialog
- [ ] T063 [US3] Test multi-turn conversation: Q1 "What is URDF?" → Q2 "Show me an example" → verify context awareness
- [ ] T064 [US3] Test history persistence: send messages → refresh page → verify history restored
- [ ] T065 [US3] Test history across page navigation: send messages on page A → navigate to page B → verify history visible

**Checkpoint**: Multi-turn conversations complete - Students can have contextual back-and-forth discussions

---

## Phase 7: User Story 5 - Conversation Persistence and Analytics (Priority: P4)

**Goal**: System logs all queries for monitoring and improvement, anonymous users have 7-day conversation persistence

**Independent Test**: Send 10 queries through chatbot, query analytics database, verify all 10 logged with response times and session IDs

### Implementation for User Story 5

- [ ] T066 [P] [US5] Verify chat_analytics table exists in database (created in T011)
- [ ] T067 [US5] Update POST /api/chat in backend/routers/chat.py to call AnalyticsService.log_query
- [ ] T068 [US5] Implement automatic localStorage expiry (7 days) in book-source/src/utils/storage.ts
- [ ] T069 [P] [US5] Implement conversation export feature in ChatWidget.tsx (download as markdown)
- [ ] T070 [US5] Test analytics logging: send 10 queries → query chat_analytics table → verify all logged
- [ ] T071 [US5] Test 7-day expiry: set mock old date → load conversation → verify auto-deletion
- [ ] T072 [US5] Test conversation export: send messages → click export → verify markdown file downloads

**Checkpoint**: Analytics and persistence complete - System tracks usage for improvement

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final production readiness

- [ ] T073 [P] Add input validation and sanitization to prevent XSS in backend/routers/chat.py
- [ ] T074 [P] Add comprehensive error messages for all edge cases in backend/routers/chat.py (empty query, too long, no results)
- [ ] T075 [P] Implement loading spinner and typing indicator in ChatWidget.tsx
- [ ] T076 [P] Add mobile-responsive styles to ChatWidget.module.css (full-screen on <768px)
- [ ] T077 [P] Add keyboard navigation support (Tab, Enter, Escape) to ChatWidget.tsx
- [ ] T078 [P] Add ARIA labels and WCAG 2.1 AA compliance to ChatWidget.tsx
- [ ] T079 [P] Optimize ChatWidget lazy loading in Root.tsx to prevent blocking page render
- [ ] T080 Test mobile responsiveness on Chrome DevTools and actual device
- [ ] T081 Test accessibility with keyboard-only navigation
- [ ] T082 Test cross-browser compatibility (Chrome, Firefox, Safari, Edge)
- [ ] T083 Deploy backend to Vercel with vercel --prod
- [ ] T084 Update book-source/.env with production backend URL
- [ ] T085 Deploy frontend to GitHub Pages via git push (automatic via GitHub Actions)
- [ ] T086 End-to-end testing on production URLs
- [ ] T087 Performance validation: test p90 latency <2s with 20 sample queries
- [ ] T088 Prepare demo with 5 test questions (general Q&A, text selection, multi-turn, out-of-scope, edge cases)

**Checkpoint**: Production-ready chatbot deployed and validated

---

## Phase 9 (OPTIONAL): Agentic Enhancement

**Purpose**: Add OpenAI Agents SDK for autonomous tool calling and improved conversation quality

**Prerequisites**: Phases 1-8 complete (manual RAG fully working)

- [ ] T089 Install openai-agents package in backend/ (pip install openai-agents)
- [ ] T090 [P] Create backend/agent.py with Teacher Agent definition
- [ ] T091 [P] Implement search_textbook function tool in backend/agent.py using @function_tool decorator
- [ ] T092 [P] Create backend/session.py with PostgresSession custom class (compatible with Agents SDK)
- [ ] T093 Update backend/scripts/schema.sql to add agent_messages table for agent session storage
- [ ] T094 Run schema update script to add agent_messages table to Neon Postgres
- [ ] T095 Create backend/routers/agent_chat.py with POST /api/agent/chat endpoint
- [ ] T096 Implement Runner.run_streamed integration in agent_chat.py
- [ ] T097 Test agent tool calling: send query → verify agent calls search_textbook → verify response
- [ ] T098 Test agent vs manual comparison: same 10 queries → compare quality, latency, cost
- [ ] T099 [P] Add frontend toggle in ChatWidget.tsx to switch between /api/chat and /api/agent/chat
- [ ] T100 Add ENABLE_AGENT_MODE environment variable to backend/config.py

**Checkpoint**: Agentic architecture available as alternative to manual RAG for comparison

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 completion - BLOCKS all user stories
- **US4 Content Indexing (Phase 3)**: Depends on Phase 2 - BLOCKS US1
- **US1 Core Q&A (Phase 4)**: Depends on Phase 3 (needs indexed content)
- **US2 Text Selection (Phase 5)**: Depends on Phase 4 (extends US1 functionality)
- **US3 Multi-turn (Phase 6)**: Depends on Phase 4 (enhances US1 with history)
- **US5 Analytics (Phase 7)**: Depends on Phase 4 (logs US1 queries)
- **Polish (Phase 8)**: Depends on all desired user stories being complete
- **Agentic (Phase 9)**: Optional - depends on Phase 8 completion

### User Story Dependencies

```
Phase 1: Setup
    ↓
Phase 2: Foundational (Core Backend)
    ↓
Phase 3: US4 - Content Indexing (P1) ← MUST complete first
    ↓
Phase 4: US1 - Core Q&A (P1) ← MVP complete here
    ↓
Phase 5: US2 - Text Selection (P2) ← Extends US1
    ↓
Phase 6: US3 - Multi-turn (P3) ← Enhances US1
    ↓
Phase 7: US5 - Analytics (P4) ← Monitors US1
    ↓
Phase 8: Polish & Deployment
    ↓
Phase 9: Agentic Enhancement (OPTIONAL)
```

**Note**: US6 (Authentication - P5 Bonus) is excluded from scope as it's explicitly a bonus feature

### Within Each User Story

- Backend services before endpoints
- Frontend components can be built in parallel (marked [P])
- Integration tasks after all components exist
- Story complete before moving to next priority

### Parallel Opportunities

- **Phase 1 Setup**: T005, T006, T007 can run in parallel (different files)
- **Phase 2 Foundational**: T010, T014, T015 can run in parallel (different files)
- **Phase 3 US4**: T019, T020 can run in parallel (different services)
- **Phase 4 US1 Backend**: Backend and frontend tasks can run in parallel (T031-T039 backend, T040-T049 frontend)
- **Phase 4 US1 Frontend**: T040, T041, T042, T043, T044 can run in parallel (different component files)
- **Phase 5 US2**: T050, T051, T052 can run in parallel (different features in same file)
- **Phase 8 Polish**: T073-T079 can run in parallel (different files/concerns)
- **Phase 9 Agentic**: T090, T091, T092 can run in parallel (different files)

---

## Parallel Example: User Story 1 (Core Q&A)

```bash
# Launch backend services in parallel:
Task T031: "Implement LLMService in backend/services/llm_service.py"
Task T032: "Implement ConversationService in backend/services/db_service.py"
Task T033: "Implement AnalyticsService in backend/services/db_service.py"

# Launch frontend components in parallel:
Task T040: "Create ChatWidget.tsx"
Task T041: "Create MessageList.tsx"
Task T042: "Create MessageInput.tsx"
Task T043: "Create SourceCitation.tsx"
Task T044: "Create ChatWidget.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 + US4 Only)

**Minimum Viable Product Scope:**

1. Complete Phase 1: Setup (T001-T009) → ~30-45 min
2. Complete Phase 2: Foundational (T010-T018) → ~3-4 hours
3. Complete Phase 3: US4 Content Indexing (T019-T030) → ~2-3 hours
4. Complete Phase 4: US1 Core Q&A (T031-T049) → ~4-5 hours
5. **STOP and VALIDATE**: Test US1 independently (open chat, ask questions, verify citations)
6. Complete Phase 8: Polish basics (T073-T079, T083-T088) → ~2 hours
7. Deploy to production and demo

**MVP delivers**: Working chatbot that answers textbook questions with source citations

**Total MVP time**: ~12-15 hours (1.5-2 days)

### Incremental Delivery (Add Features Progressively)

1. **MVP Complete** (US4 + US1) → Foundation + Core Q&A
2. **Add US2** (Text Selection) → Phase 5 (T050-T056) → +1-2 hours
3. **Add US3** (Multi-turn) → Phase 6 (T057-T065) → +2-3 hours
4. **Add US5** (Analytics) → Phase 7 (T066-T072) → +1-2 hours
5. **OPTIONAL**: Add Phase 9 (Agentic) → +2-3 hours

Each increment adds value without breaking previous features.

### Parallel Team Strategy

With multiple developers:

1. **Together**: Complete Phase 1 (Setup) and Phase 2 (Foundational)
2. **Together**: Complete Phase 3 (US4 - Content Indexing)
3. **Split Work**:
   - Developer A: Phase 4 Backend (T031-T039)
   - Developer B: Phase 4 Frontend (T040-T049)
   - Developers sync at T049 for integration testing
4. **Continue Split**:
   - Developer A: Phase 5 (US2 - Text Selection)
   - Developer B: Phase 6 (US3 - Multi-turn)
5. **Final**: Phase 8 (Polish) together

---

## Critical Path Analysis

**Critical Path (Sequential Dependencies):**

```
T001-T009 (Setup) → 
T010-T018 (Foundational) → 
T019-T030 (US4 Indexing) → 
T031-T039 (US1 Backend) → 
T035 (Chat Endpoint Integration) → 
T040-T049 (US1 Frontend) → 
T049 (Integration Test) → 
T083-T088 (Deployment)
```

**Estimated Critical Path Time**: ~12-15 hours for MVP

**Parallelizable Work (Off Critical Path)**:
- Phase 5 (US2) - can start after US1 complete
- Phase 6 (US3) - can start after US1 complete
- Phase 7 (US5) - can start after US1 complete
- Phase 9 (Agentic) - completely optional, can start after MVP deployed

---

## Task Counts

- **Phase 1 (Setup)**: 9 tasks (T001-T009)
- **Phase 2 (Foundational)**: 9 tasks (T010-T018)
- **Phase 3 (US4 - Indexing)**: 12 tasks (T019-T030)
- **Phase 4 (US1 - Core Q&A)**: 19 tasks (T031-T049) ← MVP CORE
- **Phase 5 (US2 - Text Selection)**: 7 tasks (T050-T056)
- **Phase 6 (US3 - Multi-turn)**: 9 tasks (T057-T065)
- **Phase 7 (US5 - Analytics)**: 7 tasks (T066-T072)
- **Phase 8 (Polish)**: 16 tasks (T073-T088)
- **Phase 9 (Agentic - Optional)**: 12 tasks (T089-T100)

**Total**: 100 tasks  
**MVP Only**: 62 tasks (Phases 1-4 + Phase 8 essentials)  
**Full Feature**: 88 tasks (excludes optional Phase 9)

---

## Notes

- **[P] tasks**: Different files, no dependencies - can run in parallel
- **[Story] labels**: Map tasks to user stories for traceability (US1, US2, US3, US4, US5)
- **Independent Testing**: Each user story has clear validation criteria
- **MVP Scope**: US4 (Indexing) + US1 (Core Q&A) = minimum viable product
- **Incremental Value**: Each phase after MVP adds independent value
- **Checkpoint Validation**: Test story independence at each checkpoint
- **Package Names**: Use manually updated names in requirements.txt (no changes needed)
- **Model Configuration**: All model references use settings.openai_model from .env (change in one place)

---

## Validation Checklist

Before considering implementation complete:

- [ ] All MVP tasks (T001-T049 + T073-T088) completed
- [ ] ~600 chunks indexed in Qdrant
- [ ] Chat widget appears on all Docusaurus pages
- [ ] Questions receive answers with source citations in <2s (p90)
- [ ] Text selection "Ask AI" button appears and works
- [ ] Conversation history persists across page navigation and refresh
- [ ] Mobile responsive layout works on small screens
- [ ] Keyboard navigation functional (Tab, Enter, Escape)
- [ ] Backend deployed to Vercel (accessible via HTTPS)
- [ ] Frontend deployed to GitHub Pages
- [ ] Health check endpoint returns "healthy" for all services
- [ ] Demo successfully completes in <2 minutes
- [ ] Zero console errors in browser DevTools during normal operation

---

**Status**: ✅ Ready for Implementation  
**Next**: Begin with Phase 1 (T001) and follow dependency order  
**MVP Milestone**: Complete through T049, then deploy

