# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Hackathon-Speed Development (MVP-First)
Non-negotiable: Base requirements (100 points) take absolute priority. Only after base requirements are fully functional should time be invested in bonus features. Prioritize working, deployable MVP over perfect polish. Time-boxed iterations (2-day deadline) drive all architectural and implementation decisions. Quick deployment wins (GitHub Pages live in hours, not days) validate progress.

### II. Content Accuracy & Verification
All technical content MUST be factually accurate. Code examples MUST be tested and functional before publication. All claims about APIs, features, or specifications MUST be verified against official documentation (ROS 2 docs, NVIDIA Isaac docs, OpenAI API docs). No hallucinated APIs or outdated information. If uncertain about accuracy, explicitly flag as "verify before publish" rather than guessing.

### III. Technology Stack Adherence (Non-Negotiable)
Docusaurus (TypeScript, pnpm) as book platform. RAG Chatbot using: OpenAI Agents/ChatKit SDK, FastAPI backend, Neon Serverless Postgres, Qdrant Cloud Free Tier. Deployment: GitHub Pages or Vercel. Authentication (bonus): better-auth library. No technology substitutions without explicit approval.

### IV. Educational Structure & Clarity
Each module (ROS 2, Gazebo, NVIDIA Isaac, VLA) must have clear learning outcomes. Content organized from beginner-friendly fundamentals to advanced concepts. Code examples must be production-relevant, not toy projects. Every chapter explains "why" concepts matter, not just "how" they work. Students should grasp Physical AI concepts regardless of robotics background.

### V. RAG Chatbot Excellence
Chatbot MUST answer questions from full book content. Chatbot MUST support text-selection-based queries (user highlights text → AI answers about that specific section). Response time target: <2 seconds. Handle out-of-scope questions gracefully (redirect to book sections). Proper error handling for API failures.

### VI. Spec-Driven Development Workflow
Follow Spec-Kit Plus methodology: constitution → spec → plan → tasks → implementation. Document significant architectural decisions with ADRs (Architecture Decision Records). Create PHRs (Prompt History Records) for all major work sessions. Use Claude Code agents and subagents to accelerate content creation and validation.

### VII. Bonus Feature Strategy (Systematic, Not Aspirational)
Bonus features implemented only AFTER base requirements fully functional. Implement bonuses incrementally in priority order. Each bonus feature MUST be fully functional (no partial credit). Test thoroughly before moving to next bonus. Suggested priority: Claude Subagents (+50) → Auth (+50) → Personalization (+50) → Urdu Translation (+50).

### VIII. Deployment & Performance Excellence
Book MUST deploy successfully to GitHub Pages or Vercel. Target performance: Page loads under 3 seconds (Lighthouse Performance >90). Mobile-responsive design (tested on mobile). No broken links, missing images, or 404 errors. CI/CD automation validates deployment on every commit.

## Non-Goals (Explicitly Excluded)

- Complex custom authentication flows (use better-auth defaults)
- Multi-language support beyond Urdu (scope-limited)
- Advanced robotics simulations (book is educational content, not simulator)
- Offline support (chatbot requires internet connectivity)
- User-generated content, comments, or community forums
- Real-time collaborative editing features
- Mobile app (web-responsive book suffices)

## Development Phases (2-Day Timeline)

### Phase 1: Foundation (Hours 0-6)
- Create constitution (this document)
- Initialize Docusaurus project with TypeScript
- Set up basic book structure (4 modules: ROS 2, Gazebo, Isaac, VLA)
- Deploy working skeleton to GitHub Pages
- Success metric: Live, deployable site with empty modules

### Phase 2: Content Creation (Hours 6-18)
- Write substantial content for all 4 modules
- Use Claude Code subagents for content generation
- Create code examples with testing validation
- Add diagrams, explanations, learning outcomes
- Success metric: All modules populated, book coherent, no placeholder text

### Phase 3: RAG Chatbot Integration (Hours 18-30)
- Set up FastAPI backend for chatbot
- Configure Neon Postgres database
- Integrate Qdrant Cloud for vector embeddings
- Implement OpenAI Agents chatbot
- Embed chatbot UI in Docusaurus book
- Test text-selection query functionality
- Success metric: Chatbot responds accurately to book-based queries

### Phase 4: Bonus Features (Hours 30-42)
- **Priority 1 (+50)**: Claude Subagents/Skills for content generation (demonstrates technical depth)
- **Priority 2 (+50)**: better-auth signup/signin + user background questionnaire
- **Priority 3 (+50)**: Content personalization based on user background (requires Priority 2)
- **Priority 4 (+50)**: Urdu translation toggle for chapters (good UX, technically straightforward)

### Phase 5: Demo & Submission (Hours 42-48)
- Create demo video (<90 seconds) showcasing key features
- Final testing and bug fixes
- Performance optimization (Lighthouse scores)
- Final deployment
- Submit to form: GitHub repo + published link + demo video + WhatsApp number

## Quality Standards & Acceptance Criteria

### Code Quality
- TypeScript strict mode enabled
- ESLint + Prettier configured and passing
- No hardcoded secrets (all API keys in `.env`)
- Error boundaries for React components
- Type safety: No `any` types without justification

### Content Quality
- Each module has documented learning outcomes
- Code examples are copy-pasteable and tested
- Technical claims include sources/citations
- Diagrams are clear and optimized for web
- Grammar checked (no obvious typos)

### Performance Metrics
- Lighthouse Performance score: ≥90
- Book bundle size: <500KB (excluding optimized images)
- Chatbot response time: <2 seconds
- Mobile responsiveness: Tested on phone-sized viewport

### Security
- API keys stored in environment variables (`.env`)
- No secrets committed to git
- Input sanitization for chatbot queries
- Rate limiting on chatbot endpoints
- CORS properly configured

## Governance & Amendments

### Constitutional Authority
This constitution is the supreme governing document for hackathon project execution. Time constraints justify pragmatic (not perfect) decisions. Quality bar: "Impressive to judges" not "production-ready for 5 years."

### Amendment Process
- Minor amendments (wording, clarifications): Document in commit message
- Major changes (principle removals or redefinitions): Create ADR + update constitution
- Version bumping: MAJOR (breaking changes) | MINOR (new sections) | PATCH (wording)

### Compliance & Review
- All feature implementations validated against constitution principles
- Code reviews check for tech stack adherence
- Content reviews verify accuracy and clarity
- Final checklist before submission: All non-goals respected, all phases completed

## Success Metrics (Definition of Done)

### Base Requirements (100 points)
- [ ] Docusaurus TypeScript project deployed to GitHub Pages
- [ ] Content covers all 4 modules with learning outcomes
- [ ] RAG chatbot functional (answers full-book queries)
- [ ] Chatbot supports text-selection-based questions
- [ ] Public GitHub repository with clean git history
- [ ] Demo video <90 seconds showcasing features
- [ ] Form submitted with all required links

### Bonus Requirements (up to 200 extra points)
- [ ] Claude Subagents/Skills used in content creation (+50)
- [ ] better-auth signup/signin implemented (+50)
- [ ] User background questionnaire captured (+50)
- [ ] Content personalization by chapter (+50)
- [ ] Urdu translation toggle functional (+50)

### Non-Negotiable Quality Gates
- [ ] Zero broken links in published book
- [ ] Zero missing images or 404 errors
- [ ] Chatbot responds to valid queries within 2 seconds
- [ ] Book loads in browser within 3 seconds
- [ ] Mobile-responsive (tested on phone)
- [ ] All code examples tested and functional
- [ ] No hardcoded secrets in repository

---

**Version**: 1.0.0 | **Ratified**: 2025-11-28 | **Last Amended**: 2025-11-28

**Scope**: Physical AI & Humanoid Robotics Textbook hackathon project (deadline: Sunday, Nov 30, 2025, 6:00 PM)

**Next Step**: Use `/sp.specify` to create detailed feature specification based on these constitutional principles.
