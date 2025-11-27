---
description: "Task list for Docusaurus 3.x site initialization with TypeScript and pnpm"
---

# Tasks: Docusaurus Site Initialization & Configuration

**Input**: Design documents from `/specs/1-docusaurus-setup/`

**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, contracts/

**Tests**: Tests are OPTIONAL and not included in this feature (QA/Lighthouse validation in Phase 5 of hackathon)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story

**Phase Window**: 4-5 hours (Phase 1 of hackathon timeline, hours 0-6 of 48 total)

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5, US6)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for Docusaurus 3.x

**Duration**: 0-1 hour

- [ ] T001 Create project directory structure per implementation plan (docs/, src/, .github/workflows/)
- [ ] T002 Initialize Node.js project with package.json (node 18+, pnpm 8+)
- [ ] T003 [P] Create .gitignore file (node_modules/, build/, .env.local, dist/)
- [ ] T004 [P] Create .env.example template with required configuration variables
- [ ] T005 Initialize git repository and create initial commit

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Docusaurus setup that MUST be complete before ANY user story can be implemented

**Duration**: 1-2 hours

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Install Docusaurus 3.x and dependencies via pnpm install
- [ ] T007 [P] Create docusaurus.config.js with base configuration (site metadata, URL, baseUrl, favicon)
- [ ] T008 [P] Create sidebars.js with placeholder 4-module structure (ROS 2, Gazebo, NVIDIA Isaac, VLA)
- [ ] T009 Create TypeScript configuration (tsconfig.json with strict mode enabled)
- [ ] T010 [P] Configure ESLint and Prettier for code quality
- [ ] T011 Initialize docs/ directory structure with docs/intro.md
- [ ] T012 Create src/css/custom.css for theme customization
- [ ] T013 [P] Create .github/workflows/ directory for GitHub Actions
- [ ] T014 Verify local dev server works: pnpm start → http://localhost:3000
- [ ] T015 [P] Test build process: pnpm build → build/ directory generated without errors

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Developer Initializes Docusaurus Project (Priority: P1) 🎯 MVP

**Goal**: Deliver working Docusaurus dev environment with hot-reload and static build capability

**Independent Test**: Developer can `pnpm start` locally and see site on http://localhost:3000; `pnpm build` completes without errors

### Implementation for User Story 1

- [ ] T016 [US1] Configure docusaurus.config.js project metadata (title: "Physical AI & Humanoid Robotics Textbook", description, organization info)
- [ ] T017 [P] [US1] Create docs/modules/ros2/intro.md with module introduction
- [ ] T018 [P] [US1] Create docs/modules/gazebo/intro.md with module introduction
- [ ] T019 [P] [US1] Create docs/modules/isaac/intro.md with module introduction
- [ ] T020 [P] [US1] Create docs/modules/vla/intro.md with module introduction
- [ ] T021 [US1] Update sidebars.js to reference module intro files (4 categories with placeholder lesson items)
- [ ] T022 [US1] Verify all 4 modules display in sidebar navigation locally
- [ ] T023 [US1] Test hot-reload: modify docs/intro.md → verify page updates without restart
- [ ] T024 [US1] Verify `pnpm build` completes successfully with no errors

**Checkpoint**: User Story 1 should be fully functional - developer can initialize and run Docusaurus locally

---

## Phase 4: User Story 2 - Configure Custom Theme & Styling (Priority: P2)

**Goal**: Apply custom dark/light theme, colors, and fonts for Physical AI branding

**Independent Test**: Load site locally and verify dark/light theme toggle works; colors match configuration; fonts render correctly across pages

### Implementation for User Story 2

- [ ] T025 [P] [US2] Configure theme colors in docusaurus.config.js (primary, secondary, background, text colors)
- [ ] T026 [P] [US2] Create custom CSS variables in src/css/custom.css for Physical AI branding
- [ ] T027 [P] [US2] Configure dark mode toggle in themeConfig (colorMode: { defaultMode: 'light', disableSwitch: false, respectPrefersColorScheme: true })
- [ ] T028 [US2] Test dark/light mode toggle: click toggle → verify colors switch and preference persists
- [ ] T029 [US2] Configure custom fonts in src/css/custom.css (import from Google Fonts or system fonts)
- [ ] T030 [US2] Test font rendering: load page in Chrome, Firefox, Safari, Edge → verify fonts display correctly
- [ ] T031 [US2] Configure navbar branding in docusaurus.config.js (logo, title, links to modules)
- [ ] T032 [US2] Test mobile responsiveness: view site on mobile viewport (320px) → verify colors and fonts remain readable

**Checkpoint**: User Story 2 complete - theme is customized and dark mode works

---

## Phase 5: User Story 3 - Add Essential Docusaurus Plugins (Priority: P2)

**Goal**: Enable search, sitemap, and RSS plugins for discoverability and SEO

**Independent Test**: Verify search plugin works locally; sitemap.xml generates on build; RSS feed is valid

### Implementation for User Story 3

- [ ] T033 [P] [US3] Install @docusaurus/plugin-search-local via pnpm
- [ ] T034 [P] [US3] Install @docusaurus/plugin-sitemap via pnpm
- [ ] T035 [P] [US3] Install @docusaurus/plugin-ideal-image via pnpm
- [ ] T036 [US3] Configure search-local plugin in docusaurus.config.js (hashed: true, indexDocs: true, language: 'en')
- [ ] T037 [US3] Configure sitemap plugin in docusaurus.config.js (changefreq: 'weekly', priority: 0.5)
- [ ] T038 [US3] Configure ideal-image plugin in docusaurus.config.js (empty config for defaults)
- [ ] T039 [US3] Test search locally: pnpm start → type query in search box → verify results appear
- [ ] T040 [US3] Test sitemap generation: pnpm build → check `build/sitemap.xml` exists and contains all URLs
- [ ] T041 [US3] Verify build bundle includes search index (check `build/` directory, no errors)
- [ ] T042 [US3] Test performance: run Lighthouse locally (DevTools) → verify Performance score ≥85 on local build

**Checkpoint**: User Story 3 complete - plugins configured and working

---

## Phase 6: User Story 4 - Create Book Navigation Structure (Priority: P1)

**Goal**: Create 4-module navigation with lesson organization and proper URL structure

**Independent Test**: Load site locally; verify all 4 modules visible in sidebar; clicking modules loads correct pages; breadcrumb navigation works

### Implementation for User Story 4

- [ ] T043 [P] [US4] Create docs/modules/ros2/lessons/ directory structure (9-10 lesson placeholders)
- [ ] T044 [P] [US4] Create docs/modules/gazebo/lessons/ directory structure (9-10 lesson placeholders)
- [ ] T045 [P] [US4] Create docs/modules/isaac/lessons/ directory structure (9-10 lesson placeholders)
- [ ] T046 [P] [US4] Create docs/modules/vla/lessons/ directory structure (9-10 lesson placeholders)
- [ ] T047 [US4] Create placeholder lesson files: docs/modules/ros2/lessons/lesson-01.md, lesson-02.md, etc. (empty with front matter)
- [ ] T048 [US4] Create placeholder lesson files: docs/modules/gazebo/lessons/lesson-01.md through lesson-10.md
- [ ] T049 [US4] Create placeholder lesson files: docs/modules/isaac/lessons/lesson-01.md through lesson-10.md
- [ ] T050 [US4] Create placeholder lesson files: docs/modules/vla/lessons/lesson-01.md through lesson-10.md
- [ ] T051 [US4] Update sidebars.js to list all lesson files under each module category
- [ ] T052 [US4] Test navigation: load site → click each module → verify module page loads
- [ ] T053 [US4] Test lesson navigation: click lesson from sidebar → verify correct lesson page loads
- [ ] T054 [US4] Verify breadcrumb navigation displays correct hierarchy (Home > Module > Lesson)
- [ ] T055 [US4] Verify URL structure is clean (no hashes, proper path structure: /modules/ros2/lessons/lesson-01)
- [ ] T056 [US4] Test "Previous" and "Next" navigation buttons: verify they navigate to adjacent lessons

**Checkpoint**: User Story 4 complete - book navigation is fully functional

---

## Phase 7: User Story 5 - Configure GitHub Pages Deployment (Priority: P1)

**Goal**: Automate deployment to GitHub Pages via GitHub Actions on every main branch push

**Independent Test**: Commit to main → GitHub Actions runs → site deploys to GitHub Pages → live URL accessible

### Implementation for User Story 5

- [ ] T057 [US5] Use docusaurus-deployer skill: Run `skill: "docusaurus-deployer"` to configure GitHub Pages deployment
- [ ] T058 [US5] Configure GitHub repository settings: Settings → Pages → Source = gh-pages branch
- [ ] T059 [US5] Configure docusaurus.config.js for GitHub Pages: set `url` and `baseUrl` for your repository
- [ ] T060 [US5] Commit all changes and push to main: `git add . && git commit -m "Configure Docusaurus with GitHub Pages deployment" && git push origin main`
- [ ] T061 [US5] Monitor GitHub Actions: check Actions tab → verify workflow runs successfully (green checkmark)
- [ ] T062 [US5] Verify GitHub Pages deployment: navigate to GitHub Pages URL (https://username.github.io/repo-name) → confirm site is live
- [ ] T063 [US5] Test live site functionality: verify site loads, all 4 modules visible in sidebar, navigation works
- [ ] T064 [US5] Test subsequent deployments: make small content change, push to main, verify live site updates within 1-2 minutes

**Checkpoint**: User Story 5 complete - deployment automation working

---

## Phase 8: User Story 6 - Set Up Environment Variables & Secrets (Priority: P2)

**Goal**: Manage sensitive configuration securely without exposing credentials

**Independent Test**: .env.local exists locally but git-ignored; GitHub Secrets configured; build works with secrets injected

### Implementation for User Story 6

- [ ] T069 [P] [US6] Create .env.local file with local development configuration (api endpoints, ports, debug flags)
- [ ] T070 [P] [US6] Add .env.local to .gitignore (verify `git status` doesn't show .env.local)
- [ ] T071 [US6] Verify .env.example has placeholder values without secrets (document required env vars)
- [ ] T072 [US6] Configure GitHub Secrets (Settings → Secrets and variables → Actions):
  - [ ] T073 [US6] Add GITHUB_TOKEN (automatically provided by GitHub)
  - [ ] T074 [US6] (Optional) Add any API keys needed for Phase 3+ (chatbot integration)
- [ ] T075 [US6] Update GitHub Actions workflow to use secrets if needed: `${{ secrets.GITHUB_TOKEN }}`
- [ ] T076 [US6] Test GitHub Actions with secrets: run workflow, verify no secrets appear in logs
- [ ] T077 [US6] Review GitHub Actions build logs: verify no sensitive values are printed or exposed

**Checkpoint**: User Story 6 complete - environment configuration secure

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Validation, optimization, and final readiness checks

**Duration**: 0.5-1 hour

**Key Skill**: Use `docusaurus-deployer` skill for GitHub Pages deployment automation (configured in Phase 7, US5)

- [ ] T078 [P] Run Lighthouse performance test: `pnpm build && pnpm serve` → open DevTools → Lighthouse → verify Performance ≥90
- [ ] T079 [P] Test mobile responsiveness: use browser DevTools mobile emulator → verify site works on iPhone SE (375px), iPad (768px), Desktop (1440px)
- [ ] T080 [P] Cross-browser testing: verify site loads and functions in Chrome, Firefox, Safari, Edge (manual or automated)
- [ ] T081 [P] Check for broken links: run `pnpm build` → check build logs for 404 errors
- [ ] T082 Verify sitemap.xml is valid: check `build/sitemap.xml` with XMLLint or online validator
- [ ] T083 Verify search index is included: check `build/` contains search index files (searchScraper.json or similar)
- [ ] T084 Create README.md with quickstart instructions (pnpm install, pnpm start, pnpm build, pnpm serve)
- [ ] T085 [P] Create CONTRIBUTING.md with guidelines for Phase 2 content creators
- [ ] T086 Document module structure in docs/modules/README.md (guide for adding lessons)
- [ ] T087 Verify all configuration values are correct: docusaurus.config.js, package.json, .env.example
- [ ] T088 Final deployment test: push all commits to main → verify GitHub Actions runs → verify live site reflects all changes
- [ ] T089 Validate against spec.md: cross-check all user stories (US1-US6) are complete and testable
- [ ] T090 Create git tag for Phase 1 completion: `git tag phase-1-docusaurus-init && git push origin phase-1-docusaurus-init`

**Checkpoint**: Phase 1 complete - Docusaurus setup ready for Phase 2 content creation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - START HERE ✅
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories ⚠️ CRITICAL
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - US1 (Docusaurus init) and US5 (GitHub Pages) should complete before publishing to live site
  - US2 (Theme) and US3 (Plugins) can run in parallel with US4 (Navigation) and US6 (Env vars)
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

**Independent Stories** (can run in any order after Foundational):
- **US1 (Docusaurus init)**: ✅ No dependencies on other stories - MVP foundation
- **US2 (Theme config)**: ✅ Depends on US1 only (needs Docusaurus to exist)
- **US3 (Plugins)**: ✅ Depends on US1 only (needs Docusaurus to exist)
- **US4 (Navigation)**: ✅ Depends on US1 only (needs Docusaurus to exist)
- **US5 (Deployment)**: ✅ Depends on US1 (needs code to deploy); US4 helpful (navigation finalized)
- **US6 (Env vars)**: ✅ Depends on US5 (needs GitHub Actions setup)

**Recommended Sequence** (optimal for hackathon timeline):
1. Complete Phase 1 (Setup) - 0-1 hour
2. Complete Phase 2 (Foundational) - 1-2 hours
3. Complete in parallel or sequence:
   - US1 (Docusaurus init): 0.5 hours - MUST DO FIRST
   - US5 (Deployment): 0.5 hours - MUST DO EARLY (deploy early, deploy often)
   - US4 (Navigation): 0.5 hours - content structure needed before adding lessons
   - US2 (Theme) + US3 (Plugins) + US6 (Env vars): 1 hour parallel
4. Complete Phase 9 (Polish): 0.5 hours

**Total**: 4-5 hours ✅ Fits Phase 1 window

### Within Each User Story

- Implementation tasks ordered from models → configuration → tests → validation
- Core functionality before optional enhancements
- Story complete before moving to next priority

### Parallel Opportunities

All Setup tasks marked [P] can run in parallel (different files, no dependencies)
All Foundational tasks marked [P] can run in parallel (within Phase 2)
- T007, T008, T010, T013, T015 can run simultaneously
- T023, T040, T078, T079, T080 (verification tasks) can run simultaneously

**Once Foundational (Phase 2) completes**, all user stories can start in parallel:
- Developer A: US1 + US5 (init + deployment)
- Developer B: US4 (navigation structure)
- Developer C: US2 + US3 (theme + plugins)

---

## Parallel Example: Phase 2 Foundational

```bash
# These can run in parallel (different files, no shared dependencies):
Task T007: Create docusaurus.config.js
Task T008: Create sidebars.js
Task T010: Configure ESLint/Prettier
Task T013: Create .github/workflows/ directory
Task T015: Test pnpm build

# All initiated simultaneously, monitored for completion
# Proceed to Phase 3+ only when ALL Phase 2 tasks complete
```

---

## Implementation Strategy

### MVP First (User Story 1 + 5)

1. ✅ Complete Phase 1: Setup (0-1 hour)
2. ✅ Complete Phase 2: Foundational (1-2 hours)
3. ✅ Complete US1: Docusaurus init (0.5 hour)
4. ✅ Complete US5: GitHub Pages deployment (0.5 hour)
5. **STOP and VALIDATE**: Test Docusaurus locally and on live GitHub Pages
6. **Deploy and demo** (you have working live site)

**Minimum viable deliverable**: Live Docusaurus book on GitHub Pages with 4 modules, ready for Phase 2 content creation

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. US1 + US5 → Live site MVP (foundation + deployment)
3. US4 → Book navigation structure ready for content creators
4. US2 + US3 → Professional theme and discoverability
5. US6 → Secure configuration management

**Each story adds value without breaking previous work**

### Parallel Team Strategy (if multiple developers)

With 2-3 developers:

1. **Team together**: Complete Setup (Phase 1) + Foundational (Phase 2)
2. **Once foundational done, split**:
   - **Dev A**: US1 (Docusaurus init) + US5 (Deployment) → Get live site
   - **Dev B**: US4 (Navigation) → Prepare structure for Phase 2 content creators
   - **Dev C**: US2 (Theme) + US3 (Plugins) → Make it look good + add features
3. **Parallel Polish**: Dev A/B/C work on Phase 9 validation together
4. **Merge and test**: All stories integrated, lighthouse validation, final deploy

---

## Execution Timeline (4-5 hour Phase 1)

| Phase | Duration | Tasks | Cumulative |
|-------|----------|-------|-----------|
| **Setup** | 0-1h | T001-T005 | 1h |
| **Foundational** | 1-2h | T006-T015 | 3h |
| **US1 + US5** | 1h | T016-T068 | 4h |
| **US2, US3, US4, US6** | 0.5-1h | T025-T077 | 4.5-5h |
| **Polish** | 0.5h | T078-T090 | 5h |

**Goal**: Complete all tasks within 5 hours, ready to hand off to Phase 2 content creators

---

## Acceptance Criteria (From Specification)

All tasks must satisfy these success criteria:

- ✅ **SC-001**: Developer initializes in <10 min (T001-T015, T021)
- ✅ **SC-004**: Lighthouse ≥90 (T078, T042)
- ✅ **SC-005**: <3s page load on 4G (validated in T078)
- ✅ **SC-006**: Mobile responsive (T032, T079)
- ✅ **SC-010**: GitHub Actions 100% success (T064, T088)
- ✅ **SC-011**: No hardcoded secrets (T070, T077)
- ✅ **SC-013**: Cross-browser compatible (T080)

---

## Notes & Conventions

- **[P] tasks** = different files, no dependencies, can run simultaneously
- **[Story] label** = maps task to specific user story for traceability
- **Each user story** should be independently completable and testable
- **Commit frequently**: After each task or logical group (T015, T024, T042, T056, T068, T089)
- **Stop at checkpoints** to validate story independently before proceeding
- **Avoid**: Vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Phase 1 Success Indicators

✅ When Phase 1 is **COMPLETE**, you should have:

- [ ] ✅ Docusaurus 3.x project running locally with hot-reload
- [ ] ✅ Custom theme with dark/light mode working
- [ ] ✅ Search, sitemap, and image optimization plugins configured
- [ ] ✅ 4-module navigation structure with placeholder lessons visible
- [ ] ✅ GitHub Pages live with automatic deployment on main branch push
- [ ] ✅ Environment variables managed securely (.env.local git-ignored)
- [ ] ✅ Lighthouse Performance ≥90
- [ ] ✅ Mobile-responsive design verified
- [ ] ✅ Cross-browser compatibility confirmed
- [ ] ✅ Ready to hand off to Phase 2 (content creation)

**Total Time**: 4-5 hours | **Deliverable**: Live book on GitHub Pages | **Next**: Phase 2 (Content Creation, 6-18 hours)

---

**Status**: ✅ Tasks ready for implementation
