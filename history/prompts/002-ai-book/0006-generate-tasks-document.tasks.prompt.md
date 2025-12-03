---
id: {{ID}}
title: {{TITLE}}
stage: {{STAGE}}
date: {{DATE_ISO}}
surface: {{SURFACE}}
model: {{MODEL}}
feature: {{FEATURE}}
branch: {{BRANCH}}
user: {{USER}}
command: {{COMMAND}}
labels: [{{LABELS}}]
links:
  spec: {{LINKS_SPEC}}
  ticket: {{LINKS_TICKET}}
  adr: {{LINKS_ADR}}
  pr: {{LINKS_PR}}
files:
{{FILES_YAML}}
tests:
{{TESTS_YAML}}
---

## Prompt

```text
$ARGUMENTS
```

## Response snapshot

```text
The tasks.md document has been successfully generated.

**Total Tasks**: 41
**Tasks per User Story**:
- US1: 2
- US2: 2
- US3: 2
- US4: 2
- US5: 2
- US6: 3
**Parallel Opportunities Identified**: Yes, explicitly marked with [P]
**Independent Test Criteria**: Defined for each user story within `tasks.md`
**Suggested MVP Scope**: Focus on Phase 1 (Setup) and Phase 2 (Foundational) first, then User Stories 1-3 from Phase 3 (P1).
**Format Validation**: All tasks adhere to the strict checklist format `- [ ] [TaskID] [P?] [Story?] Description with file path`.
```

## Outcome

- ✅ Impact: A comprehensive `tasks.md` document has been generated, providing a detailed, actionable plan for implementing the AI-Generated Book project.
- 🧪 Tests: N/A
- 📁 Files: specs/001-ai-book/tasks.md
- 🔁 Next prompts: The user is ready to proceed with implementing the tasks outlined in `tasks.md`.
- 🧠 Reflection: The task generation process successfully leveraged the spec, plan, data model, contracts, and research to create a structured and executable task list.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
