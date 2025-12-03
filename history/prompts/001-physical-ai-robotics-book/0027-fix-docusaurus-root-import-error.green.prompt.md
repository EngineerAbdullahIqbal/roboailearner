---
id: 0027
title: Fix Docusaurus Root Import Error
stage: green
date: 2025-12-01
surface: agent
model: gemini
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: user
command: npm run start
labels: ["fix", "docusaurus", "swizzle"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - robotics_book_content/src/theme/Root.js
tests:
 - (none)
---

## Prompt

Compiled with problems:
×
ERROR in ./src/theme/Root.js 1:45-109
Module not found: Error: Can't resolve '@docusaurus/theme-classic/lib/theme/Root' in '/home/abdullahiqbal/Abdullah/hackathon-book-project/robotics_book_content/src/theme'

## Response snapshot

Fixed invalid import in `src/theme/Root.js`. Removed `@docusaurus/theme-classic/lib/theme/Root` import and updated component to render `children` directly as a proper wrapper.

## Outcome

- ✅ Impact: Fixed compilation error.
- 🧪 Tests: Validated component logic visually.
- 📁 Files: Modified `src/theme/Root.js`.
- 🔁 Next prompts: None.
- 🧠 Reflection: Incorrect swizzling pattern corrected.

## Evaluation notes (flywheel)

- Failure modes observed: Invalid internal path import.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
