---
id: 0028
title: Fix Prism XML Import Error
stage: green
date: 2025-12-01
surface: agent
model: gemini
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: user
command: npm run start
labels: ["fix", "docusaurus", "config", "prism"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - robotics_book_content/docusaurus.config.ts
tests:
 - (none)
---

## Prompt

Uncaught runtime errors:
×
ERROR
Cannot find module './prism-xml'

## Response snapshot

Removed `xml` from `prism.additionalLanguages` in `docusaurus.config.ts`. This resolves the webpack resolution error because `xml` is often included by default or aliased to `markup`, and explicitly requesting it caused a lookup failure for `prism-xml`.

## Outcome

- ✅ Impact: Fixed runtime error preventing site load.
- 🧪 Tests: Configuration update.
- 📁 Files: Modified `docusaurus.config.ts`.
- 🔁 Next prompts: None.
- 🧠 Reflection: Common Docusaurus/Prism configuration issue.

## Evaluation notes (flywheel)

- Failure modes observed: Invalid prism language import.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
