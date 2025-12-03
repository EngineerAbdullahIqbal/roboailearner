---
id: 0044
title: enforce-source-links
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fine tune
labels: ["instruction-tuning", "citations", "ux"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/agents/agent_setup.py
tests:
 - (none)
---

## Prompt

Greate Now Capable oue chat bot When Agent answer from our book it's giving link of retriving docs on response

## Response snapshot

Updated agent instructions to strictly enforce source citations with Markdown links.
- **Diagnosis**: User requested that the agent include links to retrieved documents in its responses.
- **Fix**: Modified `agent_setup.py` to mandate the format `Source: [Chapter Title](url)` at the end of all book-based answers, including short definitions.

## Outcome

- ✅ Impact: The agent will consistently provide clickable links to the specific book chapters it references, improving navigation and trust.
- 🧪 Tests: User needs to restart server and verify links appear in the UI.
- 📁 Files: `backend/src/agents/agent_setup.py`.
- 🔁 Next prompts: Verify link correctness.
- 🧠 Reflection: Explicit formatting instructions are necessary for consistent output structure from LLMs.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None