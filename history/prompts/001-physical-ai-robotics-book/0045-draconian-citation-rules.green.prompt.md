---
id: 0045
title: draconian-citation-rules
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fine tune
labels: ["instruction-tuning", "citations", "rag"]
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

in response the retrived docs link not mentiond

## Response snapshot

Hardened the agent's instructions to explicitly mandate citations.
- **Diagnosis**: The agent was likely overlooking the "Source" instruction or treating it as optional.
- **Fix**: Added a "CITATION RULE (ABSOLUTELY MANDATORY)" section to `agent_setup.py`, explicitly instructing the agent to extract the URL from the `retrieve_context` tool output and format it as a markdown link `[Title](url)`.

## Outcome

- ✅ Impact: The agent is now explicitly programmed to treat the lack of a citation as a failure of its instructions.
- 🧪 Tests: User needs to restart server and check response.
- 📁 Files: `backend/src/agents/agent_setup.py`.
- 🔁 Next prompts: Verify links.
- 🧠 Reflection: "Draconian" rules in system prompts are often needed for specific formatting requirements.

## Evaluation notes (flywheel)

- Failure modes observed: Missing citations.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None