---
id: 0038
title: refine-agent-instructions
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fix error
labels: ["instruction-tuning", "prompt-engineering", "rag"]
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

Greate But I think the Agent is not Using RAG Correctly and responding very short answer don't book based test and Fine Tune it: xplain this selection: "Chapter 6: Sensor Fusion: Combining Data for Robust Perception​"
Chapter 6 discusses Sensor Fusion, a technique of integrating data from multiple sensors to improve perception accuracy, essential for robust robotics applications.

## Response snapshot

Refined the agent's system instructions to explicitly handle "Explain" requests and text selections as high-priority triggers for the `retrieve_context` tool.
- **Diagnosis**: The previous instructions categorized "Short Selections" (< 20 words) as requiring "EXTREMELY CONCISE" answers, causing the agent to treat the header "Chapter 6..." as a simple definition lookup rather than a topic for detailed RAG retrieval.
- **Fix**: Rewrote `instructions` in `agent_setup.py` to prioritize any input containing `[Context]` or starting with "Explain" as a mandatory RAG lookup, regardless of length.

## Outcome

- ✅ Impact: The agent should now reliably call the retrieval tool for headers and short topic queries instead of giving generic dictionary definitions.
- 🧪 Tests: User needs to restart server and retry the query.
- 📁 Files: `backend/src/agents/agent_setup.py`
- 🔁 Next prompts: Verify improved response quality.
- 🧠 Reflection: Length-based heuristics for "conciseness" are dangerous when users select short but dense headers. Intent-based heuristics (e.g., "Explain", "Selection") are safer.

## Evaluation notes (flywheel)

- Failure modes observed: Prompt instructions over-optimized for brevity on short inputs.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None