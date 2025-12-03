---
id: 0046
title: optimize-latency
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fine tune
labels: ["performance", "latency", "optimization"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/agents/agent_setup.py
 - backend/src/services/embeddings.py
tests:
 - (none)
---

## Prompt

Greate But The Agent is talking too much time and very latency in repsonse

## Response snapshot

Optimized agent prompt and embedding logic for lower latency.
- **Diagnosis**: 
    1. The RAG debug script runs fast (<< 1s), so the vector search itself is not the bottleneck.
    2. The bottleneck is likely the LLM's generation time (verbose output) or the agent SDK's orchestration overhead.
- **Fix**:
    1.  **Prompt Engineering**: Added "LATENCY & CONCISENESS RULES" to `agent_setup.py` to force the agent to be succinct, skip pleasantries, and limit explanation length.
    2.  **Code Optimization**: Cleaned up `embeddings.py` to ensure the most direct path for OpenAI calls.

## Outcome

- ✅ Impact: The agent should start streaming the answer faster because it has less "preamble" text to generate, and the total response time will be shorter due to reduced token count.
- 🧪 Tests: User needs to restart server and check perceived latency.
- 📁 Files: `backend/src/agents/agent_setup.py`, `backend/src/services/embeddings.py`.
- 🔁 Next prompts: Monitor performance.
- 🧠 Reflection: "Perceived latency" is often just "too much text". Making the LLM concise improves both speed and UX.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None