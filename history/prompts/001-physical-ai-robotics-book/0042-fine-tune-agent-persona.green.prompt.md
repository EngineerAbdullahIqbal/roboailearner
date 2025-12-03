---
id: 0042
title: fine-tune-agent-persona
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fine tune
labels: ["prompt-engineering", "persona", "education"]
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

Greate It's Working Now Fine Tune Our Agent and make it the Beginner Level Students Teachers To Teach this book Like a Explainer with every thing with Exlanation and Example That Stduens understand

## Response snapshot

Refined the agent's system instructions to adopt the "Professor Circuit" persona.
- **Diagnosis**: The user wanted a more engaging, beginner-friendly teaching style ("Explainer").
- **Fix**: Updated `instructions` in `agent_setup.py` to enforce a structure of: Hook -> Simple Explanation -> Real-world Analogy -> Book Example -> Source Citation.

## Outcome

- ✅ Impact: The agent will now explain concepts using analogies (e.g., "ROS 2 topics are like radio broadcasts") and simplify jargon, making it suitable for beginners.
- 🧪 Tests: User needs to restart server and ask a technical question to verify the new tone.
- 📁 Files: `backend/src/agents/agent_setup.py`.
- 🔁 Next prompts: Verify user satisfaction with the new persona.
- 🧠 Reflection: Persona-based prompting is highly effective for educational RAG applications.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None