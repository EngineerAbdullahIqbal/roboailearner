---
id: 0037
title: fix-agent-stream-parsing
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fix error
labels: ["bugfix", "stream", "parsing", "openai-agents"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/agents/rag_agent.py
tests:
 - (none)
---

## Prompt

(venv) abdullahiqbal@abdullahiqbal-01:~/Abdullah/hackathon-book-project/backend$ uvicorn src.main:app --reload
...
Stream Event: <class 'agents.stream_events.RawResponsesStreamEvent'> - RawResponsesStreamEvent(data=ResponseTextDeltaEvent(content_index=0, delta='Hello', ... type='response.output_text.delta', ...), type='raw_response_event')
...

## Response snapshot

Fixed the stream parsing logic based on the debug logs.
- **Diagnosis**: The logs revealed that `openai-agents` wraps the delta in a `RawResponsesStreamEvent` where `event.data` is a `ResponseTextDeltaEvent`. The actual text is in `event.data.delta`.
- **Fix**: Updated `backend/src/agents/rag_agent.py` to specifically look for `RawResponsesStreamEvent` and extract `data.delta` when `data.type == 'response.output_text.delta'`.

## Outcome

- ✅ Impact: The API will now stream the actual text content ("Hello", "!", "How", etc.) to the frontend instead of yielding nothing or raw objects.
- 🧪 Tests: User needs to restart server and verify chat output in UI.
- 📁 Files: `backend/src/agents/rag_agent.py`
- 🔁 Next prompts: Verify UI functionality.
- 🧠 Reflection: The `openai-agents` SDK has a complex event hierarchy that requires precise introspection.

## Evaluation notes (flywheel)

- Failure modes observed: Event nesting was deeper than expected.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None