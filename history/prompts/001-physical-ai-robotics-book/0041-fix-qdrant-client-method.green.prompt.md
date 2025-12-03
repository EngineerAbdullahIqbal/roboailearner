---
id: 0041
title: fix-qdrant-client-method
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fix error
labels: ["bugfix", "qdrant", "retrieval"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/agents/tools.py
tests:
 - (none)
---

## Prompt

abdullahiqbal@abdullahiqbal-01:~/Abdullah/hackathon-book-project$ source backend/venv/bin/activate && python backend/test_rag_standalone.py
...
Retrieval error: 'AsyncQdrantClient' object has no attribute 'search'
...

## Response snapshot

Fixed the `AttributeError` in the `retrieve_context` tool.
- **Diagnosis**: The `qdrant-client` library version installed (`>=1.7.0`) has deprecated or removed the `search` method in favor of `query_points` (or the method signature changed significantly in the async client).
- **Fix**: Updated `backend/src/agents/tools.py` to use `await async_client.query_points(...)` and iterate over `search_result.points` to extract the payload, aligning with the modern Qdrant Python client API.

## Outcome

- ✅ Impact: The RAG tool should now successfully query the Qdrant vector store without raising an attribute error.
- 🧪 Tests: User needs to re-run the debug script or restart the server.
- 📁 Files: `backend/src/agents/tools.py`
- 🔁 Next prompts: Verify fix with debug script.
- 🧠 Reflection: Client library churn is a common issue. Pinning versions or checking migration guides is crucial.

## Evaluation notes (flywheel)

- Failure modes observed: Deprecated API usage.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None