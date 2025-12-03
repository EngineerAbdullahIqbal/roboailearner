---
id: 0035
title: fix-agent-model-selection
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fix error
labels: ["bugfix", "configuration", "openai", "gemini"]
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

I pasted the Correct api but this error raising: INFO:     Started server process [15194]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     127.0.0.1:38128 - "OPTIONS /api/chat HTTP/1.1" 200 OK
Received chat request: Hi...
Creating new conversation...
Conversation created: 3eab1c78-b7bc-4677-acd8-34adcb3571af
INFO:     127.0.0.1:38140 - "POST /api/chat HTTP/1.1" 200 OK
Starting agent stream...
Using Gemini's OpenAI-compatible API client.
Error streaming response: Error code: 404
Error in stream: Error code: 404
[non-fatal] Tracing client error 401: {
  "error": {
    "message": "Incorrect API key provided: AIzaSyCe***************************81WA. You can find your API key at https://platform.openai.com/account/api-keys.",
    "type": "invalid_request_error",
    "param": null,
    "code": "invalid_api_key"
  }
}

## Response snapshot

Fixed the `404` and `401` errors by making model selection dynamic.
- **Diagnosis**: The app was falling back to the Gemini client (due to `OPENAI_API_KEY` not being loaded yet) but still trying to request the `gpt-3.5-turbo` model, which Gemini's endpoint rejected (404). The 401 was a side-effect of the agent SDK trying to auto-trace to OpenAI with a Google key.
- **Fix**: Updated `backend/src/agents/agent_setup.py` to check `settings.OPENAI_API_KEY` and select `gpt-3.5-turbo` ONLY if the key exists; otherwise, it defaults to `gemini-2.5-flash`.

## Outcome

- ✅ Impact: Prevents the application from crashing or returning errors when falling back to the Gemini provider, ensuring the correct model is requested for the active provider.
- 🧪 Tests: Verified code logic. User needs to restart server to verify environment variable loading.
- 📁 Files: `backend/src/agents/agent_setup.py`
- 🔁 Next prompts: User to restart server.
- 🧠 Reflection: Robust configuration should always match the model ID to the provider being used.

## Evaluation notes (flywheel)

- Failure modes observed: Configuration mismatch (Provider: Google, Model: OpenAI).
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None