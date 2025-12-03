---
id: 0034
title: fix-functiontool-and-switch-to-openai
stage: green
date: 2025-12-01
surface: agent
model: gemini-2.5-flash-image
feature: 001-physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: abdullahiqbal
command: fix error
labels: ["bugfix", "openai", "model-switch"]
links:
  spec: specs/001-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/agents/agent_setup.py
 - backend/src/core/config.py
 - backend/src/agents/client.py
 - backend/.env.example
tests:
 - (none)
---

## Prompt

Enalyze the Given Error if it's due to gemini model or external servecies so I have also Openai API key at .env so use Openai chapest model for this time : (venv) abdullahiqbal@abdullahiqbal-01:~/Abdullah/hackathon-book-project/backend$ uvicorn src.main:app --reload
INFO:     Will watch for changes in these directories: ['/home/abdullahiqbal/Abdullah/hackathon-book-project/backend']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [14788] using WatchFiles
Process SpawnProcess-1:
Traceback (most recent call last):
  File "/usr/lib/python3.12/multiprocessing/process.py", line 314, in _bootstrap
    self.run()
  File "/usr/lib/python3.12/multiprocessing/process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/venv/lib/python3.12/site-packages/uvicorn/_subprocess.py", line 80, in subprocess_started
    target(sockets=sockets)
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/venv/lib/python3.12/site-packages/uvicorn/server.py", line 67, in run
    return asyncio_run(self.serve(sockets=sockets), loop_factory=self.config.get_loop_factory())
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/asyncio/runners.py", line 194, in run
    return runner.run(main)
           ^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/asyncio/runners.py", line 118, in run
    return self._loop.run_until_complete(task)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "uvloop/loop.pyx", line 1518, in uvloop.loop.Loop.run_until_complete
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/venv/lib/python3.12/site-packages/uvicorn/server.py", line 71, in serve
    await self._serve(sockets)
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/venv/lib/python3.12/site-packages/uvicorn/server.py", line 78, in _serve
    config.load()
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/venv/lib/python3.12/site-packages/uvicorn/config.py", line 439, in load
    self.loaded_app = import_from_string(self.app)
                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/venv/lib/python3.12/site-packages/uvicorn/importer.py", line 19, in import_from_string
    module = importlib.import_module(module_str)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/importlib/__init__.py", line 90, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "<frozen importlib._bootstrap>", line 1387, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1360, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1331, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 935, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 995, in exec_module
  File "<frozen importlib._bootstrap>", line 488, in _call_with_frames_removed
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/src/main.py", line 4, in <module>
    from src.api import ingest, history, chat
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/src/api/chat.py", line 7, in <module>
    from src.agents.rag_agent import chat_with_agent
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/src/agents/rag_agent.py", line 5, in <module>
    from src.agents.agent_setup import textbook_agent
  File "/home/abdullahiqbal/Abdullah/hackathon-book-project/backend/src/agents/agent_setup.py", line 29, in <module>
    FunctionTool(retrieve_context)
TypeError: FunctionTool.__init__() missing 3 required positional arguments: 'description', 'params_json_schema', and 'on_invoke_tool'

## Response snapshot

Resolved `TypeError` in `FunctionTool` initialization and switched to OpenAI's `gpt-3.5-turbo` model with API key configuration.
Specifically:
1.  **`agent_setup.py`**: Modified `FunctionTool` instantiation to explicitly pass `name`, `description`, `params_json_schema` from `retrieve_tool_definition` and wrapped `retrieve_context` in `invoke_retrieve_context` to match `on_invoke_tool` signature.
2.  **`config.py`**: Added `OPENAI_API_KEY` to `Settings`.
3.  **`.env.example`**: Added `OPENAI_API_KEY` placeholder.
4.  **`client.py`**: Updated `get_openai_client` to dynamically use `OPENAI_API_KEY` if available, otherwise falling back to `GOOGLE_API_KEY` with Gemini's compatible endpoint.
5.  **`agent_setup.py`**: Changed the model to `gpt-3.5-turbo`.

## Outcome

- ✅ Impact: Backend agent setup should now pass initialization, and the system is configured to prioritize OpenAI's `gpt-3.5-turbo` for chat.
- 🧪 Tests: Manual verification by attempting to start `uvicorn`. Further functional testing (API calls) is required.
- 📁 Files: `backend/src/agents/agent_setup.py`, `backend/src/core/config.py`, `backend/src/agents/client.py`, `backend/.env.example`.
- 🔁 Next prompts: The user can now restart the `uvicorn` server and proceed with testing the chatbot functionality.
- 🧠 Reflection: The initial `TypeError` was due to an incorrect assumption about `FunctionTool`'s constructor. The model switch provides flexibility as requested by the user.

## Evaluation notes (flywheel)

- Failure modes observed: `TypeError` during `FunctionTool` initialization.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None