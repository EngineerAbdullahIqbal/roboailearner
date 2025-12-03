# Tasks: Integrate OpenAI Agents SDK / ChatKit

**Feature Branch**: `001-physical-ai-robotics-book`
**Plan**: specs/001-physical-ai-robotics-book/plan.md
**Context**: Backend RAG Agent Integration

This document outlines the tasks to integrate the OpenAI Agents SDK (and ChatKit) into the existing FastAPI backend, replacing/enhancing the current manual tool-calling implementation.

## Implementation Strategy

We will refactor the `backend/src/agents/` module to use the official OpenAI Agents SDK patterns (or `openai` Assistants API if that's the intended target, but adhering to "Agents SDK" phrasing). We will ensure compatibility with the existing Qdrant RAG retrieval.

## Dependency Graph

1.  **Setup**: Dependencies and Config
2.  **Agent Core**: Refactoring `rag_agent.py` and `client.py`
3.  **API Layer**: Updating `chat.py` to support the new agent flow
4.  **Verification**: Testing

## Phase 1: Setup & Dependencies

**Goal**: Ensure correct libraries are installed and configured.

- [ ] T001 Update `backend/requirements.txt` to include `openai-agents` (if available) or ensure `openai` version is compatible with Agents features.
- [ ] T002 [P] Verify environment variables in `.env` (OPENAI_API_KEY, etc.) are set for the SDK.

## Phase 2: Agent Implementation

**Goal**: Implement the RAG agent using the requested SDK.

- [ ] T003 [P] Create/Update `backend/src/agents/agent_setup.py` (or similar) to define the Agent, instructions, and Tools using the SDK.
- [ ] T004 Refactor `backend/src/agents/rag_agent.py` to utilize the SDK for orchestration instead of manual tool loops.
- [ ] T005 [P] Ensure `retrieve_context` tool in `backend/src/agents/tools.py` is correctly typed/formatted for the SDK.

## Phase 3: API Integration

**Goal**: Expose the Agent via FastAPI.

- [ ] T006 Update `backend/src/api/chat.py` to interact with the Agent (create thread, add message, run) and stream responses back.
- [ ] T007 Update `ChatRequest` model to handle any new fields required by the Agent SDK (if any).

## Phase 4: Testing & Polish

**Goal**: Verify functionality.

- [ ] T008 Test RAG retrieval through the new Agent.
- [ ] T009 Verify streaming responses work correctly.
