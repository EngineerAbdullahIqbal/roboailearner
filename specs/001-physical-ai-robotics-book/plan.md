# Implementation Plan: OpenAI Agents SDK/ChatKit Integration

**Branch**: `001-physical-ai-robotics-book` | **Date**: 2025-12-01 | **Spec**: [specs/001-physical-ai-robotics-book/spec.md](specs/001-physical-ai-robotics-book/spec.md)
**Input**: Integrate OpenAI Agents SDK / ChatKit with existing backend agent and RAG system for the Physical AI & Humanoid Robotics textbook. This includes modifications to the FastAPI backend, Qdrant vector database, and the overall RAG pipeline to effectively utilize OpenAI Agents SDK and ChatKit, focusing on architectural changes, data flow, and necessary refactoring.

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the integration of OpenAI Agents SDK and ChatKit into the existing RAG chatbot for the Physical AI & Humanoid Robotics textbook. The goal is to enhance the chatbot's capabilities by leveraging agent-orchestrated workflows for more complex, multi-turn interactions, while maintaining the RAG system's ability to provide accurate, book-context-only answers. This involves refining the data ingestion pipeline, modifying the FastAPI backend with new endpoints, and updating the React frontend for seamless user experience.

## Technical Context

**Language/Version**: Python 3.11+, JavaScript (React/TypeScript)
**Primary Dependencies**: OpenAI Agents SDK, ChatKit, FastAPI, Qdrant, Gemini API, Docusaurus 3.x
**Storage**: Neon Serverless Postgres (chat history, agent state), Qdrant Cloud Free Tier (vector store)
**Testing**: `pytest` (backend), `React Testing Library` (frontend)
**Target Platform**: Linux server (backend), Web browsers (frontend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Low latency for RAG queries (p95 < 500ms), efficient agent interaction
**Constraints**: Utilize existing RAG pipeline components (FastAPI, Qdrant, Gemini). Cost-effective solutions (Qdrant Free Tier, Gemini 2.5 Flash).
**Scale/Scope**: Support concurrent users for book-specific RAG queries and agent interactions.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Spec-Driven Development
- **Adherence**: This plan is directly derived from the user's request and the implicit requirements for integrating OpenAI Agents SDK/ChatKit into the existing RAG system, ensuring alignment with the project's overall goal.

### II. Traceability & Documentation
- **Adherence**: Architectural decisions regarding the RAG and agent integration will be documented in ADRs, and PHRs will be created for each significant step.

### III. Testability & Verification
- **Adherence**: The plan includes distinct endpoints and frontend components, allowing for isolated testing of the RAG queries and agent interactions. Code examples for OpenAI Agents SDK/ChatKit will be verifiable.

### IV. Minimalism & Focus
- **Adherence**: The plan focuses solely on integrating the OpenAI Agents SDK/ChatKit into the existing RAG and backend, avoiding unrelated refactoring or features.

### V. Explicit Contracts
- **Adherence**: New API endpoints for agent interaction will have clearly defined contracts (e.g., OpenAPI schema) to ensure explicit interfaces.

### VI. Security First
- **Adherence**: API keys and sensitive configurations will be managed securely on the backend (environment variables) and rate limiting will be applied.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/             # FastAPI endpoints (including /chat, /explain-selection, /agent-chat)
│   ├── models/          # Data models for RAG, chat history, agent state
│   ├── services/        # Business logic, RAG pipeline, agent orchestration
│   └── config/          # Configuration files (e.g., Qdrant, Gemini API settings)
└── tests/
    ├── integration/
    └── unit/

robotics_book_content/
├── src/
│   ├── components/      # React components (e.g., ChatWidget.js)
│   ├── theme/           # Docusaurus theme overrides (e.g., Root.js for global state)
│   ├── pages/
│   └── data/
├── docs/                # Markdown content for the book
└── static/

.specify/
├── scripts/
├── memory/
└── templates/

history/
├── adr/
└── prompts/
```

**Structure Decision**: The project will follow a web application structure with a `backend/` for the FastAPI application and `robotics_book_content/` for the Docusaurus frontend. This separation aligns with the existing project setup and clearly delineates responsibilities. The `backend/src/api` will house the new OpenAI Agents SDK/ChatKit integration endpoints, `backend/src/services` will contain the RAG pipeline and agent orchestration logic, and `robotics_book_content/src/components` will include the `ChatWidget.js` for frontend interaction.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
