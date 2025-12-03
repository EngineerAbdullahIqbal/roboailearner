# ADR-0012: FastAPI RAG Backend Framework

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-11-30
- **Feature:** 001-physical-ai-robotics-book
- **Context:** The RAG chatbot requires a robust, high-performance, and scalable backend API to handle chat requests, process text selections, trigger data ingestion, and manage conversation history. The choice of the backend framework and its core components directly impacts development speed, operational efficiency, and overall system capabilities.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

The backend for the RAG chatbot will be implemented using **FastAPI** in Python. Key components integrated with FastAPI include:
- **Endpoints**: `POST /api/chat`, `POST /api/chat/selection`, `POST /api/ingest`, `GET /api/health`, `WebSocket /ws/chat`.
- **Database**: Neon Serverless Postgres for chat history (`conversations`, `messages` tables) and logging.
- **Vector Database Client**: Qdrant client for interacting with the Qdrant Cloud vector database.
- **Embedding/LLM Integration**: `google.generativeai` library for Gemini `embedding-001` and `gemini-2.5-flash`.
- **Security**: Environment variable-based API key management and `slowapi` for rate limiting (to be fully implemented).
- **Asynchronous Operations**: Leveraging FastAPI's `async/await` support for non-blocking I/O with database and external API calls.

## Consequences

### Positive

- **High Performance**: FastAPI's async nature and Starlette foundation provide excellent performance, crucial for real-time chat and streaming responses.
- **Developer Experience**: Automatic interactive API documentation (Swagger UI/ReDoc) and Pydantic for data validation accelerate development and reduce errors.
- **Scalability**: Designed for cloud-native deployment, easily scalable with serverless platforms (like AWS Lambda, Google Cloud Run) or Docker containers.
- **Python Ecosystem**: Access to a rich ecosystem of Python libraries for AI/ML (e.g., Qdrant client, `google.generativeai`), data processing, and utilities.
- **Clear API Contract**: Pydantic models enforce clear input/output contracts, simplifying frontend integration.

### Negative

- **Learning Curve**: For teams unfamiliar with async Python or FastAPI, there might be an initial learning curve.
- **Deployment Complexity**: While scalable, deploying and managing a FastAPI application with async database connections and WebSockets can be more complex than a simple static site.
- **Authentication Implementation**: Requires careful implementation of authentication/authorization mechanisms (e.g., for the `/api/ingest` endpoint) to prevent security vulnerabilities.

## Alternatives Considered

**1. Flask/Django**: Python web frameworks.
   - **Why rejected**: While robust, they are generally less performant for I/O-bound tasks compared to FastAPI's async capabilities, which are critical for the RAG pipeline's external API calls and real-time features. FastAPI offers a more modern approach for API-first development.

**2. Node.js with Express/NestJS**: JavaScript/TypeScript frameworks.
   - **Why rejected**: While strong for async operations and widely used in web development, Python offers a more mature and direct ecosystem for AI/ML libraries and models (Gemini, Qdrant clients), simplifying direct integration without needing to bridge languages or manage additional services. Also, the core book content generation is Python-centric.

## References

- Feature Spec: /home/abdullahiqbal/Abdullah/hackathon-book-project/specs/rag-chatbot/spec.md
- Implementation Plan: /home/abdullahiqbal/Abdullah/hackathon-book-project/specs/001-physical-ai-robotics-book/plan.md
- Related ADRs: 0009-text-selection-strategy.md, 0010-rag-system-model-choice-and-hybrid-search-strategy.md, 0011-custom-markdown-splitting-strategy.md
- Evaluator Evidence: null
