# Research Findings: OpenAI Agents SDK/ChatKit Integration

This document summarizes the research and architectural decisions made for integrating OpenAI Agents SDK and ChatKit into the existing RAG chatbot for the Physical AI & Humanoid Robotics textbook. The primary goal is to enhance conversational capabilities through agent orchestration while maintaining the RAG system's factual grounding.

## 1. Data Ingestion Pipeline

- **Decision**: Implement a custom `MarkdownHeaderSplitter`.
- **Rationale**: Preserves semantic units (headers, code blocks, Docusaurus Admonitions) in `docs/**/*.md` to ensure contextual integrity during retrieval, crucial for technical content.
- **Alternatives Considered**: Basic `RecursiveCharacterTextSplitter` (rejected due to potential for breaking semantic units in Markdown).
- **Components**: Gemini `embedding-001` for embeddings, Qdrant Cloud (`robotics_book` collection) for vector storage. Optimized payload structure to conserve Qdrant free tier limits.

## 2. Backend API (FastAPI)

- **Decision**: Extend existing FastAPI backend with new endpoints.
- **Rationale**: Leverage existing infrastructure and provide dedicated interfaces for different interaction types (general chat, context-aware explanations, agent-orchestrated workflows).
- **Endpoints**:
    - `POST /chat`: Standard RAG conversational queries.
    - `POST /explain-selection`: Context-aware explanations for selected text.
    - `POST /agent-chat`: Entry point for OpenAI Agents SDK/ChatKit multi-turn interactions.
- **Database**: Neon Postgres for chat history and agent interaction logs.
- **Security**: `slowapi` for rate limiting; secure management of API keys (environment variables).

## 3. Retrieval Logic

- **Decision**: Implement a hybrid search strategy.
- **Rationale**: Combines semantic understanding (vector search) with precise term matching (keyword search) to improve retrieval accuracy for technical queries.
- **Components**: Gemini `embedding-001` for query embedding. Qdrant vector search with `score_threshold = 0.75` and top 5-7 chunks. Simple re-ranking to prioritize exact keyword matches.
- **System Prompt**: Designed as a "Teaching Assistant" for the 'Physical AI & Humanoid Robotics' textbook, enforcing \"Book Context Only\" answers and providing clear fallback for insufficient information. Mentions referencing chapter and code snippets.
- **Generation Model**: Gemini `gemini-2.5-flash` for latency and cost efficiency.

## 4. Frontend Integration (Docusaurus/React)

- **Decision**: Integrate ChatKit within the Docusaurus theme, managed through global state.
- **Rationale**: Provides a consistent and accessible chat interface across the textbook. Text selection-based explanations enhance the learning experience.
- **Components**: `Root.js` for global chat state and `mouseup` event listener for text selection. `ChatWidget.js` component to handle chat history, display responses, and provide an interface for `POST /chat`, `POST /explain-selection`, and `POST /agent-chat`.