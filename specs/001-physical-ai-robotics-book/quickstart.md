# Quickstart Guide: OpenAI Agents SDK/ChatKit Integration

This guide provides a quick overview of how to set up and run the integrated RAG chatbot with OpenAI Agents SDK and ChatKit for the Physical AI & Humanoid Robotics textbook. It covers both backend and frontend setup.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

- Python 3.11+
- Node.js (LTS recommended)
- npm or yarn
- Docker (for local Neon Postgres and Qdrant setup, if not using cloud services)
- OpenAI API Key (for Agents SDK)
- Gemini API Key (for embeddings and generation)
- Qdrant Cloud API Key and Cluster URL (if using cloud; otherwise, local Docker setup)
- Neon Postgres Connection String

## 2. Backend Setup (FastAPI)

1.  **Navigate to the backend directory:**
    ```bash
    cd backend
    ```
2.  **Create a virtual environment and install dependencies:**
    ```bash
    python -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    ```
3.  **Configure environment variables:**
    Create a `.env` file in the `backend/` directory with the following:
    ```
    OPENAI_API_KEY="your_openai_api_key"
    GEMINI_API_KEY="your_gemini_api_key"
    QDRANT_URL="your_qdrant_cloud_url"  # e.g., https://<cluster_id>.qdrant.tech
    QDRANT_API_KEY="your_qdrant_cloud_api_key"
    NEON_POSTGRES_URL="your_neon_postgres_connection_string"
    ```
4.  **Run database migrations (if any):**
    *(Assuming `alembic` or similar tool is configured)*
    ```bash
    alembic upgrade head
    ```
5.  **Start the FastAPI application:**
    ```bash
    uvicorn src.api.main:app --reload
    ```
    The backend API will be running at `http://127.0.0.1:8000` (or specified port).

## 3. Data Ingestion

1.  **Ensure your markdown content is in `robotics_book_content/docs/`.
2.  **Run the ingestion script:**
    ```bash
    cd backend
    source .venv/bin/activate
    python scripts/ingest.py
    ```
    This will chunk your documentation and upload embeddings to Qdrant.

## 4. Frontend Setup (Docusaurus/React)

1.  **Navigate to the frontend directory:**
    ```bash
    cd robotics_book_content
    ```
2.  **Install dependencies:**
    ```bash
    npm install # or yarn install
    ```
3.  **Start the Docusaurus development server:**
    ```bash
    npm start # or yarn start
    ```
    The frontend will be accessible at `http://localhost:3000` (or specified port).

## 5. Interacting with the Chatbot

- Open your browser to the Docusaurus site (`http://localhost:3000`).
- The Chat Widget (FAB) should be visible. Click it to open the chat interface.
- You can:
    - Type queries for standard RAG interactions.
    - Select text on the page and then click the FAB to get context-aware explanations.
    - Engage in multi-turn conversations with the integrated OpenAI Agents via the `/agent-chat` endpoint through the chat interface.