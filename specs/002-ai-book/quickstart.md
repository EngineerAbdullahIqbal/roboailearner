# Quickstart Guide: Physical AI & Humanoid Robotics Course

This guide provides instructions to set up and run the "Physical AI & Humanoid Robotics Course" project locally for development.

## Prerequisites

Ensure you have the following installed:

-   **Git**: For cloning the repository.
-   **Node.js** (LTS version): Required for Docusaurus (frontend).
-   **Python 3.11+**: Required for FastAPI (backend).
-   **pip** (Python package installer).
-   **Docker** (Optional, for Neon Postgres and Qdrant local setup if not using cloud services).

## 1. Clone the Repository

First, clone the project repository:

```bash
git clone https://github.com/your-org/your-repo-name.git # Replace with actual repo URL
cd your-repo-name
```

## 2. Set up the Frontend (Docusaurus)

Navigate to the `frontend` directory and install dependencies:

```bash
cd frontend
npm install
```

To run the Docusaurus development server:

```bash
npm start
```

This will open the book in your browser at `http://localhost:3000`.

## 3. Set up the Backend (FastAPI)

Open a new terminal, navigate to the `backend` directory, create a virtual environment, and install dependencies:

```bash
cd backend
python3.11 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Configure your environment variables for Neon Postgres and Qdrant. Create a `.env` file in the `backend` directory:

```dotenv
DATABASE_URL="your_neon_postgres_connection_string"
QDRANT_URL="your_qdrant_cloud_url"
QDRANT_API_KEY="your_qdrant_api_key"
BETTER_AUTH_API_KEY="your_better_auth_api_key"
OPENAI_API_KEY="your_openai_api_key"
```

To run the FastAPI backend:

```bash
source venv/bin/activate
uvicorn app.main:app --reload
```

The API will be available at `http://localhost:8000`.

## 4. Run the RAG Content Ingestion Script

This script ingests the book content into the Qdrant vector database. Ensure your backend is running and you have configured the `.env` file.

```bash
cd scripts
source ../backend/venv/bin/activate # Activate backend venv
python ingest_book_content.py --book-content-path ../frontend/docs
```

This will parse your Docusaurus markdown files and create embeddings in Qdrant.

## 5. Running the Full Application

With both frontend and backend running, you can interact with the full application:

-   Access the book at `http://localhost:3000`.
-   Use the RAG chatbot and authentication features, which will communicate with your local FastAPI backend.
