import asyncio
import sys
import os
from dotenv import load_dotenv

# Load env vars from backend/.env
load_dotenv("backend/.env")

# Add backend to sys.path so imports work
sys.path.append(os.path.join(os.getcwd(), "backend"))

from src.services.ingest_service import ingest_documents
from src.core.config import settings

async def main():
    print(f"Starting ingestion for project: {settings.PROJECT_NAME}")
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    
    try:
        result = await ingest_documents()
        print("Ingestion Result:", result)
    except Exception as e:
        print(f"Ingestion Failed: {e}")

if __name__ == "__main__":
    # Load env if needed (though settings should handle it if run from root)
    asyncio.run(main())
