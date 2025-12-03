import asyncio
import sys
import os
import traceback
from dotenv import load_dotenv

# Force load .env
load_dotenv("backend/.env")

# Add backend to sys.path
sys.path.append(os.path.join(os.getcwd(), "backend"))

from src.agents.tools import retrieve_context
from src.core.config import settings

async def main():
    print("--- RAG Debug Tool ---")
    print(f"Project: {settings.PROJECT_NAME}")
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    print(f"OpenAI Key Present: {bool(settings.OPENAI_API_KEY)}")
    
    query = "Module 1: Foundations of Physical AI & ROS 2"
    print(f"\nTesting retrieval for query: '{query}'")
    
    try:
        result = await retrieve_context(query)
        print("\n--- Result ---")
        print(result[:500] + "..." if len(result) > 500 else result)
        
        if "Error retrieving context" in result:
            print("\n❌ RAG Tool returned an error message.")
        else:
            print("\n✅ RAG Tool successfully returned content.")
            
    except Exception as e:
        print(f"\n❌ Exception during retrieval: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())
