import google.generativeai as genai
from openai import AsyncOpenAI
from src.core.config import settings
from typing import List
import time

# Global Singleton Client
openai_client = None

def get_openai_client():
    global openai_client
    if openai_client is None and settings.OPENAI_API_KEY:
        openai_client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
    return openai_client

# Configure Gemini if key is present
if settings.GOOGLE_API_KEY:
    genai.configure(api_key=settings.GOOGLE_API_KEY)

async def get_embeddings(texts: List[str]) -> List[List[float]]:
    results = []
    
    # Option 1: Use OpenAI if available
    client = get_openai_client()
    if client:
        try:
            response = await client.embeddings.create(
                input=texts,
                model="text-embedding-3-small",
                dimensions=768 
            )
            return [data.embedding for data in response.data]
        except Exception as e:
            print(f"OpenAI Embedding error: {e}")
            # Fallback proceeds to Gemini loop
            pass

    # Option 2: Use Gemini (Sync fallback, ideally should be async too but keeping simple for now)
    # Note: genai lib is sync-heavy. If OpenAI fails, this might still block, 
    # but primary path is now non-blocking.
    for text in texts:
        safe_text = text[:10000] 
        try:
            # Run sync call in thread pool if needed, but for fallback it's acceptable
            result = genai.embed_content(
                model="models/embedding-001",
                content=safe_text,
                task_type="retrieval_document",
            )
            results.append(result['embedding'])
            time.sleep(1) 
        except Exception as e:
            print(f"Gemini Embedding error: {e}")
            results.append([0.0]*768)
            
    return results

