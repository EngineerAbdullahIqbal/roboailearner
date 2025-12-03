from fastapi import Header, HTTPException, status
from typing import Optional
from src.core.config import settings

# Simple API Key security for ingest endpoint if needed
async def verify_api_key(x_api_key: Optional[str] = Header(None)):
    # For MVP, we might skip strict auth or use a simple secret in env
    # If BETTER_AUTH_SECRET is used as an admin key for ingestion
    if not settings.QDRANT_API_KEY: # Fallback if no specific key set
         return True
         
    # This is a placeholder. in production, use proper auth.
    # For now, we assume internal use or protected by network/vercel
    return True
