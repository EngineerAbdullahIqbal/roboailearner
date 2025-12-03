from qdrant_client import QdrantClient, AsyncQdrantClient
from src.core.config import settings

# Synchronous client for some ops if needed
client = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
)

# Async client for main API
async_client = AsyncQdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
)

async def get_qdrant_client() -> AsyncQdrantClient:
    return async_client
