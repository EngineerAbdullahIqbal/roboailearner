from fastapi import APIRouter, Depends, HTTPException, status
from src.services.ingest_service import ingest_documents
from src.core.security import verify_api_key

router = APIRouter()

@router.post("/ingest", summary="Ingest documents into Qdrant vector database")
async def ingest_data(
    target: str = "all", # Could be specific chapter ID later
    # Currently verify_api_key is a placeholder, needs proper auth for production
    _ = Depends(verify_api_key) 
):
    """
    Ingests all available markdown documents from the textbook content
    into the Qdrant vector database.
    """
    try:
        result = await ingest_documents()
        return {"message": "Ingestion successful", **result}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Ingestion failed: {e}"
        )
