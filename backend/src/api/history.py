from fastapi import APIRouter, Depends, HTTPException
from typing import List, Optional
from uuid import UUID
from src.services.history_service import HistoryService, get_history_service, Conversation

router = APIRouter()

# In a real app, user_id would come from auth token dependency
# For MVP, we'll accept it as a query param or header for testing
async def get_current_user_id(user_id: str = "test_user"):
    return user_id

@router.get("/conversations", response_model=List[Conversation])
async def get_conversations(
    user_id: str = Depends(get_current_user_id),
    history_service: HistoryService = Depends(get_history_service)
):
    """Get all conversations for the current user."""
    return await history_service.get_user_conversations(user_id)

@router.get("/conversations/{conversation_id}/messages")
async def get_messages(
    conversation_id: UUID,
    history_service: HistoryService = Depends(get_history_service)
):
    """Get all messages for a specific conversation."""
    return await history_service.get_conversation_messages(conversation_id)

@router.post("/conversations")
async def create_conversation(
    title: Optional[str] = "New Chat",
    user_id: str = Depends(get_current_user_id),
    history_service: HistoryService = Depends(get_history_service)
):
    """Create a new conversation."""
    # Ensure user exists first (simple check for MVP)
    await history_service.create_user_if_not_exists(user_id, f"{user_id}@example.com")
    
    conv_id = await history_service.create_conversation(user_id, title)
    return {"id": conv_id, "title": title}
