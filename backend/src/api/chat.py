from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Optional, Dict
from uuid import UUID

from src.agents.rag_agent import chat_with_agent
from src.services.history_service import HistoryService, get_history_service

router = APIRouter()

class ChatRequest(BaseModel):
    message: str
    conversation_id: Optional[UUID] = None
    selection_context: Optional[Dict] = None # {text: str, source_url: str}

@router.post("/chat")
async def chat_endpoint(
    request: ChatRequest,
    history_service: HistoryService = Depends(get_history_service)
):
    """
    Chat endpoint that supports RAG and streaming response.
    Persists messages to HistoryService.
    """
    print(f"Received chat request: {request.message[:50]}...") # Debug log
    user_id = "test_user" # Placeholder
    
    try:
        # 1. Handle Conversation ID
        conversation_id = request.conversation_id
        if not conversation_id:
            print("Creating new conversation...") # Debug log
            # Create new conversation if not provided
            await history_service.create_user_if_not_exists(user_id, "test@example.com")
            conversation_id = await history_service.create_conversation(
                user_id, 
                title=request.message[:50]
            )
            print(f"Conversation created: {conversation_id}") # Debug log
        
        # 2. Save User Message
        await history_service.add_message(
            conversation_id=conversation_id,
            role="user",
            content=request.message,
            metadata=request.selection_context or {}
        )
        
        # 3. Retrieve History for Context
        # Get last 10 messages for context window
        history_messages = await history_service.get_conversation_messages(conversation_id)
        formatted_history = [
            {"role": msg.role, "content": msg.content} 
            for msg in history_messages[-10:] 
        ]
        
        # 4. Prepare Selection Context String
        selection_str = None
        if request.selection_context:
            selection_str = f"Selected Text: \"{request.selection_context.get('text')}\"\nSource: {request.selection_context.get('source_url')}"

        # 5. Stream Generator Wrapper
        async def response_generator():
            full_response = ""
            try:
                print("Starting agent stream...") # Debug log
                # Yield conversation ID first as a meta-event (optional, or handle in frontend)
                # For simplicity, we just stream text content. 
                # Ideally, use Server-Sent Events (SSE) for structured data.
                
                async for chunk in chat_with_agent(formatted_history, selection_context=selection_str):
                    full_response += chunk
                    yield chunk
                
                print("Agent stream finished.") # Debug log
                # 6. Save Assistant Response (After streaming completes)
                await history_service.add_message(
                    conversation_id=conversation_id,
                    role="assistant",
                    content=full_response
                )
                
            except Exception as e:
                print(f"Error in stream: {e}") # Debug log
                yield f"\n[Error: {str(e)}]"

        return StreamingResponse(response_generator(), media_type="text/plain")
    except Exception as e:
        print(f"Error in chat endpoint setup: {e}") # Debug log
        raise HTTPException(status_code=500, detail=str(e))
