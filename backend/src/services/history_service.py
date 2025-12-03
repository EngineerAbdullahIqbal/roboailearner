from uuid import UUID
from typing import List, Optional, Dict
from sqlalchemy import select, insert, update
from sqlalchemy.ext.asyncio import AsyncSession
from src.db.neon import AsyncSessionLocal
from pydantic import BaseModel
from datetime import datetime
import json

# Models (Pydantic for service layer return types)
class Message(BaseModel):
    id: UUID
    conversation_id: UUID
    role: str
    content: str
    metadata: Dict
    created_at: datetime

class Conversation(BaseModel):
    id: UUID
    user_id: Optional[str]
    title: Optional[str]
    created_at: datetime
    updated_at: datetime

# Simple raw SQL queries or SQLAlchemy Core usage since we didn't define ORM models in schema phase
# Using SQLAlchemy Core with the async engine is robust.

class HistoryService:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_user_if_not_exists(self, user_id: str, email: str):
        # UPSERT style logic
        stmt = text("""
            INSERT INTO users (id, email) VALUES (:id, :email)
            ON CONFLICT (id) DO UPDATE SET last_seen = NOW()
        """)
        await self.db.execute(stmt, {"id": user_id, "email": email})
        await self.db.commit()

    async def create_conversation(self, user_id: str, title: Optional[str] = "New Chat") -> UUID:
        stmt = text("""
            INSERT INTO conversations (user_id, title) 
            VALUES (:user_id, :title) 
            RETURNING id
        """)
        result = await self.db.execute(stmt, {"user_id": user_id, "title": title})
        await self.db.commit()
        return result.scalar()

    async def add_message(self, conversation_id: UUID, role: str, content: str, metadata: Dict = {}) -> UUID:
        stmt = text("""
            INSERT INTO messages (conversation_id, role, content, metadata)
            VALUES (:conversation_id, :role, :content, :metadata)
            RETURNING id
        """)
        result = await self.db.execute(stmt, {
            "conversation_id": conversation_id,
            "role": role,
            "content": content,
            "metadata": json.dumps(metadata) # Ensure JSON serialization
        })
        
        # Update conversation timestamp
        update_stmt = text("""
            UPDATE conversations SET updated_at = NOW() WHERE id = :id
        """)
        await self.db.execute(update_stmt, {"id": conversation_id})
        
        await self.db.commit()
        return result.scalar()

    async def get_conversation_messages(self, conversation_id: UUID) -> List[Message]:
        stmt = text("""
            SELECT id, conversation_id, role, content, metadata, created_at
            FROM messages
            WHERE conversation_id = :conversation_id
            ORDER BY created_at ASC
        """)
        result = await self.db.execute(stmt, {"conversation_id": conversation_id})
        rows = result.mappings().all()
        return [Message(**row) for row in rows]

    async def get_user_conversations(self, user_id: str) -> List[Conversation]:
        stmt = text("""
            SELECT id, user_id, title, created_at, updated_at
            FROM conversations
            WHERE user_id = :user_id
            ORDER BY updated_at DESC
        """)
        result = await self.db.execute(stmt, {"user_id": user_id})
        rows = result.mappings().all()
        return [Conversation(**row) for row in rows]

# Helper to get service with a fresh session
async def get_history_service():
    async with AsyncSessionLocal() as session:
        yield HistoryService(session)

# Need to import text for the SQL execution
from sqlalchemy import text
