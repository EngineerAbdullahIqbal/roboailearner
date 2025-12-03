# Data Model: OpenAI Agents SDK/ChatKit Integration

This document defines the data models for managing chat history and agent-related state within the Neon Postgres database, supporting the OpenAI Agents SDK/ChatKit integration for the Physical AI & Humanoid Robotics textbook chatbot.

## 1. ChatHistory

**Purpose**: Stores the chronological conversation flow between the user and the chatbot/agents.

**Table Name**: `chat_history`

**Fields**:
- `id`: UUID (Primary Key)
- `session_id`: UUID (Foreign Key to UserSession, if applicable; for tracking conversations)
- `user_id`: UUID (Foreign Key to User, if authentication is implemented)
- `timestamp`: TIMESTAMP WITH TIME ZONE (Defaults to `NOW()`; records when the message was sent/received)
- `sender_type`: ENUM ('user', 'chatbot', 'agent')
- `message_content`: TEXT (The actual message text or agent output)
- `metadata`: JSONB (Optional; for storing additional context, e.g., tool calls, agent names, RAG source documents, selected text for explanation)

## 2. AgentState

**Purpose**: Stores the persistent state of active agent conversations or workflows.

**Table Name**: `agent_state`

**Fields**:
- `id`: UUID (Primary Key)
- `session_id`: UUID (Foreign Key to ChatHistory.session_id; links agent state to a specific conversation)
- `agent_id`: TEXT (Identifier for the specific OpenAI Agent instance)
- `last_activity`: TIMESTAMP WITH TIME ZONE (Defaults to `NOW()`; records last interaction)
- `status`: ENUM ('active', 'paused', 'completed', 'failed')
- `current_step`: TEXT (Description of the agent's current task or step)
- `agent_memory`: JSONB (Stores the agent's internal memory, context, or persistent variables)
- `tool_outputs`: JSONB (Optional; stores results of tools called by the agent)
- `metadata`: JSONB (Optional; additional agent-specific context)

## 3. UserFeedback (Optional - for future enhancement)

**Purpose**: Captures user feedback on chatbot/agent responses to improve performance.

**Table Name**: `user_feedback`

**Fields**:
- `id`: UUID (Primary Key)
- `chat_history_id`: UUID (Foreign Key to ChatHistory.id; links feedback to a specific message)
- `user_id`: UUID (Foreign Key to User, if authentication is implemented)
- `timestamp`: TIMESTAMP WITH TIME ZONE (Defaults to `NOW()`)
- `rating`: INTEGER (e.g., 1-5 stars, or binary like/dislike)
- `comment`: TEXT (Optional; user's textual feedback)
- `feedback_type`: ENUM ('accuracy', 'relevance', 'completeness', 'agent_behavior', 'other')

## Relationships

- `ChatHistory` records can be linked to `AgentState` via `session_id` to provide a complete view of agent-driven conversations.
- `UserFeedback` (if implemented) would link to specific `ChatHistory` entries to provide granular feedback on responses.