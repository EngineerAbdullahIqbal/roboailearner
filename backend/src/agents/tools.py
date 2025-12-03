from src.db.qdrant import async_client
from src.services.embeddings import get_embeddings
from src.core.config import settings

# Tool definition for OpenAI Agents SDK (or just a function for the agent to call)
# The agent will call this function to retrieve context.

import traceback

async def retrieve_context(query: str, limit: int = 3) -> str:
    """
    Retrieves relevant context from the Physical AI & Humanoid Robotics textbook.
    
    Use this tool to find information about ROS 2, simulation, hardware, or specific chapters.
    
    Args:
        query (str): The search query to find relevant textbook sections.
        limit (int): The max number of sections to return. Defaults to 3.
    
    Returns:
        str: A formatted string containing the relevant text chunks and their sources.
    """
    print(f"Tool Call: retrieve_context(query='{query}', limit={limit})") # Log tool call
    try:
        # Generate embedding for the query (Now Async!)
        embeddings = await get_embeddings([query])
        query_vector = embeddings[0]
        
        # Search Qdrant using the modern 'query_points' API for client v1.10+
        search_result = await async_client.query_points(
            collection_name="physical_ai_textbook",
            query=query_vector,
            limit=limit,
            with_payload=True
        )
        
        # Format the result
        # query_points returns a QueryResponse object which contains a list of ScoredPoint in .points
        context_parts = []
        for point in search_result.points:
            payload = point.payload
            text = payload.get("text", "")
            title = payload.get("chapter_title", "Unknown Chapter")
            url = payload.get("source_url", "")
            
            context_parts.append(f"Source: {title} ({url})\nContent: {text}")
        
        formatted_context = "\n\n---\n\n".join(context_parts)
        print(f"Tool Output: Retrieved {len(context_parts)} contexts. First context excerpt: {formatted_context[:200]}...") # Log output
        return formatted_context
        
    except Exception as e:
        print(f"Retrieval error: {e}")
        traceback.print_exc()
        return "Error retrieving context."

# OpenAI Tool Definition Schema
retrieve_tool_definition = {
    "type": "function",
    "function": {
        "name": "retrieve_context",
        "description": "Retrieves relevant information from the Physical AI and Robotics textbook. Use this whenever you need to answer questions about robotics, ROS 2, simulation, or physical AI concepts.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query to find relevant textbook sections."
                }
            },
            "required": ["query"]
        }
    }
}