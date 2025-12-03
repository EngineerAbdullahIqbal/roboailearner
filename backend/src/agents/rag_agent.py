from typing import List, Dict, AsyncGenerator
import agents
from agents import Runner
from src.agents.client import get_openai_client
from src.agents.agent_setup import textbook_agent

async def chat_with_agent(
    messages: List[Dict],
    selection_context: str = None
) -> AsyncGenerator[str, None]:
    """
    Orchestrates the chat with the OpenAI Agent using the agents SDK.
    """
    # Configure Client
    client = get_openai_client()
    agents.set_default_openai_client(client)

    # Prepare Messages
    input_messages = list(messages)
    
    if selection_context:
        if input_messages and input_messages[-1].get("role") == "user":
            input_messages[-1]["content"] = str(input_messages[-1]["content"]) + f"\n\n[Context]\n{selection_context}"
        else:
            input_messages.append({"role": "user", "content": f"Context: {selection_context}"})

    # Run Agent
    # Runner.run_streamed returns a RunResultStreaming object
    print("Agent Runner started.")
    result = Runner.run_streamed(textbook_agent, input=input_messages)
    
    # Iterate over the stream
    async for event in result.stream_events():
        # print(f"Stream Event: {type(event)} - {event}") # Debug Log (Commented out for production)
        
        # Case: RawResponsesStreamEvent containing a ResponseTextDeltaEvent
        if type(event).__name__ == 'RawResponsesStreamEvent':
             data = getattr(event, 'data', None)
             if data and hasattr(data, 'type') and data.type == 'response.output_text.delta':
                 if hasattr(data, 'delta') and data.delta:
                     yield data.delta

        # Case: Standard OpenAI-like delta (fallback)
        elif hasattr(event, "delta") and event.delta:
            if hasattr(event.delta, "content") and event.delta.content:
                yield event.delta.content
        
        # Case: ModelResponse with content (fallback)
        elif hasattr(event, "content") and isinstance(event.content, str):
            yield event.content


