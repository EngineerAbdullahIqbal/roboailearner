import asyncio
import os
from agents import Agent, Runner
from src.core.config import settings

# Mock API Key if needed for the script context (assuming .env is loaded or environment variables set)
# The user's environment has them.

async def main():
    print("Setting up agent...")
    agent = Agent(
        name="TestAgent",
        instructions="You are a helpful assistant.",
        model="gpt-3.5-turbo",
    )
    
    print(f"Running agent with model: {agent.model}")
    
    # Test run_streamed
    try:
        print("Calling Runner.run_streamed...")
        result = Runner.run_streamed(agent, input="Hello, say 'Working!'")
        print(f"Result object: {result}")
        
        print("Iterating stream...")
        async for event in result.stream_events():
            print(f"Event received: {type(event)} - {event}")
            
        print("Stream finished.")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Ensure env vars are present
    # We need to load them because we are running this script standalone
    from dotenv import load_dotenv
    load_dotenv("backend/.env") # Adjust path as needed
    
    # Check key
    if not os.getenv("OPENAI_API_KEY"):
        print("WARNING: OPENAI_API_KEY not found in env.")
    
    asyncio.run(main())
