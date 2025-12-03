from openai import AsyncOpenAI
from src.core.config import settings

# Global Singleton
_openai_client_instance = None

def get_openai_client() -> AsyncOpenAI:
    global _openai_client_instance
    if _openai_client_instance:
        return _openai_client_instance

    if settings.OPENAI_API_KEY:
        # Use OpenAI's API
        print("Initializing new OpenAI API client (Singleton).")
        _openai_client_instance = AsyncOpenAI(
            api_key=settings.OPENAI_API_KEY
        )
    else:
        # Fallback to Gemini's OpenAI-compatible API
        print("Initializing new Gemini's OpenAI-compatible API client (Singleton).")
        _openai_client_instance = AsyncOpenAI(
            api_key=settings.GOOGLE_API_KEY,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
    
    return _openai_client_instance