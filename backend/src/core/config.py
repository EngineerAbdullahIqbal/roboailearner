from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional

class Settings(BaseSettings):
    PROJECT_NAME: str = "Physical AI RAG Chatbot"
    VERSION: str = "0.1.0"
    
    GOOGLE_API_KEY: str
    OPENAI_API_KEY: Optional[str] = None
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None
    DATABASE_URL: str
    BETTER_AUTH_SECRET: Optional[str] = None # Add this field
    
    model_config = SettingsConfigDict(env_file=".env", case_sensitive=True, extra='ignore') # Allow extra fields

settings = Settings()