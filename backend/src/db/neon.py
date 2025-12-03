from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker, AsyncSession
from sqlalchemy.orm import DeclarativeBase
from src.core.config import settings
import ssl
from urllib.parse import urlparse, parse_qs, urlunparse

# Ensure we use postgresql+asyncpg schema
DATABASE_URL = settings.DATABASE_URL
if DATABASE_URL and DATABASE_URL.startswith("postgresql://"):
    DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)

# Clean parameters from URL to avoid conflict with asyncpg connect_args
# Specifically stripping 'sslmode' which causes "unexpected keyword argument"
url_parts = urlparse(DATABASE_URL)
# Reconstruct URL without query parameters
# We keep the scheme, netloc, and path. We drop params, query, and fragment for the engine URL
# because we are passing SSL context manually.
CLEAN_DATABASE_URL = urlunparse(url_parts._replace(query=""))

# Create a default SSL context for Neon
ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

# Pass SSL context to asyncpg
connect_args = {"ssl": ctx}

# Add pool_pre_ping=True to prevent "connection is closed" errors
engine = create_async_engine(
    CLEAN_DATABASE_URL,
    echo=False,
    connect_args=connect_args,
    pool_pre_ping=True, # Verify connection before using
    pool_recycle=300    # Recycle connections every 5 minutes
)

AsyncSessionLocal = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

class Base(DeclarativeBase):
    pass

async def get_db():
    async with AsyncSessionLocal() as session:
        yield session