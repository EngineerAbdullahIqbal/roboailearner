import asyncio
import os
from sqlalchemy import text
from src.db.neon import engine

async def init_db():
    print("Initializing database...")
    try:
        async with engine.begin() as conn:
            with open("src/db/schema.sql", "r") as f:
                schema = f.read()
            
            # Split by statement to execute one by one if needed, 
            # but execute() can often handle blocks depending on driver. 
            # safest is usually to just execute the whole block or split by ;
            # For asyncpg/sqlalchemy, executing the whole block usually works for DDL.
            
            statements = [s.strip() for s in schema.split(';') if s.strip()]
            
            for statement in statements:
                await conn.execute(text(statement))
                
        print("Database initialized successfully.")
    except Exception as e:
        print(f"Error initializing database: {e}")
    finally:
        await engine.dispose()

if __name__ == "__main__":
    # Need to make sure we can import src (run from backend root)
    import sys
    sys.path.append(os.getcwd())
    asyncio.run(init_db())
