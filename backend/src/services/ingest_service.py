from typing import List, Dict
from uuid import uuid4
from qdrant_client.http.models import PointStruct, VectorParams, Distance, CollectionStatus
from src.db.qdrant import async_client, client
from src.services.parser import parse_docs
from src.services.embeddings import get_embeddings
import tiktoken # for token counting

# This tokenizer is specific to OpenAI models but can be used for general token counting
# For Gemini, a specific tokenizer might be better, but tiktoken is a good proxy.
ENCODING = tiktoken.get_encoding("cl100k_base") # Matches GPT models

COLLECTION_NAME = "physical_ai_textbook"
VECTOR_SIZE = 768 # Gemini embedding-001

def num_tokens_from_string(string: str) -> int:
    """Returns the number of tokens in a text string."""
    return len(ENCODING.encode(string))

def chunk_text(text: str, chunk_size: int, chunk_overlap: int) -> List[str]:
    """Splits a text into smaller chunks based on token count."""
    tokens = ENCODING.encode(text)
    chunks = []
    
    i = 0
    while i < len(tokens):
        chunk = tokens[i : i + chunk_size]
        chunks.append(ENCODING.decode(chunk))
        i += chunk_size - chunk_overlap
        if i >= len(tokens) and len(chunk) < chunk_size: # Handle last chunk
            break
            
    return chunks

async def create_qdrant_collection():
    collections = await async_client.get_collections()
    if COLLECTION_NAME in [c.name for c in collections.collections]:
        await async_client.delete_collection(COLLECTION_NAME)
        print(f"Collection '{COLLECTION_NAME}' deleted for recreation.")
    
    await async_client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE),
    )
    print(f"Collection '{COLLECTION_NAME}' created.")

async def ingest_documents(chunk_size: int = 512, chunk_overlap: int = 50):
    await create_qdrant_collection()
    
    documents = parse_docs()
    points = []
    
    for doc in documents:
        content = doc["content"]
        metadata = doc["metadata"]
        
        # Split content into chunks
        chunks = chunk_text(content, chunk_size, chunk_overlap)
        
        # Generate embeddings for all chunks in this document
        chunk_embeddings = await get_embeddings(chunks)
        
        for i, chunk in enumerate(chunks):
            # Qdrant expects payload data to be JSON serializable
            # We need to extract chapter_id, chapter_title, section, page_number
            # from the metadata. The parser currently gives us 'source', 'url', 'title'.
            # Need to refine parser or infer these from existing metadata.
            # For now, let's map what we have.
            
            payload = {
                "chunk_id": i,
                "text": chunk,
                "source_url": metadata.get("url"),
                "chapter_title": metadata.get("title"),
                # Placeholder for chapter_id, section, page_number - needs refinement in parser
                "chapter_id": metadata.get("chapter_id", metadata.get("url", "").split('/')[2] if len(metadata.get("url", "").split('/')) > 2 else "unknown"),
                "section": f"chunk_{i}",
                "page_number": 0 # Needs to be extracted from metadata or inferred
            }
            
            points.append(
                PointStruct(
                    id=str(uuid4()),
                    vector=chunk_embeddings[i],
                    payload=payload
                )
            )
            
    # Upsert in batches to Qdrant
    if points:
        batch_size = 100
        for i in range(0, len(points), batch_size):
            await async_client.upsert(
                collection_name=COLLECTION_NAME,
                wait=True,
                points=points[i:i + batch_size]
            )
            print(f"Upserted {len(points[i:i + batch_size])} points to Qdrant.")
            
    return {"status": "success", "chunks_processed": len(points)}
