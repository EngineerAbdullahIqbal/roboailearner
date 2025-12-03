import os
from pathlib import Path
from typing import List, Dict
import frontmatter

# robustly find the docs directory relative to this file
# backend/src/services/parser.py -> ../../../robotics_book_content/docs
BASE_DIR = Path(__file__).resolve().parent.parent.parent.parent
DOCS_DIR = BASE_DIR / "robotics_book_content" / "docs"

def parse_docs() -> List[Dict]:
    documents = []
    if not DOCS_DIR.exists():
        print(f"Warning: Docs dir {DOCS_DIR} not found")
        return []

    for file_path in DOCS_DIR.rglob("*.md*"): # .md and .mdx
        try:
            post = frontmatter.load(file_path)
            content = post.content
            metadata = post.metadata
            
            # Basic chunking by paragraphs or just whole file for now?
            # Spec says "Chunk size: 512 tokens". We need a chunker.
            # For MVP T010, let's just return the raw text and metadata.
            
            # Construct source URL (approximate)
            rel_path = file_path.relative_to(DOCS_DIR)
            slug = str(rel_path).replace(".mdx", "").replace(".md", "")
            url = f"/docs/{slug}"
            
            documents.append({
                "content": content,
                "metadata": {
                    "source": str(file_path),
                    "url": url,
                    "title": metadata.get("title", slug),
                    **metadata
                }
            })
        except Exception as e:
            print(f"Error parsing {file_path}: {e}")
            
    return documents
