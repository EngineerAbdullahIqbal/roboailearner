#!/usr/bin/env python3
import os
import json
import argparse
from pathlib import Path
from chunker import MDXChunker

def load_config(root_dir):
    """Load ignore patterns and schema"""
    ref_dir = root_dir / ".claude" / "skills" / "rag-knowledge-indexer" / "reference"
    
    ignore_path = ref_dir / "ignore_patterns.json"
    ignore_list = []
    if ignore_path.exists():
        with open(ignore_path, 'r') as f:
            ignore_list = json.load(f)
            
    return ignore_list

def scan_docs(docs_dir: Path, output_file: Path, chunker: MDXChunker):
    """Recursively scan docs and generate payload"""
    all_chunks = []
    
    print(f"🔍 Scanning directory: {docs_dir}")
    
    mdx_files = list(docs_dir.rglob("*.mdx")) + list(docs_dir.rglob("*.md"))
    
    for file_path in mdx_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                
            file_chunks = chunker.chunk_file(str(file_path.name), content)
            all_chunks.extend(file_chunks)
            print(f"  ✓ Processed {file_path.name} ({len(file_chunks)} chunks)")
            
        except Exception as e:
            print(f"  ❌ Error processing {file_path.name}: {str(e)}")

    # Ensure output directory exists
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(all_chunks, f, indent=2)
        
    print(f"\n✅ Indexing complete. Payload saved to: {output_file}")
    print(f"📊 Total Chunks: {len(all_chunks)}")

def main():
    parser = argparse.ArgumentParser(description="RAG Knowledge Indexer")
    parser.add_argument("--docs", default="docs", help="Path to docs directory")
    parser.add_argument("--out", default="rag_data/qdrant_payload.json", help="Output JSON path")
    args = parser.parse_args()

    project_root = Path(os.getcwd())
    docs_path = project_root / args.docs
    out_path = project_root / args.out
    
    ignore_list = load_config(project_root)
    chunker = MDXChunker(ignore_patterns=ignore_list)
    
    if not docs_path.exists():
        print(f"❌ Docs directory not found at: {docs_path}")
        return

    scan_docs(docs_path, out_path, chunker)

if __name__ == "__main__":
    main()