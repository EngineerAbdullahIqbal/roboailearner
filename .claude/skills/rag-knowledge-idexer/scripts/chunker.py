#!/usr/bin/env python3
import re
import uuid
import hashlib
from typing import List, Dict, Optional

class MDXChunker:
    def __init__(self, ignore_patterns: List[str]):
        self.ignore_patterns = [p.lower() for p in ignore_patterns]
        # Regex to capture Frontmatter (--- ... ---)
        self.frontmatter_pattern = re.compile(r'^---\s*\n(.*?)\n---\s*\n', re.DOTALL)
        # Regex to find Headers (## Title)
        self.header_pattern = re.compile(r'(^#{2,4}\s+.+$)', re.MULTILINE)

    def parse_frontmatter(self, content: str) -> Dict:
        """Extract metadata from Docusaurus frontmatter"""
        match = self.frontmatter_pattern.match(content)
        metadata = {}
        if match:
            fm_text = match.group(1)
            # Simple parsing to avoid PyYAML dependency if not present
            for line in fm_text.split('\n'):
                if ':' in line:
                    key, val = line.split(':', 1)
                    metadata[key.strip()] = val.strip().strip('"').strip("'")
        return metadata

    def clean_mdx(self, text: str) -> str:
        """Remove MDX imports and components that confuse the RAG"""
        # Remove import statements
        text = re.sub(r'^import\s+.*;', '', text, flags=re.MULTILINE)
        # Remove opening/closing tags for Tabs, Admonitions (keep content)
        text = re.sub(r'<Tabs>|<TabItem.*?>|</Tabs>|</TabItem>', '', text)
        text = re.sub(r':::', '', text) 
        return text.strip()

    def chunk_file(self, file_path: str, content: str) -> List[Dict]:
        """Split file content into semantic chunks"""
        metadata = self.parse_frontmatter(content)
        
        # Remove frontmatter from content for chunking
        body_content = self.frontmatter_pattern.sub('', content)
        body_content = self.clean_mdx(body_content)

        # Split by headers
        # This approach keeps the header with the following text
        sections = self.header_pattern.split(body_content)
        
        chunks = []
        current_header = "Introduction"
        
        # If text starts before first header
        if sections and not sections[0].startswith('#'):
            first_chunk = sections.pop(0).strip()
            if first_chunk:
                chunks.append(self._create_payload(first_chunk, current_header, file_path, metadata))

        # Iterate through header/content pairs
        for i in range(0, len(sections) - 1, 2):
            header = sections[i].strip().replace('#', '').strip()
            text = sections[i+1].strip()
            
            # Skip ignored sections
            if any(ign in header.lower() for ign in self.ignore_patterns):
                continue
                
            if text:
                chunks.append(self._create_payload(text, header, file_path, metadata))
                
        return chunks

    def _create_payload(self, text: str, section: str, source: str, meta: Dict) -> Dict:
        """Format the chunk according to Qdrant payload schema"""
        # Create a deterministic ID based on content to avoid duplicates on re-index
        chunk_hash = hashlib.md5((source + section + text[:20]).encode()).hexdigest()
        
        return {
            "id": str(uuid.UUID(hex=chunk_hash)),
            "content": f"## {section}\n\n{text}",
            "metadata": {
                "source_file": str(source),
                "section_header": section,
                "book_title": meta.get('title', 'Unknown'),
                "week": meta.get('week', 'general'),
                "difficulty": meta.get('difficulty', 'intermediate'),
                "topics": meta.get('keywords', '')
            }
        }