---
id: "0008-fix-docusaurus-build-errors"
title: "Fix Docusaurus Build Errors: MDX Chars and Stale Sidebar References"
stage: "misc"
date: "2025-11-30"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "docusaurus-book"
branch: "main"
user: "user"
command: "fix-errors"
labels:
  - "docusaurus"
  - "mdx"
  - "debugging"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "tmp_docusaurus_project/docs/module-1/chapter-1.md"
  - "tmp_docusaurus_project/docs/module-1/chapter-3.md"
  - "tmp_docusaurus_project/sidebars.ts"
tests_yaml: []
prompt_text: |
  Compiled with problems:
  ×
  ERROR in ./docs/module-1/chapter-1.md
  ... Unexpected character `1` (U+0031) before name ...
  ERROR in ./docs/module-1/chapter-3.md
  ... Unexpected character `1` (U+0031) before name ...
  ERROR in ./.docusaurus/registry.js
  Module not found: Error: Can't resolve '@site/docs/module-4/chapter-8.md'
  Module not found: Error: Can't resolve '@site/docs/module-5/chapter-9.md'
response_text: |
  I have resolved the reported build errors:
  
  1.  **MDX Compilation Errors:**
      *   In `docs/module-1/chapter-1.md` and `docs/module-1/chapter-3.md`, the text `<100ms` was causing MDX to interpret `<1` as the start of an invalid HTML tag. I escaped the less-than sign to `&lt;100ms`.
  
  2.  **Module Not Found Errors:**
      *   The errors for `module-4/chapter-8.md` and `module-5/chapter-9.md` were due to stale references in `sidebars.ts` pointing to files I had previously deleted during the restructuring.
      *   I updated `sidebars.ts` to remove the outdated "Module 4" and "Module 5" categories.
      *   I ran `npm run clear` to wipe the Docusaurus cache and ensure the registry is rebuilt cleanly.
---
