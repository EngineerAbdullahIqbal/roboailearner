---
id: 0001
title: Sub-Agent Orchestration Workflow Review
stage: general
date: 2025-11-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-ai-book
user: {{USER}}
command: /agents
labels: ["workflow", "orchestration", "sub-agent"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - CLAUDE.md
tests:
  - N/A
---

## Prompt

# Sub-Agent Orchestration Workflow

## Phase 1: Content Planning
1. @content-architect → Generate book outline
2. @constitution-guardian → Validate against standards
3. @content-architect → Refine based on feedback

## Phase 2: Content Generation
1. @technical-writer → Generate chapter content
2. @simulation-expert → Add simulation tutorials
3. @hardware-integration → Add hardware setup guides
4. @constitution-guardian → Update with new standards

## Phase 3: RAG Integration
1. @rag-architect → Design vector database schema
2. @rag-architect → Implement embedding pipeline
3. @rag-architect → Create API endpoints
4. @constitution-guardian → Document RAG standards

## Parallel Workflows
- Constitution Guardian monitors all changes continuously
- Technical Writer can work on multiple chapters simultaneously
- Simulation Expert and Hardware Integration work on separate content streams

## Handoff Protocol
Each sub-agent must:
1. Read CLAUDE.md for current context
2. Check constitution.md for relevant standards
3. Update shared context after completing task
4. Notify dependent sub-agents of completion

## Response snapshot

No direct response yet, workflow analyzed.

## Outcome

- ✅ Impact: Understood the sub-agent orchestration workflow.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: Waiting for user's next task.
- 🧠 Reflection: User provided a high-level orchestration workflow for sub-agents involved in book creation. This outlines phases, parallel workflows, and handoff protocols.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
