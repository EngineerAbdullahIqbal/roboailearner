---
id: 0002
title: Update Constitution for Hackathon Mode
stage: constitution
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-book
branch: 001-ai-book
user: abdullahiqbal
command: "start the Physical AI & Humanoid Robotics textbook hackathon. Execute this sequence: CONTEXT: - Book Title: \"Physical AI & Humanoid Robotics: From Simulation to Reality\" - Course Syllabus: Located in \"Hackathon_ Physical AI & Humanoid Robotics Textbook.md\" - Target: 13 comprehensive chapters - Deadline: 12 hours from now STEP 1: Update Constitution for Hackathon Mode Add to .specify/memory/constitution.md: ## Hackathon Project: Physical AI & Humanoid Robotics Textbook ### Project Overview - **Book Title:** Physical AI & Humanoid Robotics: From Simulation to Reality - **Total Chapters:** 13 chapters covering 13-week robotics curriculum - **Word Count Target:** 1,800-2,500 words per chapter - **Technical Depth:** Intermediate to Advanced - **Technologies:** ROS 2, Gazebo, NVIDIA Isaac Sim, OpenAI GPT, Unitree robots ### Content Generation Standards 1. **Each chapter MUST include:** - 5 specific learning objectives - Introduction with real-world context (200 words) - Core concepts with theory (600-800 words) - Practical code examples (2-3 working examples) - Hands-on lab exercise (400-500 words) - Hardware recommendations - Safety considerations (where applicable) - Summary with key takeaways - 5 review questions - Further reading (3-5 resources) 2. **Code Quality:** - All code examples must be syntactically correct - Include full imports and main() functions - Add inline comments explaining logic - Show expected output - Use Python 3.11+ with ROS 2 Humble 3. **Technical Accuracy:** - Use Context7 MCP to verify latest ROS 2 documentation - Cite NVIDIA Isaac Sim 2023.1+ features - Reference Unitree G1/Go2 specifications accurately - Include proper hardware requirements 4. **Accessibility & UX:** - Use Docusaurus admonitions (:::tip, :::warning, :::note) - Include Mermaid diagrams for architecture - Proper heading hierarchy (H1 → H2 → H3) - Alt text for all images - Mobile-responsive design 5. **Auto-Update Protocol:** After EVERY chapter generation, automatically append to constitution: ## [TIMESTAMP] Chapter [N] Generated - Title: [chapter_title] - Word count: [count] - Code examples: [count] - Diagrams: [count] - Lab included: Yes/No - Generation time: [minutes] - Quality check: ✅ Passed"
labels: ["constitution","hackathon"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
null
---

## Prompt

"start the Physical AI & Humanoid Robotics textbook hackathon. Execute this sequence:

CONTEXT:
- Book Title: "Physical AI & Humanoid Robotics: From Simulation to Reality"
- Course Syllabus: Located in "Hackathon_ Physical AI & Humanoid Robotics Textbook.md"
- Target: 13 comprehensive chapters
- Deadline: 12 hours from now

STEP 1: Update Constitution for Hackathon Mode
Add to .specify/memory/constitution.md:

## Hackathon Project: Physical AI & Humanoid Robotics Textbook

### Project Overview
- **Book Title:** Physical AI & Humanoid Robotics: From Simulation to Reality
- **Total Chapters:** 13 chapters covering 13-week robotics curriculum
- **Word Count Target:** 1,800-2,500 words per chapter
- **Technical Depth:** Intermediate to Advanced
- **Technologies:** ROS 2, Gazebo, NVIDIA Isaac Sim, OpenAI GPT, Unitree robots

### Content Generation Standards
1. **Each chapter MUST include:**
   - 5 specific learning objectives
   - Introduction with real-world context (200 words)
   - Core concepts with theory (600-800 words)
   - Practical code examples (2-3 working examples)
   - Hands-on lab exercise (400-500 words)
   - Hardware recommendations
   - Safety considerations (where applicable)
   - Summary with key takeaways
   - 5 review questions
   - Further reading (3-5 resources)

2. **Code Quality:**
   - All code examples must be syntactically correct
   - Include full imports and main() functions
   - Add inline comments explaining logic
   - Show expected output
   - Use Python 3.11+ with ROS 2 Humble

3. **Technical Accuracy:**
   - Use Context7 MCP to verify latest ROS 2 documentation
   - Cite NVIDIA Isaac Sim 2023.1+ features
   - Reference Unitree G1/Go2 specifications accurately
   - Include proper hardware requirements

4. **Accessibility & UX:**
   - Use Docusaurus admonitions (:::tip, :::warning, :::note)
   - Include Mermaid diagrams for architecture
   - Proper heading hierarchy (H1 → H2 → H3)
   - Alt text for all images
   - Mobile-responsive design

5. **Auto-Update Protocol:**
   After EVERY chapter generation, automatically append to constitution:

   ## [TIMESTAMP] Chapter [N] Generated
   - Title: [chapter_title]
   - Word count: [count]
   - Code examples: [count]
   - Diagrams: [count]
   - Lab included: Yes/No
   - Generation time: [minutes]
   - Quality check: ✅ Passed"

## Response snapshot

The `.specify/memory/constitution.md` file has been updated with the hackathon project constitution and content generation standards.

## Outcome

- ✅ Impact: Constitution updated with hackathon rules.
- 🧪 Tests: No automated tests for constitution updates.
- 📁 Files: Modified .specify/memory/constitution.md.
- 🔁 Next prompts: Next step is to start content generation based on the updated constitution.
- 🧠 Reflection: Successfully updated the constitution file as per user's request.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): None
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None
