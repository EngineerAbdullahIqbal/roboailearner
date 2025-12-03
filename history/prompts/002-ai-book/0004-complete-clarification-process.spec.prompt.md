---
id: {{ID}}
title: {{TITLE}}
stage: {{STAGE}}
date: {{DATE_ISO}}
surface: {{SURFACE}}
model: {{MODEL}}
feature: {{FEATURE}}
branch: {{BRANCH}}
user: {{USER}}
command: {{COMMAND}}
labels: [{{LABELS}}]
links:
  spec: {{LINKS_SPEC}}
  ticket: {{LINKS_TICKET}}
  adr: {{LINKS_ADR}}
  pr: {{LINKS_PR}}
files:
{{FILES_YAML}}
tests:
{{TESTS_YAML}}
---

## Prompt

```text
Requirements

You are required to complete a unified book project using Claude Code and Spec-Kit Plus. The core deliverables are:

1. AI/Spec-Driven Book Creation: Write a book using Docusaurus and deploy it to GitHub Pages. You will use Spec-Kit Plus ( https://github.com/panaversity/spec-kit-plus/ ) and Claude Code ( https://www.claude.com/product/claude-code ) to write the book.

2. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user.

3. Participants will receive points out of 100, for base functionality defined above.

4. Participants can earn up to 50 extra bonus points by creating and using reusable intelligence via Claude Code Subagents and Agent Skills in the book project.

5. Participants can receive up to 50 extra bonus points if they also implement Signup and Signin using https://www.better-auth.com/ At signup you will ask questions from the user about their software and hardware background. Knowing the background of the user we will be able to personalize the content.

6.  Participants can receive up to 50 extra bonus points if the logged user can personalise the content in the chapters by pressing a button at the start of each chapter.

7. Participants can receive up to 50 extra bonus points if the logged user can translate the content in Urdu in the chapters by pressing a button at the start of each chapter.


The Course Details
Physical AI & Humanoid Robotics
Focus and Theme: AI Systems in the Physical World. Embodied Intelligence.
Goal: Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.
Quarter Overview
The future of AI extends beyond digital spaces into the physical world. This capstone quarter introduces Physical AI—AI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.
Module 1: The Robotic Nervous System (ROS 2)
Focus: Middleware for robot control.
ROS 2 Nodes, Topics, and Services.
Bridging Python Agents to ROS controllers using rclpy.
Understanding URDF (Unified Robot Description Format) for humanoids.


Module 2: The Digital Twin (Gazebo & Unity)
Focus: Physics simulation and environment building.
Simulating physics, gravity, and collisions in Gazebo.
High-fidelity rendering and human-robot interaction in Unity.
Simulating sensors: LiDAR, Depth Cameras, and IMUs.


Module 3: The AI-Robot Brain (NVIDIA Isaac™)
Focus: Advanced perception and training.
NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.
Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation.
Nav2: Path planning for bipedal humanoid movement.


Module 4: Vision-Language-Action (VLA)
Focus: The convergence of LLMs and Robotics.
Voice-to-Action: Using OpenAI Whisper for voice commands.
Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions.
Capstone Project: The Autonomous Humanoid. A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.
```

## Response snapshot

```text
No critical ambiguities detected worth formal clarification. All major requirements and details have been incorporated into the `spec.md`.
```

## Outcome

- ✅ Impact: The feature specification has been thoroughly updated with all user requirements, core deliverables, bonus points, and course details. No critical ambiguities remain.
- 🧪 Tests: N/A
- 📁 Files: specs/001-ai-book/spec.md
- 🔁 Next prompts: Proceed to the planning phase (`/sp.plan`).
- 🧠 Reflection: The `spec.md` is now comprehensive and ready for architectural planning.

## Evaluation notes (flywheel)

- Failure modes observed: Initial prerequisite check failed due to multiple spec directories matching the prefix. This was resolved by user selecting one feature to proceed with.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
