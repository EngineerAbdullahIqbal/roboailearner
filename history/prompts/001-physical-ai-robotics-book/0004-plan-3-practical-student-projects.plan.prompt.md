---
id: "0004-plan-3-practical-student-projects"
title: "Plan 3 Practical Student Projects"
stage: "plan"
date: "2025-11-30"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "001-physical-ai-robotics-book"
branch: "001-physical-ai-robotics-book"
user: "user"
command: "/sp.plan"
labels:
  - "planning"
  - "robotics"
  - "education"
links:
  spec: "specs/001-physical-ai-robotics-book/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/001-physical-ai-robotics-book/plan.md"
  - "specs/001-physical-ai-robotics-book/research.md"
  - "specs/001-physical-ai-robotics-book/data-model.md"
  - "specs/001-physical-ai-robotics-book/quickstart.md"
  - "specs/001-physical-ai-robotics-book/contracts/SorterControl.srv"
tests_yaml: []
prompt_text: |
  /sp.plan analyze all the specs and chapters then plan 3 proejct After reading this book for students that they can make with practicle
response_text: |
  I have successfully planned 3 practical projects for students of the "Physical AI & Humanoid Robotics" book.
  
  **Projects Planned:**
  1.  **Sentient Sentry (Beginner):** A smart security camera using OpenCV for face tracking and simple motor control.
  2.  **Visual Sorter Arm (Intermediate):** A robotic arm application using MoveIt 2 and Inverse Kinematics to sort objects by color.
  3.  **Office Runner (Advanced):** An autonomous delivery robot utilizing the ROS 2 Navigation Stack (Nav2) and SLAM.
  
  **Artifacts Generated:**
  -   `plan.md`: The master implementation plan.
  -   `research.md`: Detailed project descriptions and hardware/simulation requirements.
  -   `data-model.md`: ROS 2 package structures and node graphs for each project.
  -   `quickstart.md`: Setup and launch instructions.
  -   `contracts/SorterControl.srv`: Example service definition for the intermediate project.
---