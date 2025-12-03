# ADR-0003: Simulation Platforms: Gazebo and NVIDIA Isaac Sim

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-11-30
-   **Feature:** 001-physical-ai-robotics-book
-   **Context:** The textbook aims to teach robot development, which heavily relies on simulation for safe and efficient testing. Two primary simulation platforms, Gazebo and NVIDIA Isaac Sim, are identified as industry-relevant tools, offering different strengths for various simulation needs.

## Decision

Gazebo and NVIDIA Isaac Sim 2023.1+ are selected as the primary simulation platforms.
    -   General-purpose Simulation: Gazebo
    -   High-fidelity/AI Simulation: NVIDIA Isaac Sim 2023.1+

## Consequences

### Positive

Provides comprehensive coverage of widely used simulation tools, exposing students to both open-source (Gazebo) and commercial/AI-focused (Isaac Sim) environments. Gazebo offers a robust, flexible environment for general robotics, while Isaac Sim provides high-fidelity physics, synthetic data generation capabilities, and deep integration with NVIDIA's AI ecosystem (Isaac ROS). This dual approach caters to different levels of simulation complexity and AI integration needs within robotics.

### Negative

Requires students to learn and configure two distinct simulation environments, which might increase the initial learning curve. NVIDIA Isaac Sim has higher hardware requirements, specifically needing a compatible NVIDIA GPU, which could be a barrier for some students.

## Alternatives Considered

-   **Unity**: A powerful game engine with robotics extensions (e.g., Unity Robotics Hub), offering high-fidelity rendering and extensive customization. However, its integration for purely robotics-focused tasks might be more complex than Isaac Sim, which is purpose-built for robotics.
-   **Webots**: Another open-source robotics simulator known for its user-friendliness and comprehensive robot models. While a viable option, it is generally less prevalent in the broader robotics community compared to Gazebo and Isaac Sim, potentially limiting student exposure to industry-standard tools.
-   **Custom physics engines**: Developing a custom physics engine or simulation environment requires significant development effort, deep expertise in physics modeling, and extensive validation, which is beyond the scope of a textbook and highly impractical.

## References

-   Feature Spec: specs/001-physical-ai-robotics-book/spec.md
-   Implementation Plan: specs/001-physical-ai-robotics-book/plan.md
-   Related ADRs: null
-   Evaluator Evidence: null
