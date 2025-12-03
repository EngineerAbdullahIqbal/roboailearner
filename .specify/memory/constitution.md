<!-- Sync Impact Report
- Version change: 0.0.0 -> 1.0.0
- List of modified principles: Initial ratification
- Added sections: All
- Removed sections: None
- Templates requiring updates: None (Initial setup)
- Follow-up TODOs: None
-->

# Physical AI & Robotics Book Constitution

## Core Principles

### I. Spec-Driven Development
Development MUST NOT proceed without a clear specification and plan. Business requirements (Why/What) must be separated from technical implementation (How). All features require a `spec.md` and `plan.md` before task generation to ensure alignment with user intent.

### II. Traceability & Documentation
Every significant user interaction and architectural decision MUST be recorded. Prompt History Records (PHRs) track intent and execution. Architectural Decision Records (ADRs) capture significant technical choices and trade-offs to preserve context for future contributors.

### III. Testability & Verification
All changes MUST be verifiable. Code should be written to be testable. Changes should be small, atomic, and include appropriate tests or verification steps where applicable. "Red-Green-Refactor" is the preferred workflow for code changes to ensure stability.

### IV. Minimalism & Focus
Prefer the smallest viable change to achieve the goal. Do not refactor unrelated code during a feature implementation ("YAGNI"). Focus on the "Smallest Viable Diff" to reduce risk and review complexity.

### V. Explicit Contracts
Interfaces, APIs, and Data Models MUST be defined explicitly before implementation. Do not assume internal knowledge or invent contracts on the fly; clarify and document them first to prevent integration issues.

### VI. Security First
Secrets and tokens MUST NEVER be hardcoded. Use environment variables (`.env`) and documentation. Security implications should be considered at the planning stage to prevent vulnerabilities.

## Operational Standards

### Technology Stack
- **Primary Language**: Python 3.11+
- **Robotics Framework**: ROS 2 Humble
- **Simulation**: Gazebo / NVIDIA Isaac Sim
- **Documentation/Book**: Docusaurus 3.x (React/TypeScript)
- **Authentication**: Better-Auth
- **Database**: Neon Postgres / Qdrant

### Code Quality
- Follow idiomatic patterns for the respective language (Pythonic for Python, Hooks for React).
- Ensure code is readable and maintainable.
- Use descriptive naming conventions.

## Development Workflow

### Planning
1.  **Spec**: Define user stories and requirements.
2.  **Plan**: Design architecture and identify technical tasks.
3.  **Review**: Validate plan against Constitution and constraints.

### Execution
1.  **Tasks**: Break down plan into small, testable tasks.
2.  **Implement**: Execute tasks iteratively.
3.  **Verify**: Run tests/checks to ensure success criteria are met.

## Governance

This Constitution supersedes all other project practices.

### Amendments
- Any change to these principles requires a Constitution Amendment (Version Bump).
- Amendments must be documented with rationale.
- Significant changes may require an ADR.

### Compliance
- All Plans and Pull Requests MUST verify compliance with these principles.
- Non-compliant changes should be rejected or reworked.

**Version**: 1.0.0 | **Ratified**: 2025-12-01 | **Last Amended**: 2025-12-01