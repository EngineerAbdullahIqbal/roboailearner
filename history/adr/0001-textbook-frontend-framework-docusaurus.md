# ADR-0001: Textbook Frontend Framework: Docusaurus

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

-   **Status:** Proposed
-   **Date:** 2025-11-30
-   **Feature:** 001-physical-ai-robotics-book
-   **Context:** The project requires a comprehensive 13-chapter textbook with hands-on labs, code examples, and multimedia support. The chosen framework needs to handle Markdown/MDX content, provide robust search, responsive design, and easy deployment to GitHub Pages.

## Decision

Docusaurus 3.9.0 is selected as the frontend framework.
    -   Framework: Docusaurus 3.9.0
    -   Content Format: Markdown/MDX
    -   Custom UI: React components
    -   Configuration: `docusaurus.config.js`, `sidebars.js`
    -   Deployment: GitHub Pages

## Consequences

### Positive

Excellent for technical documentation, strong Markdown/MDX support, good extensibility with React, responsive design out-of-the-box, easy deployment to GitHub Pages, built-in search functionality.

### Negative

May have a learning curve for those unfamiliar with React/Docusaurus, customization beyond standard features might require deeper understanding of its architecture.

## Alternatives Considered

-   **Next.js/Gatsby**: More flexible React frameworks but might require more setup for documentation-specific features and less opinionated structure.
-   **Custom HTML/CSS/JS**: Highest flexibility but also highest development effort for features like search, routing, and responsive design.

## References

-   Feature Spec: specs/001-physical-ai-robotics-book/spec.md
-   Implementation Plan: specs/001-physical-ai-robotics-book/plan.md
-   Related ADRs: null
-   Evaluator Evidence: null
