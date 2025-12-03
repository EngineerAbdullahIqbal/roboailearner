# ADR-0005: Authentication Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-11-30
- **Feature:** 003-better-auth-authentication
- **Context:** The project requires a robust and secure authentication system for user signup, signin, and protected routes. The system needs to collect additional user background information during signup. Integration with a React frontend and Neon Postgres backend is required.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

*   Authentication Service: `better-auth.com`
*   Frontend Integration: `React` with `AuthProvider` component, dedicated `signup.tsx` and `signin.tsx` pages.
*   Backend Integration: Node.js, storing user profile data in `Neon Postgres`.

## Consequences

### Positive

*   Leverages a specialized third-party service for authentication, reducing development effort and improving security posture (handling password hashing, sessions, etc.).
*   Provides a structured approach for frontend integration.

### Negative

*   Introduction of a third-party dependency (`better-auth.com`) which might lead to vendor lock-in, potential cost implications, and dependency on their service uptime and features.
*   Requires careful management of custom user data collection.

## Alternatives Considered

*   **Alternative 1 (Self-hosted Auth)**: Implement authentication logic from scratch using a library like Passport.js (Node.js) or integrate with a self-hosted solution like Keycloak.
    *   **Why rejected**: Higher development and maintenance overhead, increased responsibility for security best practices.
*   **Alternative 2 (Other Identity Providers)**: Use other managed identity providers like Auth0, AWS Cognito, Google Firebase Authentication.
    *   **Why rejected**: `better-auth.com` was specified in the initial prompt, implying a preference or existing integration. Further evaluation needed if `better-auth.com` proves insufficient.

## References

- Feature Spec: `specs/003-better-auth-authentication/spec.md`
- Implementation Plan: `specs/003-better-auth-authentication/plan.md`
- Quickstart Guide: `specs/003-better-auth-authentication/quickstart.md`
- Auth API Contract: `specs/003-better-auth-authentication/contracts/auth.yaml`
- Related ADRs: null
- Evaluator Evidence: null