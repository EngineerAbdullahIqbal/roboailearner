# ADR-0006: User Data Management and Storage

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-11-30
- **Feature:** 003-better-auth-authentication
- **Context:** User profile data, including email, password hash, and background questions (software experience, hardware experience, learning goals), needs to be persistently stored. The chosen authentication system (`better-auth.com`) integrates with a database. Data integrity, scalability, and relationships are important for future features.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

*   Primary Storage: `Neon Postgres`
*   Schema: `User` table as defined in `data-model.md`, including fields for `id`, `email`, `password_hash`, `name`, `software_level`, `hardware_level`, `learning_goals`, `created_at`, `updated_at`.

## Consequences

### Positive

*   `Neon Postgres` offers a managed, scalable, and reliable relational database solution.
*   Strong ACID compliance and support for complex queries and relationships.
*   Well-suited for structured user data.

### Negative

*   Relational database might require schema migrations for evolving data structures.
*   Potential for increased operational complexity if not fully managed.

## Alternatives Considered

*   **Alternative 1 (Other Relational Databases)**: Use other managed PostgreSQL services (e.g., AWS RDS, Azure Database for PostgreSQL) or other SQL databases (e.g., MySQL).
    *   **Why rejected**: `Neon Postgres` was specified in the initial prompt, implying a preference or existing infrastructure. Functionally similar alternatives would not offer significant benefits at this stage.
*   **Alternative 2 (NoSQL Database)**: Use a NoSQL database like MongoDB or DynamoDB.
    *   **Why rejected**: While flexible for evolving schemas, the current user data structure is well-defined and benefits from the relational model. No immediate need for schema-less storage.

## References

- Feature Spec: `specs/003-better-auth-authentication/spec.md`
- Implementation Plan: `specs/003-better-auth-authentication/plan.md`
- Data Model: `specs/003-better-auth-authentication/data-model.md`
- User API Contract: `specs/003-better-auth-authentication/contracts/user.yaml`
- Related ADRs: null
- Evaluator Evidence: null