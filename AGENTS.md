# AGENTS.md

## Project

Anti-drone system simulation for a robotics course.

The current MVP consists of:
- a 2D workspace
- a planar 2-DOF robotic manipulator
- a simulated drone/target
- a simulated position sensor
- inverse kinematics
- joint-level PID control
- a closed-loop simulation
- visualization and basic validation

The architecture should allow future extension to additional
degrees of freedom and potentially a 3D workspace.

Do not implement future functionality unless explicitly requested.

## Architecture

The main architecture is documented in:

docs/architecture.md

Treat this document as the architectural source of truth.

Do not introduce architectural changes without explaining
the reason and impact.

## Code

- Use Python.
- Prefer simple, readable implementations.
- Use radians internally for angles.
- Avoid unnecessary abstractions.
- Do not introduce dependencies unless they are justified.
- Keep modules focused on a single responsibility.

## Testing

Non-trivial functionality must have tests.

Tests belong in:

tests/

Run the project's validation commands before considering
a task complete.

## Scope

Change the smallest possible surface area required to solve
the requested task.

Do not modify unrelated files.

Do not implement functionality that was not requested.

Do not refactor unrelated code unless explicitly requested.

## Git

Codex must not create commits unless explicitly instructed.

Keep changes logically separated.

The human developer reviews changes before committing.

## Workflow

For non-trivial tasks:

1. Inspect the repository.
2. Explain the proposed implementation plan.
3. Wait for approval if the task requires architectural decisions.
4. Implement the smallest reasonable change.
5. Add or update tests.
6. Run relevant checks.
7. Report what changed and what was verified.

## AI Decision Boundary

The human developer owns:
- project requirements
- architecture
- technology choices
- scope
- final code review

Codex assists with:
- implementation
- tests
- refactoring when requested
- documentation
- debugging
