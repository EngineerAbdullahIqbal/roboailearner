# Implementation Plan: 3D Interactive Diagrams

**Branch**: `004-3d-diagrams` | **Date**: 2025-12-03 | **Spec**: [link](./spec.md)
**Input**: Feature specification from `/specs/004-3d-diagrams/spec.md`

## Summary

Replace all static Mermaid diagrams in the Robotics + AI book with interactive 3D diagrams using `three.js`. This involves creating a reusable React component that loads 3D scenes from a JSON manifest, implementing generic "builders" for flowcharts and robot visualizations, and migrating existing markdown content.

## Technical Context

**Language/Version**: TypeScript 5+, React 18+ (Docusaurus environment)
**Primary Dependencies**: `three`, `@types/three` (optionally `@react-three/fiber` if permitted, but sticking to vanilla wrapper per prompt "HTML+JS scaffold").
**Storage**: Static JSON manifest (`manifest.json`) and GLB assets.
**Target Platform**: Web (Desktop/Mobile).
**Performance Goals**: 60fps for simple graphs, smooth LOD for complex models.
**Constraints**: Must work within Docusaurus build system. Accessible fallback required.

## Constitution Check

*   **Complexity**: Low-Poly, reusable builders minimize code complexity.
*   **Dependencies**: Standard `three.js` is robust and stable.
*   **Performance**: Using `InstancedMesh` or merged geometries for node graphs if density is high (unlikely for this book).

## Project Structure

### Documentation (this feature)

```text
specs/004-3d-diagrams/
├── plan.md              # This file
├── research.md          # Diagram inventory and tech strategy
├── data-model.md        # Manifest schema
├── quickstart.md        # Usage guide for authors
└── tasks.md             # Task breakdown
```

### Source Code

```text
robotics_book_content/
├── static/
│   └── 3d/
│       ├── manifest.json       # The "Database" of diagrams
│       └── assets/             # .glb models, textures
└── src/
    └── components/
        └── ThreeDiagram/       # New Component
            ├── index.tsx
            ├── SceneLoader.ts
            ├── Engine.ts
            └── Builders/
                ├── FlowBuilder.ts
                └── RobotBuilder.ts
```

## Phases & Tasks

### Phase 1: Scaffold & Infrastructure
1.  **Dependencies**: Install `three` and `@types/three`.
2.  **Component**: Create `src/components/ThreeDiagram/index.tsx`.
    *   Accepts `id` prop.
    *   Loads `manifest.json`.
    *   Initializes `Three.js` scene.
3.  **Manifest**: Create `static/3d/manifest.json` with the 12+ diagrams identified in `research.md`.

### Phase 2: The Generic Builders
1.  **FlowBuilder**: Implement logic to turn `nodes` and `edges` from JSON into 3D objects.
    *   Nodes: Rounded Box Geometry.
    *   Edges: TubeGeometry or LineSegments.
    *   Labels: HTML overlays.
2.  **Camera Controls**: Implement `OrbitControls` with constraints (min/max zoom).

### Phase 3: Specific Visualizations
1.  **RobotBuilder**: Integration for loading `.glb` models (e.g., Unitree Go2) and placing annotations on specific bones/positions.
2.  **Point Cloud**: Simple particle system generator for Chapter 5.

### Phase 4: Content Migration
1.  **Remove Mermaid**: Script or manual edit to delete all ` ```mermaid ` blocks.
2.  **Insert Component**: Replace with `<ThreeDiagram id="chapter-1-node-graph" />`.
3.  **Verification**: Build Docusaurus and check for errors.

## Future Work
*   VR/AR support (WebXR).
*   Code-driven animation (highlight path when code block is hovered).