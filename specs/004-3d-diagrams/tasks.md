# Tasks: 3D Interactive Diagrams

**Branch**: `004-3d-diagrams` | **Status**: Pending
**Spec**: [specs/004-3d-diagrams/spec.md](./spec.md)

## Phase 1: Setup
*Goal: Initialize the environment and dependencies.*

- [x] T001 Install `three` and `@types/three` dependencies in `robotics_book_content` package.json
- [x] T002 Create directory structure `src/components/ThreeDiagram/Builders` and `static/3d/assets`

## Phase 2: Foundational
*Goal: Establish the core Three.js engine and React wrapper.*

- [x] T003 Create `src/components/ThreeDiagram/Engine.ts` to handle WebGLRenderer, Scene, and Camera initialization
- [x] T004 Create `src/components/ThreeDiagram/SceneLoader.ts` with TypeScript interfaces for the Manifest schema
- [x] T005 Create `src/components/ThreeDiagram/index.tsx` as the main React component wrapping the Engine
- [x] T006 Create initial `static/3d/manifest.json` with a single test scene configuration

## Phase 3: Generic Builders (Flow Graphs)
*Goal: Implement the builder capable of rendering node-based architecture diagrams.*

- [x] T007 [US1] Implement `src/components/ThreeDiagram/Builders/FlowBuilder.ts` to render nodes (BoxGeometry) from config
- [x] T008 [US1] Add edge rendering (TubeGeometry or LineSegments) to `FlowBuilder.ts` connecting nodes
- [x] T009 [US1] Implement HTML label overlay system in `ThreeDiagram/index.tsx` synced with 3D positions
- [x] T010 [US1] Add `OrbitControls` to `Engine.ts` for user interaction (zoom/pan/rotate)
- [ ] T011 [P] [US1] Create "Test Flow" scene in `manifest.json` to verify FlowBuilder

## Phase 4: Specific Visualizations
*Goal: Implement specialized builders for Robot models and Point Clouds.*

- [x] T012 [US2] Implement `src/components/ThreeDiagram/Builders/RobotBuilder.ts` using GLTFLoader
- [ ] T013 [P] [US2] Add a placeholder robot GLB to `static/3d/assets/` (e.g., a simple primitive composition if actual model unavailable)
- [x] T014 [US2] Implement annotation system in `RobotBuilder.ts` to highlight specific bones/links
- [x] T015 [US2] Implement `src/components/ThreeDiagram/Builders/PointCloudBuilder.ts` using THREE.Points
- [x] T016 [P] [US2] Create "Test Robot" and "Test PointCloud" scenes in `manifest.json`

## Phase 5: Content Migration
*Goal: Replace all existing Mermaid diagrams with the new 3D component.*

- [x] T017 [US3] Populate `static/3d/manifest.json` with configurations for all 12+ diagrams identified in research.md
- [x] T018 [US3] Update `robotics_book_content/docs/module-1/chapter-1.md` replacing ROS Node Graph with `<ThreeDiagram id="1.1" />`
- [x] T019 [US3] Update `robotics_book_content/docs/module-1/chapter-2.md` replacing Topic/Service graphs with `<ThreeDiagram />`
- [x] T020 [US3] Update `robotics_book_content/docs/module-1/chapter-3.md` replacing Gazebo Arch with `<ThreeDiagram />`
- [x] T021 [US3] Update `robotics_book_content/docs/module-2/chapter-4.md` replacing Image Pipeline with `<ThreeDiagram />`
- [x] T022 [US3] Update `robotics_book_content/docs/module-2/chapter-5.md` replacing Point Cloud Pipeline with `<ThreeDiagram />`
- [x] T023 [US3] Update `robotics_book_content/docs/module-2/chapter-6.md` replacing Sensor Fusion/TF2 with `<ThreeDiagram />`
- [x] T024 [US3] Update `robotics_book_content/docs/module-3/chapter-7.md`, `chapter-8.md`, `chapter-9.md` with respective diagrams
- [x] T025 [US3] Update Project files (`project-1`, `project-2`, `project-3`) replacing architecture diagrams

## Phase 6: Polish & Cross-Cutting
*Goal: Ensure accessibility, performance, and fallback handling.*

- [x] T026 [P] Implement fallback mechanism (display static image if WebGL fails) in `ThreeDiagram/index.tsx`
- [x] T027 Add `aria-label` and keyboard navigation support to the canvas container
- [x] T028 Verify mobile responsiveness and touch controls for all diagrams
- [x] T029 Final cleanup: Remove any remaining unused Mermaid artifacts or deps

## Dependencies
- **Story Order**: Infrastructure -> Generic Builders -> Specific Builders -> Content Migration
- **Critical Path**: T003 (Engine) -> T007 (FlowBuilder) -> T017 (Manifest Population)

## Implementation Strategy
- **MVP**: A working `FlowBuilder` rendering the Chapter 1 ROS Graph.
- **Incremental**: Add `RobotBuilder` next, then migrate chapters one by one.
