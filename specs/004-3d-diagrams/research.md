# Research: 3D Interactive Diagrams

## Objective
Replace static Mermaid diagrams with interactive 3D visualizations using `three.js` to enhance the "Physical AI" learning experience.

## Diagram Inventory & Analysis

| ID | Location | Original Content | 3D Concept | Complexity |
|---|---|---|---|---|
| 1.1 | Ch 1 | ROS Node Graph (Camera->Actuator) | **3D Flow:** Floating nodes with data packets flowing between them. Connected to a simplified low-poly robot model to show physical context. | Low |
| 2.1 | Ch 2 | Two-node Graph (Topic) | **3D Pub/Sub:** Visualizing the "bus" or connection. Data packets traveling. | Low |
| 2.2 | Ch 2 | Service Architecture (Req/Res) | **3D Request/Response:** Animation showing a "Request" packet going out and a "Response" packet returning. | Low |
| 3.1 | Ch 3 | Gazebo Architecture | **Layered Stack:** "Real World" (User) -> "Software" (ROS) -> "Simulation" (Gazebo physics). Exploded view of the stack. | Med |
| 4.1 | Ch 4 | Image Processing Pipeline | **Data Transformation:** Raw image texture floating -> Entering "Processing" box -> Edge-detected texture coming out. | Med |
| 5.1 | Ch 5 | Point Cloud Pipeline | **Depth Viz:** 2D Depth image transforming into 3D points in space (particle system). | High |
| 6.1 | Ch 6 | Sensor Fusion Architecture | **Fusion Core:** Multiple inputs (Lidar, Camera, IMU) feeding into a central "Fusion" core, outputting a stable "Pose" arrow. | Med |
| 6.2 | Ch 6 | TF2 System | **Coordinate Tree:** 3D visualization of coordinate frames (`map`, `odom`, `base_link`) attached to a robot model. | High |
| 7.1 | Ch 7 | MoveIt 2 Architecture | **Motion Planning:** Ghost robot showing planned path vs. current state. Exploded view of MoveIt components. | High |
| 8.1 | Ch 8 | ros2_control Stack | **Control Loop:** Visual PID loop. Error bars shrinking as "force" is applied to a virtual joint. | Med |
| 9.1 | Ch 9 | Nav2 Stack | **Nav Stack:** Layered costmaps (Static, Inflation, Dynamic) visualized as stacked 3D planes. | High |
| P1.1 | Proj 1 | Sentient Sentry Arch | **System View:** Camera tracking a target (face) in 3D space. | Med |
| P2.1 | Proj 2 | Visual Sorter Arch | **Manipulation:** Arm picking object. Visualization of the "Perception-Action" loop. | High |
| P3.1 | Proj 3 | Office Runner Arch | **Navigation:** Robot navigating a simple maze. | High |

## Technical Strategy

### 1. Engine: Three.js (Vanilla wrapped in React)
Since the host is Docusaurus (React), we will create a `ThreeScene` React component.
*   **Why?** Integrates cleanly with MDX.
*   **Implementation:** `useEffect` hook initializes `THREE.WebGLRenderer`, `Scene`, `Camera`.
*   **State Management:** React state controls the "manifest" loading, but the render loop is pure Three.js.

### 2. Reusable "Builders"
Most diagrams are variations of "Node Graphs". We will build a generic `GraphBuilder` class in Three.js.
*   **Input:** JSON nodes and edges.
*   **Output:** 3D Group with:
    *   **Nodes:** Rounded cubes or spheres (Low Poly).
    *   **Edges:** Curves/Tubes connecting nodes.
    *   **Animation:** "Particles" traveling along tubes to simulate data flow.
    *   **Labels:** HTML overlays (`CSS2DRenderer` or simple absolute positioned divs tracked to 3D positions).

### 3. Specific "Simulations"
For Ch 5 (Point Cloud), Ch 7 (IK), Ch 9 (Nav), we need specific visualizers.
*   **PointCloudViewer:** Uses `THREE.Points`.
*   **RobotViewer:** Loads a `.glb` of a generic robot (Unitree Go2 or generic arm).

### 4. Asset Strategy
*   **Models:** Use primitives (Box, Sphere, Cylinder) for abstract concepts. Use a single high-quality `.glb` for the "Physical Robot" reference, re-used across scenes.
*   **Textures:** Procedural grid textures for "ground".
*   **Fonts:** Use system fonts via HTML overlay to avoid heavy 3D font files and improve accessibility.

### 5. Accessibility & Fallback
*   **Canvas:** `aria-label` describing the scene.
*   **Keyboard:** `OrbitControls` supports keys. Add custom tab-focus to "Nodes" to show tooltips.
*   **Fallback:** `noscript` or conditional rendering for static images if WebGL fails.

### 6. Delivery Pipeline
1.  Create `src/components/ThreeScene/` in `robotics_book_content`.
2.  Create `static/3d/manifest.json`.
3.  Create `static/3d/assets/` (for glb).
4.  Modify Markdown files to import `<ThreeScene id="..." />`.

## Decision Record
*   **Decision:** Use HTML overlays for text instead of `TextGeometry`.
    *   **Rationale:** Better readability, accessibility, standard CSS styling, no heavy font files.
*   **Decision:** One generic `GraphScene` type for 80% of diagrams.
    *   **Rationale:** Reduces dev effort. Most diagrams are "A -> B -> C".
*   **Decision:** Use "Low Poly" aesthetic.
    *   **Rationale:** Performance on mobile/web, matches "Blueprint" feel.

## Unknowns / Risks
*   **Performance:** Multiple WebGL contexts on one page (if multiple diagrams per chapter).
    *   *Mitigation:* Use a single background renderer or Intersection Observer to only render when in viewport.
*   **Mobile Support:** Touch controls for OrbitControls.
