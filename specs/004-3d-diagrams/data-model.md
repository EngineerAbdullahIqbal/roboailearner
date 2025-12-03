# Data Model: 3D Diagram Manifest

## Overview
The 3D diagrams are driven by a JSON manifest (`scenes/manifest.json`). This allows adding/modifying diagrams without changing code.

## Schema

```json
{
  "scenes": {
    "[scene_id]": {
      "title": "String",
      "type": "flow | robot | pointcloud | custom",
      "description": "String (for accessibility)",
      "camera": {
        "position": [x, y, z],
        "target": [x, y, z],
        "fov": 45
      },
      "assets": [
        { "id": "robot", "url": "/3d/assets/robot.glb" }
      ],
      "config": {
        // Type-specific configuration
      }
    }
  }
}
```

## Scene Types

### 1. `flow` (Node Graphs)
Used for architecture diagrams (ROS Nodes, Topics).

```json
"config": {
  "nodes": [
    { "id": "camera", "label": "Camera Node", "pos": [-2, 0, 0], "type": "box" },
    { "id": "proc", "label": "Processor", "pos": [2, 0, 0], "type": "box" }
  ],
  "edges": [
    { "from": "camera", "to": "proc", "label": "/image_raw", "animated": true, "color": "#ff0000" }
  ]
}
```

### 2. `robot` (Physical Visualization)
Used for TF trees, Hardware interfaces.

```json
"config": {
  "model": "robot", // Refers to asset ID
  "annotations": [
    { "id": "lidar", "label": "Lidar", "target_bone": "lidar_link", "text": "Velodyne VLP-16" }
  ],
  "highlight_bones": ["arm_link_1", "arm_link_2"]
}
```

### 3. `pointcloud` (Sensor Data)
Used for Chapter 5.

```json
"config": {
  "source_image": "/3d/assets/depth_sample.png",
  "visualization": "points" // or "mesh"
}
```

## File Structure

```
robotics_book_content/
├── static/
│   └── 3d/
│       ├── manifest.json
│       ├── assets/
│       │   ├── robot.glb
│       │   └── texture.png
│       └── previews/
│           └── chapter-1-graph.png
└── src/
    └── components/
        └── ThreeDiagram/
            ├── index.tsx       # Main React Component
            ├── SceneLoader.ts  # Logic to parse manifest
            ├── Builders/       # Scene generators
            │   ├── FlowBuilder.ts
            │   └── RobotBuilder.ts
            └── Engine.ts       # Three.js boilerplate (Renderer, Loop)
```
