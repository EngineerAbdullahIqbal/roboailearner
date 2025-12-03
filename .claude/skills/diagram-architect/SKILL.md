# ROS 2 Diagram Architect

## Overview
Generates syntactically correct and visually consistent Mermaid.js diagrams for ROS 2 architectures (Node Graphs) and Hardware set-ups. It enforces the textbook's visual theme (Dracula) and standardizes how nodes, topics, and devices are represented.

## Activation Triggers

### Keyword Triggers
- "create ros diagram"
- "visualize node graph"
- "add wiring diagram"
- "generate system architecture"
- "draw hardware setup"

### Command Triggers
```bash
# Explicit activation
generate_diagram --type ros --nodes "CameraNode, VLANode" --topics "/image_raw, /cmd_vel"
generate_diagram --type hardware --devices "JetsonOrin, RealSense, UnitreeGo2"
```

## Core Functionality

### 1. ROS Graph Generation
- Creates standard "RQT Graph" style diagrams.
- **Nodes**: Represented as Ellipses (circles).
- **Topics**: Represented as Rectangles (squares).
- **Actions/Services**: Represented as Rhombuses.
- Automatically applies the project's color palette.

### 2. Hardware Wiring Generation
- Creates physical connection diagrams.
- standardizes port connections (USB, MIPI-CSI, UART, Ethernet).
- Groups components by sub-system (e.g., "Power Subsystem", "Perception Subsystem").

### 3. Syntax Validation
- Ensures generated Mermaid code is valid.
- Wraps output in Docusaurus `<Mermaid>` components.
- Adds accessibility titles and captions.

## Inputs

### Required Parameters
```python
{
  "type": str,            # "ros" or "hardware"
  "components": List[str] # List of nodes or devices
}
```

### Optional Parameters
```python
{
  "connections": List[str], # e.g., ["NodeA->Topic1", "Topic1->NodeB"]
  "title": str              # Diagram title
}
```

## Outputs

### Generated Content
Returns a Markdown code block ready to paste into `.mdx` files:

```jsx
<Mermaid chart={`
  graph LR;
    %% Styles
    classDef node fill:#ff79c6,stroke:#bd93f9,stroke-width:2px;
    classDef topic fill:#8be9fd,stroke:#6272a4,stroke-width:2px;
    
    %% Graph
    CameraNode((Camera Node)) --> /image_raw[/image_raw/]
    /image_raw[/image_raw/] --> VisionNode((Vision Node))
`}/>
```

## Usage Examples

### Scenario 1: ROS Node Graph
```
User: "Create a diagram for a camera publishing to a VLA model"

Skill activates:
✓ Detected type: ROS Graph
✓ Nodes: Camera, VLA_Model
✓ Topic: /image_raw
✓ Generated Mermaid syntax with 'Dracula' theme
```

### Scenario 2: Hardware Setup
```
User: "Draw the wiring for Jetson connected to RealSense via USB"

Skill activates:
✓ Detected type: Hardware
✓ Devices: Jetson Orin, RealSense D435i
✓ Connection: USB-C
✓ Generated wiring diagram
```