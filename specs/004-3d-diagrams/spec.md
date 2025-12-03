# Feature Spec: 3D Interactive Diagrams

## Objective
Replace all static/Mermaid diagrams in the Robotics + AI book with interactive 3D diagrams using `three.js`.

## Requirements

### Delete Mermaid Diagrams
- Find and remove all Mermaid diagram blocks.
- Log removed diagrams.

### 3D Implementation Tech Stack
- **Renderer:** `three.js`
- **Helpers:** `OrbitControls`, `GLTFLoader`, `DRACOLoader` (optional), `RGBELoader`.
- **Assets:** `.glb/.gltf` for models, `.png` for static views.

### Output Format
- **Plan:** Structured list of every concept needing a 3D diagram.
  - Details: Title, purpose, perspective, components, interactivity, assets, complexity.
- **Manifest:** `scenes/manifest.json` for loading scenes.
- **Scaffold:** Working `three.js` scaffold (HTML + JS) supporting:
  - Camera controls.
  - Toggleable labels.
  - Explode/assemble animation.
  - Hover highlights/info panels.
  - Responsive canvas, LOD.

### Design & Consistency
- **Style Guide:** Color palette, fonts, iconography, lighting, units, naming.

### Export & Deliverables
- Per diagram: `.glb`, `meta.json`, `.png` hero, HTML preview.
- Single preview index page.

### Quality & Accessibility
- Keyboard navigation.
- Alt-text.
- Focusable controls, ARIA labels.

### Performance & Fallback
- Low-detail fallback (PNG/SVG) for print.
- Medium-quality fallback for mobile.
- DRACO compression.

### Testing & Logs
- Automated test checklist per scene.
- Removal log for Mermaid diagrams.

## Behavioral Constraints
- Prefer native geometry over heavy models where possible.
- No Mermaid artifacts remaining.
- Deterministic filenames.
- Flag medium-fidelity diagrams for review if in doubt.
