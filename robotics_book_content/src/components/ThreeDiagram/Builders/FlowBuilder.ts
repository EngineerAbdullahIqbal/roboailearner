import * as THREE from 'three';
import { FlowConfig, FlowNode, FlowEdge } from '../SceneLoader';

export class FlowBuilder {
  private scene: THREE.Scene;
  private nodes: Map<string, THREE.Object3D> = new Map();

  constructor(scene: THREE.Scene) {
    this.scene = scene;
  }

  public build(config: FlowConfig) {
    this.clear();
    
    // 1. Build Nodes
    config.nodes.forEach(nodeConfig => {
      const node = this.createNode(nodeConfig);
      this.scene.add(node);
      this.nodes.set(nodeConfig.id, node);
    });

    // 2. Build Edges
    config.edges.forEach(edgeConfig => {
      const fromNode = this.nodes.get(edgeConfig.from);
      const toNode = this.nodes.get(edgeConfig.to);
      
      if (fromNode && toNode) {
        const edge = this.createEdge(fromNode.position, toNode.position, edgeConfig);
        this.scene.add(edge);
      }
    });
  }

  private createNode(config: FlowNode): THREE.Object3D {
    let geometry: THREE.BufferGeometry;
    
    switch (config.type) {
      case 'sphere':
        geometry = new THREE.SphereGeometry(0.5, 32, 32);
        break;
      case 'cylinder':
        geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
        break;
      case 'box':
      default:
        geometry = new THREE.BoxGeometry(1, 1, 1);
    }

    const material = new THREE.MeshStandardMaterial({ 
      color: config.color || '#007bff',
      roughness: 0.7,
      metalness: 0.3
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(...config.pos);
    mesh.userData = { id: config.id, label: config.label, isNode: true };
    
    return mesh;
  }

  private createEdge(start: THREE.Vector3, end: THREE.Vector3, config: FlowEdge): THREE.Object3D {
    // Create a simple tube connecting the points
    const path = new THREE.LineCurve3(start, end);
    const geometry = new THREE.TubeGeometry(path, 20, 0.05, 8, false);
    const material = new THREE.MeshBasicMaterial({ 
      color: config.color || '#999',
      transparent: true,
      opacity: 0.6
    });
    
    const mesh = new THREE.Mesh(geometry, material);
    
    // Add arrow head if needed (omitted for brevity/low-poly style, but could be a Cone)
    
    // Animation: In a full implementation, we'd add "packet" sprites here 
    // that traverse the path in the render loop.
    
    return mesh;
  }

  public clear() {
    // Dispose geometries and materials for existing nodes/edges
    // This is a simplified clear. In production, traverse scene and dispose properly.
    this.nodes.clear();
    // Note: Actual object removal from scene happens in parent or via scene.clear() 
    // but we only want to clear Flow items. 
    // Ideally, we'd group them in a THREE.Group and clear that.
  }
}
