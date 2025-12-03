import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { RobotConfig, Annotation } from '../SceneLoader';

export class RobotBuilder {
  private scene: THREE.Scene;
  private loader: GLTFLoader;
  private robotModel: THREE.Object3D | null = null;
  private annotations: Map<string, Annotation> = new Map();

  constructor(scene: THREE.Scene) {
    this.scene = scene;
    this.loader = new GLTFLoader();
  }

  public async build(config: RobotConfig, assetUrl: string) {
    this.clear();

    return new Promise<void>((resolve, reject) => {
      this.loader.load(
        assetUrl,
        (gltf) => {
          this.robotModel = gltf.scene;
          this.scene.add(this.robotModel);

          // Handle highlighting bones
          if (config.highlight_bones) {
            config.highlight_bones.forEach(boneName => {
              const bone = this.robotModel?.getObjectByName(boneName);
              if (bone) {
                // Add a visual indicator (e.g., a wireframe box or color change)
                // For simplicity, let's attach a small axis helper
                const axesHelper = new THREE.AxesHelper(0.5);
                bone.add(axesHelper);
              }
            });
          }

          // Handle Annotations (store data for HTML overlay to use)
          // The actual HTML rendering happens in the React component, 
          // but we need to identify the target positions.
          // This Builder might not render text directly if we are using HTML overlays.
          // We can return the positions of the bones.
          
          resolve();
        },
        undefined,
        (error) => {
          console.error('An error happened loading the robot model:', error);
          // Fallback: Create a simple box if model fails
          const geometry = new THREE.BoxGeometry(1, 2, 1);
          const material = new THREE.MeshStandardMaterial({ color: 0x888888 });
          this.robotModel = new THREE.Mesh(geometry, material);
          this.scene.add(this.robotModel);
          resolve();
        }
      );
    });
  }

  public getAnnotationPositions(config: RobotConfig): { id: string; position: THREE.Vector3 }[] {
    const positions: { id: string; position: THREE.Vector3 }[] = [];
    
    if (!this.robotModel || !config.annotations) return positions;

    config.annotations.forEach(annotation => {
      const bone = this.robotModel!.getObjectByName(annotation.target_bone);
      if (bone) {
        // Get world position of the bone
        const position = new THREE.Vector3();
        bone.getWorldPosition(position);
        positions.push({ id: annotation.id, position });
      } else {
        // If bone not found, maybe position relative to model root?
        // Or just ignore.
        console.warn(`Bone ${annotation.target_bone} not found for annotation ${annotation.id}`);
      }
    });

    return positions;
  }

  public clear() {
    if (this.robotModel) {
      this.scene.remove(this.robotModel);
      // Dispose traversal could go here
      this.robotModel = null;
    }
  }
}
