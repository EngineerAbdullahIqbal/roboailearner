import * as THREE from 'three';
import { PointCloudConfig } from '../SceneLoader';

export class PointCloudBuilder {
  private scene: THREE.Scene;
  private pointCloud: THREE.Points | null = null;

  constructor(scene: THREE.Scene) {
    this.scene = scene;
  }

  public build(config: PointCloudConfig) {
    this.clear();

    if (config.source_image) {
      this.buildFromImage(config.source_image);
    } else {
      this.buildRandom();
    }
  }

  private buildRandom() {
    const geometry = new THREE.BufferGeometry();
    const count = 5000;
    const positions = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);

    for (let i = 0; i < count; i++) {
      const x = (Math.random() - 0.5) * 10;
      const y = (Math.random() - 0.5) * 10;
      const z = (Math.random() - 0.5) * 10;

      positions[i * 3] = x;
      positions[i * 3 + 1] = y;
      positions[i * 3 + 2] = z;

      colors[i * 3] = (x + 5) / 10;
      colors[i * 3 + 1] = (y + 5) / 10;
      colors[i * 3 + 2] = (z + 5) / 10;
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

    const material = new THREE.PointsMaterial({ size: 0.1, vertexColors: true });
    this.pointCloud = new THREE.Points(geometry, material);
    this.scene.add(this.pointCloud);
  }

  private buildFromImage(url: string) {
    // Simplified implementation: Create a plane of points
    // Real depth displacement would require reading pixel data from a canvas
    const loader = new THREE.TextureLoader();
    loader.load(url, (texture) => {
      const width = 100;
      const height = 100;
      const geometry = new THREE.BufferGeometry();
      const positions = new Float32Array(width * height * 3);
      
      // Create a canvas to read pixel data
      const canvas = document.createElement('canvas');
      canvas.width = texture.image.width;
      canvas.height = texture.image.height;
      const context = canvas.getContext('2d');
      if (context) {
        context.drawImage(texture.image, 0, 0);
        const data = context.getImageData(0, 0, canvas.width, canvas.height).data;
        
        let ptr = 0;
        for (let i = 0; i < width; i++) {
          for (let j = 0; j < height; j++) {
            const x = (i - width / 2) * 0.1;
            const y = (j - height / 2) * 0.1;
            
            // Sample depth from image (simplified sampling)
            const imgX = Math.floor((i / width) * texture.image.width);
            const imgY = Math.floor((j / height) * texture.image.height);
            const pixelIndex = (imgY * texture.image.width + imgX) * 4;
            const depthVal = data[pixelIndex] / 255.0; // Red channel 0-1

            const z = depthVal * 5; 

            positions[ptr] = x;
            positions[ptr + 1] = y;
            positions[ptr + 2] = z;
            ptr += 3;
          }
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const material = new THREE.PointsMaterial({ size: 0.05, color: 0x00ffff });
        this.pointCloud = new THREE.Points(geometry, material);
        this.scene.add(this.pointCloud);
      }
    });
  }

  public clear() {
    if (this.pointCloud) {
      this.scene.remove(this.pointCloud);
      this.pointCloud.geometry.dispose();
      (this.pointCloud.material as THREE.Material).dispose();
      this.pointCloud = null;
    }
  }
}
