export interface SceneManifest {
  scenes: {
    [key: string]: SceneConfig;
  };
}

export interface SceneConfig {
  title: string;
  type: 'flow' | 'robot' | 'pointcloud' | 'custom';
  description: string;
  camera?: {
    position: [number, number, number];
    target: [number, number, number];
    fov?: number;
  };
  assets?: Asset[];
  config: FlowConfig | RobotConfig | PointCloudConfig | CustomConfig;
}

export interface Asset {
  id: string;
  url: string;
}

export interface FlowConfig {
  nodes: FlowNode[];
  edges: FlowEdge[];
}

export interface FlowNode {
  id: string;
  label: string;
  pos: [number, number, number];
  type?: 'box' | 'sphere' | 'cylinder';
  color?: string;
}

export interface FlowEdge {
  from: string;
  to: string;
  label?: string;
  color?: string;
  animated?: boolean;
}

export interface RobotConfig {
  model: string; // Asset ID
  annotations?: Annotation[];
  highlight_bones?: string[];
}

export interface Annotation {
  id: string;
  label: string;
  target_bone: string;
  text: string;
}

export interface PointCloudConfig {
  source_image: string; // URL
  visualization: 'points' | 'mesh';
}

export interface CustomConfig {
  [key: string]: any;
}

export class SceneLoader {
  private manifestUrl: string;
  private manifest: SceneManifest | null = null;

  constructor(manifestUrl: string = '/3d/manifest.json') {
    this.manifestUrl = manifestUrl;
  }

  public async loadManifest(): Promise<SceneManifest> {
    if (this.manifest) return this.manifest;
    try {
      const response = await fetch(this.manifestUrl);
      if (!response.ok) {
        throw new Error(`Failed to load manifest: ${response.statusText}`);
      }
      this.manifest = await response.json();
      return this.manifest!;
    } catch (error) {
      console.error('SceneLoader error:', error);
      throw error;
    }
  }

  public getSceneConfig(id: string): SceneConfig | undefined {
    return this.manifest?.scenes[id];
  }
}
