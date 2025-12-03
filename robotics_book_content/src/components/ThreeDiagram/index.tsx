import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { Engine } from './Engine';
import { SceneLoader, SceneConfig } from './SceneLoader';
import { FlowBuilder } from './Builders/FlowBuilder';
import { RobotBuilder } from './Builders/RobotBuilder';
import { PointCloudBuilder } from './Builders/PointCloudBuilder';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useBaseUrl from '@docusaurus/useBaseUrl';

interface ThreeDiagramProps {
  id: string;
  height?: string;
  width?: string;
  fallbackImage?: string;
}

interface LabelData {
  id: string;
  text: string;
  position: THREE.Vector3;
}

const ThreeDiagramContent: React.FC<ThreeDiagramProps> = ({ 
  id, 
  height = '400px', 
  width = '100%',
  fallbackImage 
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const engineRef = useRef<Engine | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [labels, setLabels] = useState<LabelData[]>([]);
  const requestRef = useRef<number>();
  const manifestUrl = useBaseUrl('/3d/manifest.json'); // Resolve correct path

  useEffect(() => {
    let mounted = true;

    const init = async () => {
      if (!containerRef.current) return;

      try {
        // Initialize Engine
        engineRef.current = new Engine(containerRef.current);
        
        // Load Manifest with resolved URL
        const loader = new SceneLoader(manifestUrl);
        await loader.loadManifest();
        const config = loader.getSceneConfig(id);

        if (!config) {
          throw new Error(`Scene ID "${id}" not found in manifest.`);
        }

        if (!mounted) return;

        // Apply Camera Settings
        if (config.camera) {
          const camera = engineRef.current.getCamera();
          camera.position.set(...config.camera.position);
          camera.lookAt(...config.camera.target);
          if (config.camera.fov) camera.fov = config.camera.fov;
          camera.updateProjectionMatrix();
        }

        // Build Scene based on type
        if (config.type === 'flow') {
          const builder = new FlowBuilder(engineRef.current.getScene());
          builder.build(config.config as any);
          
          // Extract labels from config for HTML overlay
          const newLabels: LabelData[] = [];
          (config.config as any).nodes.forEach((node: any) => {
            newLabels.push({
              id: node.id,
              text: node.label,
              position: new THREE.Vector3(...node.pos)
            });
          });
          setLabels(newLabels);
        } else if (config.type === 'robot') {
          const builder = new RobotBuilder(engineRef.current.getScene());
          const assetId = (config.config as any).model;
          const asset = config.assets?.find(a => a.id === assetId);
          
          // Use a fallback or the provided URL
          const url = asset ? asset.url : ''; 
          await builder.build(config.config as any, url);
          
          // Get annotations for labels
          const positions = builder.getAnnotationPositions(config.config as any);
          const newLabels: LabelData[] = [];
          
          // Match positions with annotation text
          (config.config as any).annotations?.forEach((ann: any) => {
             const posData = positions.find(p => p.id === ann.id);
             if (posData) {
               newLabels.push({
                 id: ann.id,
                 text: ann.text || ann.label,
                 position: posData.position
               });
             }
          });
          setLabels(newLabels);

        } else if (config.type === 'pointcloud') {
          const builder = new PointCloudBuilder(engineRef.current.getScene());
          builder.build(config.config as any);
        }

        // Start Engine
        engineRef.current.start();
        setLoading(false);

        // Start Label Loop
        const updateLabels = () => {
          if (!engineRef.current || !containerRef.current) return;
          
          const camera = engineRef.current.getCamera();
          const width = containerRef.current.clientWidth;
          const height = containerRef.current.clientHeight;
          const widthHalf = width / 2;
          const heightHalf = height / 2;

          labels.forEach(label => {
            const el = document.getElementById(`label-${id}-${label.id}`);
            if (!el) return;

            const tempV = label.position.clone();
            
            // Small offset for robot labels if needed, or handle in Builder
            if (config.type === 'robot') {
                tempV.y += 0.5; 
            }

            tempV.project(camera);

            // Check if behind camera
            if (tempV.z > 1) {
              el.style.display = 'none';
              return;
            } 
            
            el.style.display = 'block';
            // Centering
            const x = (tempV.x * widthHalf) + widthHalf;
            const y = -(tempV.y * heightHalf) + heightHalf;
            el.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
          });

          requestRef.current = requestAnimationFrame(updateLabels);
        };
        
        // Start the loop logic, handled by the separate useEffect below 
        // but we need to make sure labels state is updated.
        
      } catch (err) {
        console.error('ThreeDiagram Init Error:', err);
        if (mounted) setError(err instanceof Error ? err.message : String(err));
      }
    };

    init();

    return () => {
      mounted = false;
      if (requestRef.current) cancelAnimationFrame(requestRef.current);
      if (engineRef.current) {
        engineRef.current.dispose();
        engineRef.current = null;
      }
    };
  }, [id]);

  // Separate effect to run the label update loop whenever labels change
  useEffect(() => {
    if (labels.length === 0 || !engineRef.current) return;

    const update = () => {
      if (!engineRef.current || !containerRef.current) return;
      const camera = engineRef.current.getCamera();
      const width = containerRef.current.clientWidth;
      const height = containerRef.current.clientHeight;
      const widthHalf = width / 2;
      const heightHalf = height / 2;

      labels.forEach(label => {
        const el = document.getElementById(`label-${id}-${label.id}`);
        if (!el) return;

        // Clone position to not modify original
        const pos = label.position.clone();
        
        // Add slight offset to be above the node/bone
        pos.y += 0.8; 

        pos.project(camera);

        // z < 1 means in front of camera frustum in NDC
        if (pos.z < 1) {
            el.style.display = 'block';
            const x = (pos.x * widthHalf) + widthHalf;
            const y = -(pos.y * heightHalf) + heightHalf;
            el.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
        } else {
            el.style.display = 'none';
        }
      });
      
      requestRef.current = requestAnimationFrame(update);
    };

    update();

    return () => {
      if (requestRef.current) cancelAnimationFrame(requestRef.current);
    };
  }, [labels, id]);

  if (error) {
    return (
      <div style={{ width, height, background: '#f8d7da', color: '#721c24', padding: '20px', display: 'flex', alignItems: 'center', justifyContent: 'center', border: '1px solid #f5c6cb', borderRadius: '4px' }}>
        <div>
          <strong>Error loading 3D Diagram:</strong> {error}
          {fallbackImage && <img src={fallbackImage} alt="Fallback" style={{ maxWidth: '100%', marginTop: '10px' }} />}
        </div>
      </div>
    );
  }

  return (
    <div 
      style={{ position: 'relative', width, height, overflow: 'hidden', borderRadius: '8px', border: '1px solid #ddd', background: '#f0f0f0' }}
      role="region"
      aria-label={`Interactive 3D Diagram: ${id}`}
    >
      {loading && (
        <div style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, background: '#eee', display: 'flex', alignItems: 'center', justifyContent: 'center', zIndex: 10 }}>
          Loading 3D Scene...
        </div>
      )}
      
      {/* Labels Overlay */}
      <div style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%', pointerEvents: 'none', overflow: 'hidden' }}>
        {labels.map(label => (
          <div 
            key={label.id} 
            id={`label-${id}-${label.id}`}
            style={{
              position: 'absolute',
              top: 0,
              left: 0,
              background: 'rgba(255, 255, 255, 0.8)',
              padding: '2px 6px',
              borderRadius: '4px',
              fontSize: '12px',
              fontWeight: 'bold',
              color: '#333',
              boxShadow: '0 1px 3px rgba(0,0,0,0.2)',
              whiteSpace: 'nowrap',
              display: 'none', // Initially hidden until positioned
              willChange: 'transform'
            }}
          >
            {label.text}
          </div>
        ))}
      </div>

      <div ref={containerRef} style={{ width: '100%', height: '100%' }} aria-label={`3D Diagram: ${id}`} role="img" />
    </div>
  );
};

export default function ThreeDiagram(props: ThreeDiagramProps): JSX.Element {
  return (
    <BrowserOnly fallback={<div style={{ height: props.height || '400px', background: '#eee' }}>Loading...</div>}>
      {() => <ThreeDiagramContent {...props} />}
    </BrowserOnly>
  );
}
