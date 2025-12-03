import React from 'react';
import styles from './styles.module.css';

interface VisualDiagramProps {
  caption?: string;
  children: React.ReactNode;
  type?: 'mermaid' | 'image';
}

export default function VisualDiagram({
  caption,
  children,
  type = 'image',
}: VisualDiagramProps): JSX.Element {
  return (
    <div className={styles.container}>
      <figure className={styles.figure}>
        <div className={type === 'mermaid' ? styles.mermaidWrapper : ''}>
          {children}
        </div>
        {caption && <figcaption className={styles.caption}>{caption}</figcaption>}
      </figure>
    </div>
  );
}
