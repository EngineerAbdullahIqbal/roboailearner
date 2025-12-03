import React from 'react';
import clsx from 'clsx';
import styles from './FutureOfRoboticsAI.module.css';

interface FutureOfRoboticsAIProps {
  title: string;
  description: string;
}

function FutureOfRoboticsAI({ title, description }: FutureOfRoboticsAIProps) {
  return (
    <section className={styles.futureOfRoboticsAISection}>
      <div className="container">
        <h2 className={clsx('text--center', styles.sectionTitle)}>{title}</h2>
        <p className={clsx('text--center', styles.sectionDescription)}>{description}</p>
      </div>
    </section>
  );
}

export default FutureOfRoboticsAI;