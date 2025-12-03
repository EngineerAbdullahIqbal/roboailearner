import React from 'react';
import clsx from 'clsx';
import styles from './InteractiveFeatures.module.css';

interface FeatureCardProps {
  name: string;
  description: string;
  illustration: string; // Path to SVG illustration
}

function FeatureCard({ name, description, illustration }: FeatureCardProps) {
  return (
    <div className={styles.featureCard}>
      <img src={illustration} alt={`${name} Illustration`} className={styles.featureIllustration} />
      <h3 className={styles.featureName}>{name}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

interface InteractiveFeaturesProps {
  features: FeatureCardProps[];
}

function InteractiveFeatures({ features }: InteractiveFeaturesProps) {
  return (
    <section className={styles.interactiveFeaturesSection}>
      <div className="container">
        <h2 className="text--center">Interactive Features</h2>
        <div className={clsx('row', styles.featureGrid)}>
          {features.map((feature, idx) => (
            <div key={idx} className={clsx('col col--4', styles.featureGridItem)}>
              <FeatureCard {...feature} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default InteractiveFeatures;