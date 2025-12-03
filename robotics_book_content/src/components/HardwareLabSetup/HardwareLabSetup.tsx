import React from 'react';
import clsx from 'clsx';
import styles from './HardwareLabSetup.module.css';

interface HardwareCardProps {
  name: string;
  illustration: string; // Path to PNG illustration
}

function HardwareCard({ name, illustration }: HardwareCardProps) {
  return (
    <div className={styles.hardwareCard}>
      <img src={illustration} alt={`${name} Illustration`} className={styles.hardwareIllustration} />
      <h3 className={styles.hardwareName}>{name}</h3>
    </div>
  );
}

interface HardwareLabSetupProps {
  hardwareItems: HardwareCardProps[];
}

function HardwareLabSetup({ hardwareItems }: HardwareLabSetupProps) {
  return (
    <section className={styles.hardwareLabSetupSection}>
      <div className="container">
        <h2 className="text--center">Hardware & Lab Setup</h2>
        <div className={clsx('row', styles.hardwareGrid)}>
          {hardwareItems.map((item, idx) => (
            <div key={idx} className={clsx('col col--4', styles.hardwareGridItem)}>
              <HardwareCard {...item} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default HardwareLabSetup;