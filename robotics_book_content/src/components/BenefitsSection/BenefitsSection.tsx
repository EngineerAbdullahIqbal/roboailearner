import React from 'react';
import clsx from 'clsx';
import styles from './BenefitsSection.module.css';

interface BenefitCardProps {
  title: string;
  description: string;
}

function BenefitCard({ title, description }: BenefitCardProps) {
  return (
    <div className={styles.benefitCard}>
      <h3 className={styles.benefitTitle}>{title}</h3>
      <p className={styles.benefitDescription}>{description}</p>
    </div>
  );
}

interface BenefitsSectionProps {
  benefits: BenefitCardProps[];
}

function BenefitsSection({ benefits }: BenefitsSectionProps) {
  return (
    <section className={styles.benefitsSection}>
      <div className="container">
        <h2 className={clsx('text--center', styles.sectionTitle)}>Benefits of Robotics AI & Our Book</h2>
        <div className={styles.scrollContainer}>
          <div className={styles.benefitGrid}>
            {benefits.map((benefit, idx) => (
              <BenefitCard key={idx} {...benefit} />
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

export default BenefitsSection;