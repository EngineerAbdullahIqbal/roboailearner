import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './HeroBanner.module.css'; // Assuming a CSS module for custom styles

function HeroBanner({ title, description, ctaLabel, ctaLink }) {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{title}</h1>
        <p className="hero__subtitle">{description}</p>
        {ctaLabel && ctaLink && (
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to={ctaLink}>
              {ctaLabel}
            </Link>
          </div>
        )}
      </div>
    </header>
  );
}

export default HeroBanner;