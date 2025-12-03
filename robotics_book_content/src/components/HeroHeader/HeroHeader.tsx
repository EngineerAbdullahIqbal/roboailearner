import React from 'react';
import clsx from 'clsx';
import styles from './HeroHeader.module.css';

interface HeroHeaderProps {
  title: string;
  subtitle: string;
  ctaLabel: string;
  ctaLink: string;
}

function HeroHeader({ title, subtitle, ctaLabel, ctaLink }: HeroHeaderProps) {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{title}</h1>
        <p className="hero__subtitle">{subtitle}</p>
        <div className={styles.buttons}>
          <a
            className="button button--secondary button--lg"
            href={ctaLink}>
            {ctaLabel}
          </a>
        </div>
      </div>
    </header>
  );
}

export default HeroHeader;