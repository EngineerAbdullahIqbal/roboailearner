import React from 'react';
import clsx from 'clsx';
import styles from './ChaptersSection.module.css';

interface ChapterCardProps {
  chapterNumber: number;
  title: string;
  synopsis: string;
  illustration: string; // Path to SVG illustration
  targetLink: string;
}

function ChapterCard({ chapterNumber, title, synopsis, illustration, targetLink }: ChapterCardProps) {
  return (
    <a href={targetLink} className={styles.chapterCard}>
      <img src={illustration} alt={`Chapter ${chapterNumber} Illustration`} className={styles.chapterIllustration} />
      <div className={styles.chapterContent}>
        <span className={styles.chapterNumber}>Chapter {chapterNumber}</span>
        <h3 className={styles.chapterTitle}>{title}</h3>
        <p className={styles.chapterSynopsis}>{synopsis}</p>
      </div>
    </a>
  );
}

interface ChaptersSectionProps {
  chapterCards: ChapterCardProps[];
}

function ChaptersSection({ chapterCards }: ChaptersSectionProps) {
  return (
    <section className={styles.chaptersSection}>
      <div className="container">
        <h2 className="text--center">Chapters</h2>
        <div className={clsx('row', styles.chapterGrid)}>
          {chapterCards.map((card, idx) => (
            <div key={idx} className={clsx('col col--4', styles.chapterGridItem)}>
              <ChapterCard {...card} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default ChaptersSection;