import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface ConceptCardProps {
  title: string;
  description: string;
  icon?: React.ComponentType<React.ComponentProps<'svg'>> | string;
  linkUrl?: string;
  color?: string; // Optional accent color override
}

export default function ConceptCard({
  title,
  description,
  icon,
  linkUrl,
  color,
}: ConceptCardProps): JSX.Element {
  const CardContent = (
    <>
      <div className={styles.iconWrapper} style={color ? { color: color, backgroundColor: `${color}20` } : {}}>
        {typeof icon === 'string' ? (
          <img src={icon} alt={title} style={{ width: 24, height: 24 }} />
        ) : (
          icon && React.createElement(icon, { style: { width: 24, height: 24 } })
        )}
      </div>
      <h3 className={styles.title}>{title}</h3>
      <p className={styles.description}>{description}</p>
      {linkUrl && (
        <div className={styles.linkText} style={color ? { color: color } : {}}>
          Learn More <span>&rarr;</span>
        </div>
      )}
    </>
  );

  if (linkUrl) {
    return (
      <Link to={linkUrl} className={styles.card} style={color ? { borderColor: `${color}40` } : {}}>
        {CardContent}
      </Link>
    );
  }

  return (
    <div className={styles.card} style={color ? { borderColor: `${color}40` } : {}}>
      {CardContent}
    </div>
  );
}
