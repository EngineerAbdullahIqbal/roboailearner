import React from 'react';
import clsx from 'clsx';
import styles from './Footer.module.css';
import Link from '@docusaurus/Link'; // Import Docusaurus Link component

interface FooterLinkProps {
  label: string;
  href: string;
}

interface FooterProps {
  branding: string;
  links: FooterLinkProps[];
  authors: string;
  copyright: string;
}

function Footer({ branding, links, authors, copyright }: FooterProps) {
  return (
    <footer className={clsx(styles.footerSection, 'text--center')}>
      <div className="container">
        <div className={styles.footerBranding}>{branding}</div>
        <div className={styles.footerLinks}>
          {links.map((link, idx) => (
            <Link key={idx} to={link.href} className={styles.footerLink}> {/* Use Link component */}
              {link.label}
            </Link>
          ))}
        </div>
        <p className={styles.footerAuthors}>{authors}</p>
        <p className={styles.footerCopyright}>{copyright}</p>
      </div>
    </footer>
  );
}

export default Footer;
