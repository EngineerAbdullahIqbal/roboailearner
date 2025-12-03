import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  plugins: [
    require.resolve('./plugins/my-webpack-aliases'),
  ],
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  favicon: 'img/book-logo.png',

  customFields: {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
  },

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://engineerabdullahiqbal.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
    baseUrl: '/roboailearner/',

  // GitHub pages deployment config.
  organizationName: 'EngineerAbdullahIqbal', // Usually your GitHub org/user name.
  projectName: 'roboailearner', // Usually your repo name.

  onBrokenLinks: 'throw',
  // onBrokenMarkdownLinks: 'warn', // Deprecated, use siteConfig.markdown.onBrokenMarkdownLinks if needed

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/EngineerAbdullahIqbal/roboailearner/tree/main/robotics_book_content/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/EngineerAbdullahIqbal/roboailearner/tree/main/robotics_book_content/',
          onInlineAuthors: 'ignore',
          onUntruncatedBlogPosts: 'ignore',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'RoboAI',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Curriculum',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/EngineerAbdullahIqbal/roboailearner',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    // footer: {
    //   style: 'dark',
    //   links: [
    //     {
    //       title: 'Curriculum',
    //       items: [
    //         { label: 'Foundations', to: '/docs/module-1/chapter-1' },
    //         { label: 'Perception', to: '/docs/module-2/chapter-4' },
    //         { label: 'Control', to: '/docs/module-3/chapter-7' },
    //         { label: 'Advanced', to: '/docs/module-4/chapter-10' },
    //       ],
    //     },
    //     {
    //       title: 'Projects',
    //       items: [
    //         { label: 'Sentient Sentry', to: '/docs/module-5/project-1-sentient-sentry' },
    //         { label: 'Visual Sorter', to: '/docs/module-5/project-2-visual-sorter' },
    //         { label: 'Office Runner', to: '/docs/module-5/project-3-office-runner' },
    //       ],
    //     },
    //     {
    //       title: 'Community',
    //       items: [
    //         {
    //           label: 'GitHub Repo',
    //           href: 'https://github.com/EngineerAbdullahIqbal/hackathon-book-project',
    //         },
    //       ],
    //     },
    //   ],
    //   copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
    // },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'yaml'],
    },
    mermaid: {
      theme: {light: 'neutral', dark: 'dark'},
    },
  } satisfies Preset.ThemeConfig,
};

export default config;