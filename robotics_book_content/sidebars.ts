import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'course-outline', // Link to the main course outline
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI & ROS 2',
      items: [
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Sensing and Perception for Humanoids',
      items: [
        'module-2/chapter-4',
        'module-2/chapter-5',
        'module-2/chapter-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Motion, Control, and Navigation',
      items: [
        'module-3/chapter-7',
        'module-3/chapter-8',
        'module-3/chapter-9',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Advanced Topics and Real-World Deployment',
      items: [
        'module-4/chapter-10',
        'module-4/chapter-11',
        'module-4/chapter-12',
        'module-4/chapter-13',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Projects',
      items: [
        'module-5/chapter-14-student-projects',
        'module-5/project-1-sentient-sentry',
        'module-5/project-2-visual-sorter',
        'module-5/project-3-office-runner',
      ],
    },
  ],
};

export default sidebars;
