import {Config} from '@docusaurus/types';
import * as preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Learn embodied intelligence, ROS 2, simulation, and vision-language models',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://devhammad0.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-robotics-textbook/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'devhammad0',
  projectName: 'physical-ai-robotics-textbook',
  trailingSlash: false,
  onBrokenLinks: 'warn',

  markdown: {
    mermaid: true,
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/hammad/physical-ai-robotics-textbook/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/logo.svg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook',
      // logo: {
      //   alt: 'Physical AI Logo',
      //   src: 'img/logo.svg',
      // },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {
          href: 'https://github.com/hammad/physical-ai-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Chapters',
          items: [
            {
              label: 'ROS 2',
              to: '/docs/chapters/chapter-1-ros2',
            },
            {
              label: 'Gazebo',
              to: '/docs/chapters/chapter-2-gazebo',
            },
            {
              label: 'NVIDIA Isaac',
              to: '/docs/chapters/chapter-3-isaac',
            },
            {
              label: 'Vision-Language-Action',
              to: '/docs/chapters/chapter-4-vla',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/devhammad0/physical-ai-robotics-textbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: require('prism-react-renderer').themes.github,
      darkTheme: require('prism-react-renderer').themes.dracula,
    },
  } satisfies preset.ThemeConfig,

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        indexDocs: true,
        indexPages: false,
        language: 'en',
      },
    ],
  ],
};

export default config;
