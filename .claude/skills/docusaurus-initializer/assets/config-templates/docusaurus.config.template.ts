/**
 * Docusaurus Configuration Template
 *
 * This is an annotated template for configuring a Docusaurus site
 * optimized for robotics and educational content.
 *
 * Replace placeholders [LIKE_THIS] with your actual values.
 *
 * For complete documentation, see:
 * https://docusaurus.io/docs/docusaurus.config.js
 */

import { Config } from '@docusaurus/types';
import * as preset from '@docusaurus/preset-classic';

// ============================================================================
// REQUIRED: Site Identity Configuration
// ============================================================================

/**
 * The title shown in the browser tab and site header.
 * Example: "Physical AI Robotics Textbook"
 */
const siteTitle = '[SITE_TITLE]';

/**
 * The production URL of your website.
 * This is used for generating sitemaps and in metadata.
 * Examples:
 *   - "https://example.com"
 *   - "https://robotics.example.edu"
 *   - "https://my-username.github.io" (for GitHub Pages)
 */
const siteUrl = '[PRODUCTION_URL]';

/**
 * The base URL path after the domain.
 * Examples:
 *   - "/" (for root domain)
 *   - "/docs/" (if hosted at example.com/docs/)
 *   - "/robotics-textbook/" (for project site)
 *
 * For GitHub Pages:
 *   - User/org pages: "/"
 *   - Project pages: "/project-name/"
 */
const baseUrl = '/';

/**
 * Path to your site favicon (relative to static folder).
 * Typically a 32x32 PNG or ICO file.
 */
const favicon = 'img/favicon.ico';

// ============================================================================
// OPTIONAL: Organization/Project Information
// ============================================================================

/**
 * Brief tagline shown in navbar (optional).
 * Example: "Learn Robotics Through Interactive Textbooks"
 */
const tagline = '[TAGLINE]';

/**
 * Organization name (appears in metadata and footer).
 * Example: "MIT Media Lab" or "Your Robotics Program"
 */
const organizationName = '[ORG_NAME]';

/**
 * GitHub repository name (for edit URL functionality).
 * Only relevant if using GitHub for hosting.
 * Example: "robotics-textbook"
 */
const projectName = '[PROJECT_NAME]';

// ============================================================================
// ANALYTICS & TRACKING (Optional)
// ============================================================================

/**
 * Google Analytics tracking ID (optional).
 * Example: "G-XXXXXXXXXX"
 * Leave empty to disable: ""
 */
const googleAnalyticsId = '[GOOGLE_ANALYTICS_ID]';

/**
 * Algolia DocSearch credentials (optional).
 * Required for search functionality.
 * Get these from https://docsearch.algolia.com
 */
const algoliaAppId = '[ALGOLIA_APP_ID]';
const algoliaApiKey = '[ALGOLIA_API_KEY]';
const algoliaIndexName = '[ALGOLIA_INDEX_NAME]';

// ============================================================================
// MAIN CONFIGURATION
// ============================================================================

const config: Config = {
  // ──────────────────────────────────────────────────────────────────────
  // Basic Metadata
  // ──────────────────────────────────────────────────────────────────────

  title: siteTitle,
  tagline: tagline,
  url: siteUrl,
  baseUrl: baseUrl,
  favicon: favicon,

  // ──────────────────────────────────────────────────────────────────────
  // GitHub Integration (optional)
  // ──────────────────────────────────────────────────────────────────────

  organizationName: organizationName,
  projectName: projectName,

  /**
   * Enable "Edit this page" links.
   * This will add an edit button that links to your GitHub repository.
   */
  onBrokenLinks: 'throw',  // 'throw' prevents build if broken links found
  onBrokenMarkdownLinks: 'warn',

  // ──────────────────────────────────────────────────────────────────────
  // Preset Configuration
  // ──────────────────────────────────────────────────────────────────────

  presets: [
    [
      'classic',
      {
        docs: {
          // Where your documentation markdown files are located
          path: 'docs',

          // Sidebar configuration file location
          sidebarPath: require.resolve('./sidebars.ts'),

          // Enable "Edit this page" links (requires GitHub setup)
          editUrl:
            'https://github.com/[ORG]/[REPO]/tree/main/',

          // Show "Last Updated" information
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,

          // Enable remarkPlugins for advanced markdown features
          remarkPlugins: [],

          // Enable rehypePlugins for post-processing HTML
          rehypePlugins: [],
        },

        blog: {
          // Disable blog or customize as needed
          showReadingTime: true,
          editUrl:
            'https://github.com/[ORG]/[REPO]/tree/main/',
        },

        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },

        // Google Analytics plugin configuration
        googleAnalytics: {
          trackingID: googleAnalyticsId,
          anonymizeIP: true,
        },
      } as preset.Options,
    ],
  ],

  // ──────────────────────────────────────────────────────────────────────
  // Theme Configuration
  // ──────────────────────────────────────────────────────────────────────

  themeConfig: {
    /**
     * Navbar Configuration
     */
    navbar: {
      title: siteTitle,
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
        href: 'https://example.com',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docs',
          position: 'left',
          label: 'Documentation',
        },
        {
          href: 'https://github.com/[ORG]/[REPO]',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    /**
     * Footer Configuration
     */
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discussions',
              href: 'https://github.com/[ORG]/[REPO]/discussions',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/[ORG]/[REPO]',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} [ORGANIZATION]. All rights reserved.`,
    },

    /**
     * Color Mode / Dark Mode Configuration
     */
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    /**
     * Prism Code Block Highlighting
     */
    prism: {
      theme: require('prism-react-renderer/themes/dracula'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
      additionalLanguages: ['python', 'bash', 'javascript', 'typescript'],
    },

    /**
     * Algolia Search Configuration
     * Only populate these if you have Algolia credentials
     */
    algolia: {
      appId: algoliaAppId,
      apiKey: algoliaApiKey,
      indexName: algoliaIndexName,
      contextualSearch: true,
    },
  },

  // ──────────────────────────────────────────────────────────────────────
  // Scripts and Plugins
  // ──────────────────────────────────────────────────────────────────────

  scripts: [
    // Add any custom scripts here
    // {
    //   src: 'https://example.com/script.js',
    //   async: true,
    // },
  ],

  plugins: [
    // Local search plugin (alternative to Algolia)
    // Uncomment to enable and disable Algolia above
    // [
    //   require.resolve("@easyops-cn/docusaurus-search-local"),
    //   {
    //     hashed: true,
    //     language: ["en"],
    //   },
    // ],
  ],

  // ──────────────────────────────────────────────────────────────────────
  // Experimental Features
  // ──────────────────────────────────────────────────────────────────────

  future: {
    experimental_faster: true,
  },
};

export default config;

// ============================================================================
// CONFIGURATION NOTES FOR ROBOTICS/EDUCATIONAL CONTENT
// ============================================================================

/**
 * RECOMMENDED SETTINGS FOR ROBOTICS TEXTBOOKS:
 *
 * 1. Dark Mode (Recommended for coding/technical content):
 *    - colorMode.defaultMode: 'dark'
 *    - Reduces eye strain during extended reading
 *
 * 2. Code Block Highlighting:
 *    - Enable Python, Bash, JavaScript for robotics examples
 *    - Use Dracula theme for visual clarity
 *
 * 3. Search:
 *    - Use Algolia for comprehensive full-text search
 *    - Or use local search for no external dependencies
 *
 * 4. Edit on GitHub:
 *    - Makes it easy for contributors to improve docs
 *    - Enable via editUrl configuration above
 *
 * 5. Sidebar Structure:
 *    - Use sidebar position to organize chapters, labs, exercises
 *    - See sidebars.ts for detailed structure
 *
 * DEPLOYMENT TARGETS:
 *
 * For GitHub Pages (User/Org):
 *   - url: "https://username.github.io"
 *   - baseUrl: "/"
 *
 * For GitHub Pages (Project):
 *   - url: "https://username.github.io"
 *   - baseUrl: "/project-name/"
 *
 * For Custom Domain:
 *   - url: "https://yourdomain.com"
 *   - baseUrl: "/"
 *
 * For Subfolder on Existing Site:
 *   - url: "https://example.com"
 *   - baseUrl: "/robotics-docs/"
 */
