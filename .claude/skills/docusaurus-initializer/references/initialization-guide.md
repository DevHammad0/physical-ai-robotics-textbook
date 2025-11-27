# Docusaurus Initialization Guide

Quick start guide for setting up Docusaurus with pnpm and TypeScript.

## Prerequisites

### System Requirements

- **Node.js**: v18.0.0 or higher
- **pnpm**: v8.0.0 or higher
- **Bash**: For running the theme installer script

### Verification

Verify your system is ready:

```bash
# Check Node.js version (should be 18+)
node --version

# Install pnpm globally if needed
npm install -g pnpm

# Verify pnpm installation
pnpm --version
```

Expected output:
```
v18.16.0
8.6.5
```

## Step 1: Create a New Docusaurus Project

Initialize a new Docusaurus project with TypeScript:

```bash
# Create project
pnpm create docusaurus@latest my-website classic --typescript

# Navigate to project
cd my-website

# Install dependencies
pnpm install
```

**Project Structure:**

```
my-website/
├── docs/                    # Documentation files (Markdown/MDX)
├── blog/                    # Blog posts (optional)
├── src/
│   ├── css/custom.css      # Custom styles (create if not exists)
│   ├── components/         # Custom React components
│   └── pages/             # Custom pages
├── static/                # Static assets (images, fonts, etc.)
├── docusaurus.config.ts   # Main configuration file
├── sidebars.ts           # Sidebar navigation structure
├── package.json          # Project dependencies
├── tsconfig.json         # TypeScript configuration
└── .gitignore           # Git ignore patterns
```

## Step 2: Verify Installation

Test that Docusaurus is working:

```bash
# Type check
pnpm run typecheck

# Build site
pnpm run build

# Start development server
pnpm run start
```

The site should open at http://localhost:3000

## Step 3: Install Custom Theme

Copy and apply the custom dark mode theme:

```bash
# Make script executable (if on macOS/Linux)
chmod +x scripts/install-robotics-theme.sh

# Run theme installer
bash scripts/install-robotics-theme.sh
```

The script will:
- Create `src/css/` directory
- Copy `custom.css` from assets
- Apply theme colors and styles

## Step 4: Configure Your Site

### Copy Configuration Template

```bash
cp assets/config-templates/docusaurus.config.template.ts docusaurus.config.ts
```

### Edit Configuration

Open `docusaurus.config.ts` and update:

```typescript
export default {
  // Required fields
  title: 'Your Site Title',              // Browser tab title
  url: 'https://example.com',           // Production domain
  baseUrl: '/',                         // / for root, /project/ for subdirectory
  favicon: 'img/favicon.ico',          // Favicon path

  // Metadata (optional)
  organizationName: 'your-github-username',
  projectName: 'your-repo-name',

  // Theme configuration
  themeConfig: {
    navbar: {
      title: 'Your Site Title',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
      },
    },
    colorMode: {
      defaultMode: 'dark',              // Start with dark mode
      disableSwitch: false,              // Allow theme toggle
    },
  },
};
```

## Step 5: Test Theme

Rebuild and test with the new theme:

```bash
# Clear cache
pnpm run clear

# Rebuild
pnpm run build

# Start development server
pnpm run start
```

Visit http://localhost:3000 and verify:
- ✅ Dark theme is applied
- ✅ Colors look correct
- ✅ Layout is responsive
- ✅ No TypeScript errors

## Configuration Reference

### Essential Fields

| Field | Type | Purpose | Example |
|-------|------|---------|---------|
| `title` | string | Browser tab title | "My Docs" |
| `url` | string | Production URL | "https://docs.example.com" |
| `baseUrl` | string | URL path | "/" or "/docs/" |
| `favicon` | string | Tab icon | "img/favicon.ico" |

### Optional Fields

```typescript
organizationName: 'github-username',    // For "Edit on GitHub"
projectName: 'repo-name',              // Repository name
onBrokenLinks: 'throw',                // Error on broken links
onBrokenMarkdownLinks: 'warn',         // Warn on markdown link issues
```

## Adding Content

### Create Documentation Files

Create markdown files in the `docs/` directory:

```bash
# Create a document
cat > docs/my-page.mdx << 'EOF'
---
id: my-page
title: My Page
sidebar_position: 1
---

# My Page

This is my documentation.
EOF
```

### Update Navigation Sidebar

Edit `sidebars.ts` to control document order:

```typescript
export const sidebars = {
  docs: [
    'intro',                    // First page
    'my-page',                 // Second page
    {
      type: 'category',
      label: 'Section Name',
      items: [
        'doc1',
        'doc2',
      ],
    },
  ],
};
```

## Customizing the Theme

### Change Colors

Edit `src/css/custom.css`:

```css
:root {
  --primary-color: #3b82f6;           /* Main brand color */
  --primary-dark: #1e40af;            /* Darker variant */
  --bg-primary: #0f172a;              /* Background color */
  --text-primary: #f1f5f9;            /* Text color */
}
```

### Add Navigation Links

Edit `docusaurus.config.ts`:

```typescript
themeConfig: {
  navbar: {
    items: [
      {
        to: '/docs/intro',
        label: 'Documentation',
        position: 'left',
      },
      {
        href: 'https://github.com/yourname/yourrepo',
        label: 'GitHub',
        position: 'right',
      },
    ],
  },
}
```

### Customize Footer

```typescript
footer: {
  style: 'dark',
  links: [
    {
      title: 'Docs',
      items: [
        {
          label: 'Getting Started',
          to: '/docs/intro',
        },
      ],
    },
  ],
  copyright: `Copyright © ${new Date().getFullYear()} Your Organization.`,
},
```

## Common Commands

### Development

```bash
pnpm run start              # Start dev server (with hot reload)
```

### Building

```bash
pnpm run build              # Production build
pnpm run serve              # Serve built site locally
```

### Maintenance

```bash
pnpm run clear              # Clear build cache
pnpm run typecheck          # Check TypeScript errors
```

## Troubleshooting

### Port 3000 Already in Use

```bash
# Use different port
pnpm start -- --port 3001
```

### Changes Not Reflecting

```bash
# Clear cache and restart
pnpm run clear
pnpm run start
```

### Build Fails with TypeScript Errors

```bash
# Check what went wrong
pnpm run typecheck

# Rebuild
pnpm run build
```

### Dependencies Not Resolving

```bash
# Reinstall everything
rm -rf node_modules pnpm-lock.yaml
pnpm install
pnpm run build
```

## Next Steps

1. **Add your documentation** - Create `.mdx` or `.md` files in `docs/`
2. **Customize navigation** - Update `sidebars.ts`
3. **Adjust theme** - Modify `src/css/custom.css` for branding
4. **Add content** - Write your documentation
5. **Deploy** - Use `docusaurus-deployer` skill to publish

## File Locations Reference

| File | Purpose |
|------|---------|
| `docusaurus.config.ts` | Main site configuration |
| `sidebars.ts` | Navigation structure |
| `src/css/custom.css` | Custom styling |
| `docs/` | Documentation files |
| `static/img/` | Images and assets |
| `package.json` | Dependencies |

## Resources

- [Docusaurus Official Documentation](https://docusaurus.io/docs)
- [Markdown Guide](https://www.markdownguide.org/)
- [MDX Documentation](https://mdxjs.com/)

For detailed theme customization, see `theming-guide.md`.
For common issues, see `troubleshooting-init.md`.
