# Custom Theme Guide for Robotics Documentation

Customize Docusaurus theming for optimal readability and branding.

## CSS Architecture

Docusaurus uses CSS variables for theming. Customize in `src/css/custom.css`.

### Color System

Define color palette at the top of `custom.css`:

```css
:root {
  /* Primary Colors */
  --primary: #3b82f6;              /* Main brand color (blue) */
  --primary-dark: #1e40af;         /* Darker variant */
  --primary-light: #60a5fa;        /* Lighter variant */

  /* Status Colors */
  --success: #10b981;              /* Green (for success/correct) */
  --warning: #f59e0b;              /* Amber (for cautions) */
  --error: #ef4444;                /* Red (for errors/danger) */
  --info: #06b6d4;                 /* Cyan (for information) */

  /* Dark Mode Background */
  --bg-primary: #0f172a;           /* Main background */
  --bg-secondary: #1e293b;         /* Secondary elements */
  --bg-tertiary: #334155;          /* Subtle background */

  /* Text Colors */
  --text-primary: #f1f5f9;         /* Main text */
  --text-secondary: #cbd5e1;       /* Secondary text */
  --text-muted: #94a3b8;           /* Muted/meta text */

  /* Code/Technical */
  --code-bg: #1e293b;              /* Code block background */
  --code-text: #e2e8f0;            /* Code text color */
}
```

### Applying Colors in CSS

Use CSS variables throughout `custom.css`:

```css
body {
  background-color: var(--bg-primary);
  color: var(--text-primary);
}

a {
  color: var(--primary-light);
}

a:hover {
  color: var(--primary-color);
}

code {
  background-color: var(--code-bg);
  color: var(--code-text);
}
```

## Typography Configuration

### Font Selection

For technical/robotics content, use:
- **Body**: Clear sans-serif (e.g., Inter, Segoe UI)
- **Code**: Monospace (e.g., Fira Code, JetBrains Mono)
- **Headings**: Bold sans-serif (same as body)

Import fonts in `custom.css`:

```css
@import url('https://fonts.googleapis.com/css2?family=Fira+Code:wght@400;500;700&family=Inter:wght@400;500;600;700&display=swap');

body {
  font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
  line-height: 1.6;                /* Readable line height */
  font-size: 16px;                 /* Base font size */
}

code {
  font-family: 'Fira Code', monospace;
  font-size: 0.9em;                /* Slightly smaller */
  font-weight: 500;                /* Medium weight for readability */
}

h1, h2, h3, h4, h5, h6 {
  font-weight: 600;                /* Bold headings */
  line-height: 1.3;                /* Tighter heading lines */
}
```

### Heading Styles

Style heading hierarchy:

```css
h1 {
  font-size: 2.5rem;
  margin-bottom: 1.5rem;
  border-bottom: 3px solid var(--primary-color);
  padding-bottom: 0.5rem;
}

h2 {
  font-size: 2rem;
  margin-top: 2rem;
  margin-bottom: 1rem;
}

h3 {
  font-size: 1.5rem;
  margin-top: 1.5rem;
  color: var(--primary-light);
}

h4 {
  font-size: 1.2rem;
  font-weight: 600;
  margin-top: 1rem;
}
```

## Code Block Styling

### Syntax Highlighting Theme

Configure in `docusaurus.config.ts`:

```typescript
themeConfig: {
  prism: {
    theme: require('prism-react-renderer/themes/dracula'),
    darkTheme: require('prism-react-renderer/themes/dracula'),
    additionalLanguages: ['python', 'bash', 'cpp'],
  },
}
```

Available Prism themes:
- `dracula` - Dark, high contrast (recommended)
- `github-dark` - GitHub dark theme
- `nord` - Arctic, north-bluish color palette
- `palenight` - Pale night theme

### Custom Code Block Styling

Add to `custom.css`:

```css
.docusaurus-highlight {
  background-color: var(--code-bg);
  border: 1px solid var(--bg-tertiary);
  border-radius: 8px;
  margin: 1rem 0;
}

.docusaurus-highlight pre {
  padding: 1rem;
  overflow-x: auto;
  font-size: 0.95em;
  line-height: 1.5;
}

/* Language badges */
.codeBlockTitle {
  background-color: var(--bg-tertiary);
  color: var(--text-secondary);
  padding: 0.5rem 1rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.85em;
  border-radius: 8px 8px 0 0;
}

/* Token colors for specific syntax elements */
.token.keyword {
  color: #c792ea;  /* Purple */
  font-weight: 600;
}

.token.string {
  color: #a1e21a;  /* Green */
}

.token.number {
  color: #f8b500;  /* Amber */
}

.token.operator {
  color: #6db3f2;  /* Blue */
}

.token.comment {
  color: #888888;  /* Gray */
  font-style: italic;
}
```

## Component Styling

### Tables

Style tables for data presentation:

```css
table {
  width: 100%;
  border-collapse: collapse;
  margin: 1.5rem 0;
  background-color: var(--bg-secondary);
  border: 1px solid var(--bg-tertiary);
  border-radius: 8px;
}

thead {
  background-color: var(--primary-dark);
  color: white;
}

th {
  padding: 0.75rem 1rem;
  text-align: left;
  font-weight: 600;
  border-bottom: 2px solid var(--bg-tertiary);
}

td {
  padding: 0.75rem 1rem;
  border-bottom: 1px solid var(--bg-tertiary);
}

tbody tr:hover {
  background-color: var(--bg-tertiary);
}
```

### Lists

Customize list appearance:

```css
ul, ol {
  margin: 1rem 0;
  padding-left: 2rem;
  color: var(--text-secondary);
}

li {
  margin: 0.5rem 0;
  line-height: 1.7;
}

ul > li::marker {
  color: var(--primary-color);
  font-weight: bold;
}

ol > li::marker {
  color: var(--primary-color);
  font-weight: bold;
}
```

### Admonitions/Callouts

Style note, warning, and info boxes:

```css
.admonition {
  padding: 1rem;
  margin: 1.5rem 0;
  border-radius: 8px;
  border-left: 4px solid;
  background-color: var(--bg-secondary);
}

.admonition.note {
  border-color: var(--info);
}

.admonition.tip {
  border-color: var(--success);
}

.admonition.warning {
  border-color: var(--warning);
}

.admonition.danger {
  border-color: var(--error);
}

.admonition-heading {
  font-weight: 600;
  margin-bottom: 0.5rem;
  color: var(--text-primary);
}
```

### Images

Style image appearance:

```css
img {
  max-width: 100%;
  height: auto;
  border-radius: 8px;
  margin: 1.5rem 0;
  border: 1px solid var(--bg-tertiary);
  transition: border-color 0.2s ease;
}

img:hover {
  border-color: var(--primary-color);
}
```

## Navigation Styling

### Sidebar

Customize sidebar appearance:

```css
.sidebar {
  background-color: var(--bg-secondary);
  border-right: 1px solid var(--bg-tertiary);
}

.menu__link {
  color: var(--text-secondary);
  border-radius: 6px;
  transition: all 0.2s ease;
}

.menu__link:hover {
  color: var(--primary-light);
  background-color: var(--bg-tertiary);
}

.menu__link--active {
  color: var(--primary-color);
  background-color: var(--bg-tertiary);
  border-left: 3px solid var(--primary-color);
  font-weight: 600;
}
```

### Navbar

Customize top navigation:

```css
.navbar {
  background-color: var(--bg-secondary);
  border-bottom: 1px solid var(--bg-tertiary);
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
}

.navbar__item, .navbar__link {
  color: var(--text-secondary);
  transition: color 0.2s ease;
}

.navbar__link:hover, .navbar__item:hover {
  color: var(--primary-light);
}

.navbar__link--active {
  color: var(--primary-color);
  border-bottom: 2px solid var(--primary-color);
}
```

## Dark/Light Mode Support

### Conditional Colors

Support both dark and light modes:

```css
/* Dark mode (default) */
:root {
  --bg-primary: #0f172a;
  --text-primary: #f1f5f9;
}

/* Light mode */
html[data-theme="light"] {
  --bg-primary: #ffffff;
  --bg-secondary: #f8fafc;
  --text-primary: #0f172a;
  --text-secondary: #475569;
  --code-bg: #f1f5f9;
  --code-text: #1e293b;
}
```

### Disable Dark Mode Toggle

To enforce dark mode only:

```typescript
// In docusaurus.config.ts
themeConfig: {
  colorMode: {
    defaultMode: 'dark',
    disableSwitch: true,           // Hide the toggle
    respectPrefersColorScheme: false,
  },
}
```

## Responsive Design

### Mobile Optimization

Ensure good readability on mobile:

```css
@media (max-width: 768px) {
  h1 {
    font-size: 2rem;
  }

  h2 {
    font-size: 1.5rem;
  }

  body {
    font-size: 14px;
  }

  .docusaurus-highlight pre {
    font-size: 0.85em;
    padding: 0.75rem;
  }

  table {
    font-size: 0.9em;
  }

  th, td {
    padding: 0.5rem 0.75rem;
  }
}
```

## Print Styles

Optimize for printing:

```css
@media print {
  :root {
    --bg-primary: white;
    --text-primary: black;
    --code-bg: #f5f5f5;
  }

  body {
    background-color: white;
    color: black;
  }

  a {
    color: #0066cc;
  }

  .navbar, .sidebar, .DocSearch {
    display: none;
  }

  code {
    background-color: #f5f5f5;
  }
}
```

## Custom Components (Optional)

### Circuit Diagram Component

Create `src/components/CircuitDiagram.tsx`:

```typescript
import React from 'react';

export default function CircuitDiagram({ circuitData }) {
  return (
    <div style={{
      padding: '1rem',
      backgroundColor: 'var(--bg-secondary)',
      borderRadius: '8px',
      border: '1px solid var(--bg-tertiary)',
      margin: '1.5rem 0',
    }}>
      {/* SVG or diagram content */}
      <svg width="100%" viewBox="0 0 400 300">
        {/* Circuit elements */}
      </svg>
    </div>
  );
}
```

Use in MDX:

```mdx
<CircuitDiagram circuitData={{ /* ... */ }} />
```

## Common Customization Examples

### Robotics-Themed Colors

```css
:root {
  --primary: #e74c3c;              /* Red (robot danger) */
  --primary-dark: #c0392b;
  --success: #2ecc71;              /* Green (safe/normal) */
  --warning: #f39c12;              /* Amber (caution) */
}
```

### Tech Company Style

```css
:root {
  --primary: #0066cc;              /* Blue */
  --bg-primary: #0a0e27;           /* Very dark */
  --text-primary: #ffffff;
  font-family: 'SF Pro Display', sans-serif;
}
```

### Academic/Educational

```css
:root {
  --primary: #003d82;              /* Deep blue */
  --success: #00701a;              /* Deep green */
  font-family: 'Georgia', serif;   /* Serif for academic feel */
}
```

## Testing Your Theme

Preview changes during development:

```bash
pnpm run start
```

The site will hot-reload as you modify `custom.css`.

## Troubleshooting

**Colors not applying**: Clear cache and rebuild
```bash
pnpm run clear
pnpm run start
```

**Dark mode not working**: Verify CSS variables are defined in `:root`

**Print style issues**: Test with browser print preview (Ctrl+P or Cmd+P)

## Resources

- [Docusaurus CSS Variables Reference](https://docusaurus.io/docs/styling-layout)
- [Prism Themes](https://github.com/FormidableLabs/prism-react-renderer)
- [Web.dev - Color](https://web.dev/color/)
- [MDN - CSS Custom Properties](https://developer.mozilla.org/en-US/docs/Web/CSS/--*)
