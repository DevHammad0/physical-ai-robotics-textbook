# Troubleshooting Guide: Docusaurus Initialization

Common issues and solutions for setting up Docusaurus with pnpm and TypeScript.

## Installation Issues

### Issue: pnpm Command Not Found

**Error Message**:
```
pnpm: command not found
```

**Cause**: pnpm is not installed globally or not in system PATH

**Solution**:

```bash
# Install pnpm globally using npm (which comes with Node.js)
npm install -g pnpm

# Verify installation
pnpm --version

# If still not working, try with full path
/usr/local/bin/pnpm --version
```

**For Windows**:
- Download installer from https://pnpm.io/installation
- Or use PowerShell:
  ```powershell
  iwr https://get.pnpm.io -useb | iex
  ```

### Issue: Node.js Version Too Old

**Error Message**:
```
error: Node.js version 14.x.x is not supported
```

**Cause**: Docusaurus requires Node.js 18+

**Solution**:

```bash
# Check current version
node --version

# Update Node.js
# Using nvm (Node Version Manager)
nvm install 18
nvm use 18

# Or download from https://nodejs.org (LTS version)
```

### Issue: Disk Space or Permission Error

**Error Message**:
```
ENOSPC: no space left on device
Error: EACCES: permission denied
```

**Cause**: Insufficient disk space or permission issues

**Solution**:

```bash
# Check disk space
df -h

# Clear pnpm cache if needed
pnpm store prune

# For permission issues, don't use sudo
# Instead, fix npm/pnpm permissions:
mkdir ~/.npm-global
npm config set prefix '~/.npm-global'
export PATH=~/.npm-global/bin:$PATH
```

---

## Project Initialization Issues

### Issue: Scaffolding Fails

**Error Message**:
```
create-docusaurus: ERR! ERR! Error:
```

**Cause**: Network issue, corrupted cache, or invalid template

**Solution**:

```bash
# Clear cache
pnpm store prune
npm cache clean --force

# Try again with explicit template
pnpm create docusaurus@latest my-website classic --typescript

# If still failing, try with npm instead
npm init docusaurus@latest my-website classic -- --typescript
```

### Issue: TypeScript Configuration Error

**Error Message**:
```
error TS2307: Cannot find module '@docusaurus/types'
```

**Cause**: Dependencies not installed or TypeScript not recognizing paths

**Solution**:

```bash
# Reinstall dependencies
rm -rf node_modules pnpm-lock.yaml
pnpm install

# Verify TypeScript installation
pnpm list typescript

# Run type check
pnpm run typecheck
```

### Issue: Port Already in Use

**Error Message**:
```
Error: listen EADDRINUSE: address already in use :::3000
```

**Cause**: Another application is using port 3000

**Solution**:

```bash
# Find and kill process on port 3000
# On macOS/Linux
lsof -ti:3000 | xargs kill -9

# On Windows (PowerShell)
Get-Process | Where-Object {$_.Ports -eq 3000} | Stop-Process -Force

# Or use a different port
pnpm start -- --port 3001
```

---

## Build Issues

### Issue: Build Fails with TypeScript Errors

**Error Message**:
```
error TS2693: 'React' cannot be used as a JSX component
error TS2322: Type 'X' is not assignable to type 'Y'
```

**Cause**: TypeScript strict mode issues or missing imports

**Solution**:

```bash
# Check for errors
pnpm run typecheck

# Fix common issues
# Add React import if needed:
import React from 'react';

# Or configure JSX in tsconfig.json
{
  "compilerOptions": {
    "jsx": "react-jsx"  // For React 17+
  }
}

# Clear and rebuild
pnpm run clear
pnpm run build
```

### Issue: MDX/Markdown Syntax Error

**Error Message**:
```
error Unescaped left brace in regex is deprecated
error Cannot compile MDX file
```

**Cause**: Invalid MDX syntax or unescaped characters

**Solution**:

```mdx
// WRONG: Unescaped braces
const obj = { key: 'value' }

// CORRECT: Escape with extra braces or wrap in code block
const obj = { { key: 'value' } }

// Or use code block (better for code)
\```javascript
const obj = { key: 'value' }
\```

// Escape special characters
\* This is not italic \*
\[ This is not a link \]
```

### Issue: Build Takes Too Long (> 60 seconds)

**Cause**: Large codebase, many images, or system resources

**Solution**:

```bash
# Optimize images
pnpm add sharp  # For image optimization
# Then use in src/components as needed

# Enable cache
pnpm run build --cache=true

# Check what's slow
pnpm run build --debug

# Increase Node.js memory
NODE_OPTIONS=--max-old-space-size=4096 pnpm run build
```

---

## Development Server Issues

### Issue: Changes Not Reflecting

**Error Message**:
Hot reload not working, changes not visible

**Cause**: Cache issue or incorrect file path

**Solution**:

```bash
# Clear cache and restart
pnpm run clear
pnpm start

# Check file was saved properly
ls -la docs/

# Verify file path in sidebar
cat sidebars.ts  # Should reference your file
```

### Issue: Asset Images Not Loading

**Error Message**:
```
Failed to load resource: the server responded with a status of 404 (Not Found)
```

**Cause**: Image path incorrect or file in wrong location

**Solution**:

```markdown
// If image is in static/ folder:
![alt text](/img/my-image.png)

// If image is in same directory:
![alt text](./my-image.png)

// File structure should be:
static/
└── img/
    └── my-image.png

// Or in chapter directory:
docs/
└── chapters/
    └── 01-intro/
        ├── index.mdx
        └── my-image.png
```

---

## Theme & Styling Issues

### Issue: Dark Mode Not Working

**Error Message**: Colors not applying or toggle missing

**Cause**: CSS variables not defined or theme config issue

**Solution**:

```css
/* Check src/css/custom.css has root variables */
:root {
  --ifm-color-primary: #3b82f6;
  --bg-primary: #0f172a;
  /* ... more variables ... */
}

/* Verify dark mode in docusaurus.config.ts */
themeConfig: {
  colorMode: {
    defaultMode: 'dark',
    disableSwitch: false,
  },
}
```

### Issue: Custom CSS Not Applied

**Error Message**: Styles not visible even after editing custom.css

**Cause**: Cache issue or CSS selector too weak

**Solution**:

```bash
# Clear cache
pnpm run clear
pnpm start

# Check CSS specificity
# If not working, add !important temporarily for debugging
body {
  background-color: #0f172a !important;
}

# Then remove !important after confirming it works
```

### Issue: Code Block Colors Wrong

**Error Message**: Syntax highlighting looks wrong or hard to read

**Cause**: Theme mismatch or missing language

**Solution**:

```typescript
// In docusaurus.config.ts, verify theme:
themeConfig: {
  prism: {
    theme: require('prism-react-renderer/themes/dracula'),
    darkTheme: require('prism-react-renderer/themes/dracula'),
    additionalLanguages: ['python', 'bash', 'cpp'],
  },
}

// Clear and rebuild
pnpm run clear && pnpm start
```

---

## Navigation & Sidebar Issues

### Issue: Sidebar Not Showing Content

**Error Message**: Files not appearing in sidebar, even though they exist

**Cause**: File not referenced in sidebars.ts

**Solution**:

```typescript
// In sidebars.ts, explicitly reference the file:
export const sidebars = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Chapters',
      items: [
        'chapters/01-introduction/index',  // Include the doc ID
      ],
    },
  ],
};

// Note: Use the doc ID from frontmatter, not file path
// File: docs/chapters/01-introduction/index.mdx
// ID in frontmatter: id: chapter-1
// Reference: 'chapters/01-introduction/index' (path) or use ID if configured
```

### Issue: Broken Navigation Links

**Error Message**: Links in sidebar 404

**Cause**: File deleted, wrong path, or ID mismatch

**Solution**:

```bash
# Verify file exists
ls -la docs/chapters/

# Check ID in frontmatter matches sidebar reference
grep "^id:" docs/chapters/*/index.mdx

# Validate with type check
pnpm run typecheck
```

---

## Search Issues

### Issue: Search Not Working

**Error Message**: Search button disabled or no results

**Cause**: Search not configured

**Solution**:

```typescript
// Option 1: Use Algolia (requires credentials)
themeConfig: {
  algolia: {
    appId: 'YOUR_APP_ID',
    apiKey: 'YOUR_API_KEY',
    indexName: 'YOUR_INDEX_NAME',
  },
}

// Option 2: Use local search (no setup needed)
plugins: [
  [
    require.resolve("@easyops-cn/docusaurus-search-local"),
    {
      hashed: true,
    },
  ],
],
```

### Issue: Algolia Indexing Fails

**Cause**: Invalid credentials or index not created

**Solution**:

1. Verify Algolia account at https://docsearch.algolia.com
2. Check app ID and API key are correct
3. Ensure index exists in Algolia dashboard
4. Rebuild to re-index: `pnpm run build`

---

## Deployment Issues

### Issue: GitHub Pages 404 Error

**Error Message**: Site returns 404 even though it deployed

**Cause**: baseUrl configuration incorrect

**Solution**:

```typescript
// For user/org pages (https://username.github.io):
url: 'https://username.github.io',
baseUrl: '/',

// For project pages (https://username.github.io/project-name):
url: 'https://username.github.io',
baseUrl: '/project-name/',

// Clear and rebuild
pnpm run clear
pnpm run build

// Verify build/index.html exists
ls -la build/index.html
```

---

## Performance Issues

### Issue: Site Is Slow

**Cause**: Large images, slow network, or heavy components

**Solution**:

```bash
# Optimize images (reduce file size)
pnpm add imagemin-cli
# Then optimize images before adding to static/

# Check bundle size
pnpm run build --analyze

# Enable caching
NODE_OPTIONS=--max-old-space-size=4096 pnpm run build
```

---

## Getting Help

### Useful Diagnostic Commands

```bash
# Check environment
node --version
pnpm --version
git --version

# Verify project structure
ls -la

# Check for errors in build
pnpm run build 2>&1 | grep -i error

# Full type check output
pnpm run typecheck --noEmit

# Check for broken links
pnpm run build -- --validate
```

### Reporting Issues

If you encounter an issue not listed here:

1. **Search existing issues**:
   - https://github.com/facebook/docusaurus/issues
   - Look for similar error messages

2. **Check documentation**:
   - https://docusaurus.io/docs
   - https://docusaurus.io/docs/troubleshooting

3. **Enable verbose logging**:
   ```bash
   pnpm run start -- --verbose
   pnpm run build -- --verbose
   ```

4. **Provide debugging info**:
   - Node.js version: `node --version`
   - pnpm version: `pnpm --version`
   - OS: macOS/Linux/Windows
   - Full error message
   - What you were trying to do

---

## Quick Fixes Checklist

When something breaks:

- [ ] Clear cache: `pnpm run clear`
- [ ] Reinstall: `rm node_modules && pnpm install`
- [ ] Check Node.js version: `node --version` (should be 18+)
- [ ] Verify file paths in `sidebars.ts`
- [ ] Run type check: `pnpm run typecheck`
- [ ] Restart dev server: Stop and run `pnpm start` again
- [ ] Check browser console for errors (F12)
- [ ] Try different browser
- [ ] Restart computer (if all else fails)

---

## Common Error Messages Reference

| Error | Meaning | Fix |
|-------|---------|-----|
| `EADDRINUSE` | Port in use | Use different port or kill process |
| `ENOENT` | File not found | Check file path |
| `ERR NOFOUND` | Dependency missing | Run `pnpm install` |
| `TS2307` | Module not found | Reinstall `node_modules` |
| `Cannot find module` | Import issue | Check exact path and spelling |
| `Unescaped left brace` | MDX syntax error | Escape braces or use code block |

---

## When All Else Fails

Start fresh:

```bash
# Backup your content
cp -r docs/ docs.backup/

# Remove everything and reinstall
rm -rf node_modules pnpm-lock.yaml

# Reinstall from scratch
pnpm install

# Rebuild
pnpm run clear
pnpm run build

# Restart dev server
pnpm start
```

Your `docs/` folder is always safe - the node_modules and cache can be rebuilt.
