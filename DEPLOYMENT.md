# Production Deployment Guide

This guide covers deploying all components of the Physical AI & Robotics Textbook to production.

## Architecture Overview

- **Frontend (Docusaurus)**: Deployed to GitHub Pages
- **Backend (FastAPI)**: Deployed to Vercel (port 8000 equivalent)
- **Auth Service (Better Auth)**: Deployed to Vercel as serverless function (integrated with backend)

## 1. Docusaurus Site → GitHub Pages

### Automatic Deployment (Recommended)

The site automatically deploys to GitHub Pages when you push to the `main` branch.

**Workflow**: `.github/workflows/deploy-docusaurus.yml`

**What it does:**
- Triggers on push to `main` branch (only when `book-source/` changes)
- Builds the Docusaurus site
- Deploys to GitHub Pages

**URL**: `https://devhammad0.github.io/physical-ai-robotics-textbook`

### Manual Deployment

If you need to deploy manually:

```bash
cd book-source
pnpm install
pnpm build
# Then use GitHub CLI or web interface to deploy the build folder
```

### Enable GitHub Pages

1. Go to your repository settings
2. Navigate to **Pages** section
3. Set source to **GitHub Actions**
4. The workflow will handle the rest

## 2. Backend + Auth Service → Vercel

Both the FastAPI backend and Better Auth service deploy together to Vercel.

### Prerequisites

1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Login to Vercel:
```bash
vercel login
```

### Environment Variables

Set these in Vercel dashboard (Project Settings → Environment Variables):

**Required:**
- `DATABASE_URL` - PostgreSQL connection string
- `BETTER_AUTH_SECRET` - Secret key for better-auth (generate with: `node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"`)
- `BETTER_AUTH_URL` - Your Vercel backend URL (e.g., `https://your-project.vercel.app`)
- `CORS_ORIGIN` - Your GitHub Pages URL (e.g., `https://devhammad0.github.io`)

**Optional (for FastAPI backend):**
- `OPENAI_API_KEY` - For RAG chatbot functionality
- `QDRANT_URL` - Vector database URL
- `QDRANT_API_KEY` - Vector database API key

### Deployment Steps

1. **Navigate to backend directory:**
```bash
cd backend
```

2. **Deploy to production:**
```bash
vercel --prod
```

This will:
- Deploy FastAPI backend (handles `/api/*` routes except `/api/auth/*`)
- Deploy Better Auth service (handles `/api/auth/*` routes)
- Both run as serverless functions on the same domain

### Verify Deployment

After deployment, check:
- Backend health: `https://your-project.vercel.app/api/health`
- Auth endpoint: `https://your-project.vercel.app/api/auth/session`

## 3. Update Frontend Configuration

After deploying the backend, update the frontend to use the production backend URL.

### Option 1: Environment Variable (Recommended)

Add to your Docusaurus build process or GitHub Actions:

```bash
# In GitHub Actions workflow, add:
env:
  NEXT_PUBLIC_BACKEND_URL: https://your-project.vercel.app
```

### Option 2: Runtime Configuration

Update `book-source/src/lib/api-config.ts` to detect production:

```typescript
// Already configured to use origin for production
// Just ensure your GitHub Pages site knows the backend URL
```

You can also set `window.__BACKEND_URL__` in your HTML if needed.

## 4. CORS Configuration

Ensure your backend allows requests from GitHub Pages:

In `backend/config.py` or environment variables:
```python
CORS_ORIGINS = [
    "https://devhammad0.github.io",
    "http://localhost:3000",  # For local dev
]
```

## 5. Database Setup

### Production Database

1. Set up a PostgreSQL database (e.g., Neon, Supabase, Railway)
2. Run migrations:
```bash
cd backend
# Run schema.sql to create tables
# Better Auth will auto-create its tables on first request
```

3. Index your content:
```bash
python scripts/index_content.py
```

## Deployment Checklist

- [ ] GitHub Pages enabled in repository settings
- [ ] GitHub Actions workflow runs successfully
- [ ] Vercel project created and linked
- [ ] All environment variables set in Vercel
- [ ] Backend deployed with `vercel --prod`
- [ ] Database migrations run
- [ ] Content indexed in vector database
- [ ] CORS configured for GitHub Pages domain
- [ ] Frontend API configuration updated
- [ ] Test authentication flow
- [ ] Test RAG chatbot functionality

## Troubleshooting

### GitHub Pages not updating
- Check GitHub Actions workflow status
- Verify `gh-pages` branch is created
- Check repository Pages settings

### Backend not responding
- Check Vercel deployment logs
- Verify environment variables are set
- Check database connection
- Verify CORS configuration

### Auth service not working
- Verify `BETTER_AUTH_URL` matches your Vercel domain
- Check that `/api/auth/*` routes are accessible
- Verify database schema is created
- Check Vercel function logs

### CORS errors
- Ensure `CORS_ORIGIN` includes your GitHub Pages URL
- Check that credentials are allowed
- Verify preflight requests are handled

## URLs Summary

After deployment:
- **Frontend**: `https://devhammad0.github.io/physical-ai-robotics-textbook`
- **Backend API**: `https://your-project.vercel.app/api/*`
- **Auth API**: `https://your-project.vercel.app/api/auth/*`


