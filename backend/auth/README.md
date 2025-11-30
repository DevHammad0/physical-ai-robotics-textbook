# Better-Auth Service

This directory contains the better-auth authentication service.

## Local Development Setup

1. **Install dependencies:**
```bash
cd backend/auth
npm install
```

2. **Set environment variables:**
   
   Create a `.env` file in either:
   - `backend/.env` (recommended - shared with FastAPI)
   - `backend/auth/.env` (auth-specific)
   
   The auth server checks `backend/auth/.env` first, then falls back to `backend/.env`.
   
   **Required variables:**
   ```bash
   DATABASE_URL=postgresql://user:password@host:port/dbname?sslmode=require
   ```
   
   **Recommended variables:**
   ```bash
   BETTER_AUTH_SECRET=your-secret-key-here  # Generate: node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
   BETTER_AUTH_URL=http://localhost:8001    # For local dev
   CORS_ORIGIN=http://localhost:3000        # Docusaurus dev server
   ```
   
   **Optional variables:**
   ```bash
   AUTH_PORT=8001                            # Default: 8001
   VERIFY_SCHEMA=false                      # Enable schema verification on startup
   ```
   
   See `ENV_SETUP.md` for detailed setup instructions.

3. **Start the auth server:**
```bash
npm start
# Or for auto-reload:
npm run start:dev
```

The auth server will run on `http://localhost:8001` and handle all `/api/auth/*` requests.

## Production Deployment (Vercel)

The auth service is deployed as a Vercel serverless function. The route is configured in `backend/vercel.json`:

```json
{
  "src": "/api/auth/(.*)",
  "dest": "auth/api/auth/[...all].ts"
}
```

Set these environment variables in Vercel:
- `BETTER_AUTH_SECRET` - Secret key for better-auth
- `BETTER_AUTH_URL` - Base URL where auth API is accessible
- `DATABASE_URL` - PostgreSQL connection string

## Database Schema

Better-auth will automatically create its required tables. The `users` table should have:
- `physical_ai_experience` field (integer 0-10) - configured in `lib/auth.ts`

## API Endpoints

All better-auth endpoints are available under `/api/auth/*`:
- `POST /api/auth/sign-up/email` - Sign up with email/password
- `POST /api/auth/sign-in/email` - Sign in with email/password
- `GET /api/auth/get-session` - Get current session
- `POST /api/auth/sign-out` - Sign out

See [better-auth documentation](https://www.better-auth.com/docs) for full API reference.

## Running Both Servers Locally

For local development, you need to run both:

1. **FastAPI backend** (port 8000):
```bash
cd backend
uvicorn main:app --reload --port 8000
```

2. **Auth server** (port 8001):
```bash
cd backend/auth
npm start
```

3. **Docusaurus frontend** (port 3000):
```bash
cd book-source
pnpm start
```

The frontend is configured to use `http://localhost:8001` for auth requests when running locally.
