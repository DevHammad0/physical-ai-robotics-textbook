# Environment Variables Setup Guide

## Where to Create .env File

You have **two options** for where to place the `.env` file:

### Option 1: `backend/.env` (Recommended)
Create the file at: `backend/.env`

**Pros:**
- Shared with FastAPI backend
- Single file to manage
- Already exists in your project

### Option 2: `backend/auth/.env`
Create the file at: `backend/auth/.env`

**Pros:**
- Auth-specific configuration
- Isolated from backend config

**Note:** The auth server checks `backend/auth/.env` first, then falls back to `backend/.env` if not found.

---

## Required Environment Variables

### 1. `DATABASE_URL` (REQUIRED)
PostgreSQL connection string for your database.

**Format:**
```env
DATABASE_URL=postgresql://username:password@host:port/database?sslmode=require
```

**Example (Neon PostgreSQL):**
```env
DATABASE_URL=postgresql://neondb_owner:your_password@ep-summer-river-adw751ek-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
```

**Example (Local PostgreSQL):**
```env
DATABASE_URL=postgresql://postgres:password@localhost:5432/physical_ai_db
```

**Important:** 
- Do NOT wrap the value in quotes (the code automatically removes them)
- For Neon/cloud databases, include `?sslmode=require`

---

### 2. `BETTER_AUTH_SECRET` (OPTIONAL but RECOMMENDED)
Secret key for encrypting sessions and tokens. **Generate a random string for production!**

**Format:**
```env
BETTER_AUTH_SECRET=your-random-secret-key-here
```

**How to generate:**
```bash
# Using Node.js
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"

# Using OpenSSL
openssl rand -hex 32

# Using Python
python -c "import secrets; print(secrets.token_hex(32))"
```

**Default:** `"change-this-secret-in-production"` (NOT secure for production!)

---

## Optional Environment Variables

### 3. `BETTER_AUTH_URL` (OPTIONAL)
Base URL where the auth API is accessible.

**For Local Development:**
```env
BETTER_AUTH_URL=http://localhost:8001
```

**For Production:**
```env
BETTER_AUTH_URL=https://your-domain.com
```

**Default:** 
- `http://localhost:8001` (local)
- Uses `VERCEL_URL` if set (production)

---

### 4. `CORS_ORIGIN` (OPTIONAL)
Allowed origin for CORS requests (your frontend URL).

**For Local Development:**
```env
CORS_ORIGIN=http://localhost:3000
```

**For Production:**
```env
CORS_ORIGIN=https://your-frontend-domain.com
```

**Default:** `http://localhost:3000`

---

### 5. `AUTH_PORT` (OPTIONAL)
Port for the auth server to run on.

```env
AUTH_PORT=8001
```

**Default:** `8001`

---

### 6. `VERIFY_SCHEMA` (OPTIONAL)
Enable schema verification on server startup.

```env
VERIFY_SCHEMA=true
```

**Default:** `false` (disabled)

---

## Complete .env File Example

### For Local Development (`backend/.env`):

```env
# Database Connection (REQUIRED)
DATABASE_URL=postgresql://neondb_owner:your_password@ep-summer-river-adw751ek-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# Better Auth Configuration
BETTER_AUTH_SECRET=your-generated-secret-key-here-min-32-chars
BETTER_AUTH_URL=http://localhost:8001
CORS_ORIGIN=http://localhost:3000

# Server Configuration
AUTH_PORT=8001

# Optional: Schema Verification
VERIFY_SCHEMA=false
```

### For Production:

```env
# Database Connection
DATABASE_URL=postgresql://user:pass@host:5432/db?sslmode=require

# Better Auth Configuration
BETTER_AUTH_SECRET=your-production-secret-key-min-32-chars
BETTER_AUTH_URL=https://your-backend-domain.com
CORS_ORIGIN=https://your-frontend-domain.com

# Vercel (if deploying to Vercel)
VERCEL_URL=your-backend.vercel.app
```

---

## Quick Setup Steps

1. **Create `.env` file:**
   ```bash
   # Option 1: In backend/ directory
   cd backend
   touch .env
   
   # Option 2: In backend/auth/ directory
   cd backend/auth
   touch .env
   ```

2. **Add required variables:**
   ```bash
   # Copy this template and fill in your values
   echo 'DATABASE_URL=postgresql://user:pass@host:port/db?sslmode=require' >> .env
   echo 'BETTER_AUTH_SECRET=your-secret-here' >> .env
   ```

3. **Generate a secure secret:**
   ```bash
   # Generate and add to .env
   node -e "console.log('BETTER_AUTH_SECRET=' + require('crypto').randomBytes(32).toString('hex'))" >> .env
   ```

4. **Verify setup:**
   ```bash
   cd backend/auth
   npm start
   # Should show: ✓ DATABASE_URL found
   ```

---

## Testing Checklist

Before testing authentication:

- [ ] `.env` file created in `backend/` or `backend/auth/`
- [ ] `DATABASE_URL` is set and valid
- [ ] `BETTER_AUTH_SECRET` is set (generate a secure one!)
- [ ] Database tables are created (run `npm run fix-tables` if needed)
- [ ] Auth server starts without errors (`npm start` in `backend/auth/`)
- [ ] FastAPI backend is running on port 8000
- [ ] Docusaurus frontend is running on port 3000

---

## Troubleshooting

### "DATABASE_URL is not set!"
- Check that `.env` file exists in `backend/` or `backend/auth/`
- Verify the file has `DATABASE_URL=` line
- Make sure there are no quotes around the value

### "relation 'user' does not exist"
- Run: `cd backend/auth && npm run fix-tables`
- This will create all required tables

### "ECONNREFUSED" errors
- Verify `DATABASE_URL` is correct
- Check database is accessible
- For Neon: ensure SSL mode is set (`?sslmode=require`)

### CORS errors
- Set `CORS_ORIGIN` to match your frontend URL
- Default is `http://localhost:3000` for local dev

---

## Security Notes

1. **Never commit `.env` files to git** (already in `.gitignore`)
2. **Use strong secrets in production** (min 32 characters)
3. **Rotate secrets periodically**
4. **Use different secrets for dev/staging/production**
5. **Keep `DATABASE_URL` secure** (contains credentials)

