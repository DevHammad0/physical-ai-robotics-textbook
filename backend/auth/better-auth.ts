/**
 * Better-auth configuration file for CLI tools
 * This file is used by the Better Auth CLI to detect configuration
 * and run migrations.
 */

// Load environment variables FIRST
import { config } from 'dotenv';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';
import { existsSync } from 'fs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Try loading from auth/.env first, then fall back to parent backend/.env
const authEnvPath = resolve(__dirname, '.env');
const parentEnvPath = resolve(__dirname, '..', '.env');

const envPath = existsSync(authEnvPath) ? authEnvPath : parentEnvPath;
config({ path: envPath });

// Clean up DATABASE_URL - remove quotes if present
if (process.env.DATABASE_URL) {
  process.env.DATABASE_URL = process.env.DATABASE_URL.replace(/^['"]|['"]$/g, '');
}

// Import database and auth setup
import { betterAuth } from "better-auth";
import { kyselyAdapter } from "better-auth/adapters/kysely-adapter";
import { db } from "./lib/db.js";

// Export the auth instance - CLI will use this to detect configuration
export const auth = betterAuth({
  database: kyselyAdapter(db, {
    type: "postgres", // Explicitly specify PostgreSQL for CLI detection
  }),
  baseURL: process.env.BETTER_AUTH_URL || (process.env.VERCEL_URL 
    ? `https://${process.env.VERCEL_URL}` 
    : "http://localhost:8001"),
  basePath: "/api/auth",
  secret: process.env.BETTER_AUTH_SECRET || "change-this-secret-in-production",
  trustedOrigins: [
    "http://localhost:3000", // Docusaurus frontend
    "http://localhost:8000", // FastAPI backend (for proxy requests)
    "https://devhammad0.github.io", // GitHub Pages production
    process.env.CORS_ORIGIN || "http://localhost:3000",
    ...(process.env.CORS_ORIGINS ? process.env.CORS_ORIGINS.split(",").map((origin: string) => origin.trim()) : []),
    ...(process.env.VERCEL_URL ? [`https://${process.env.VERCEL_URL}`] : []),
  ],
  user: {
    additionalFields: {
      physical_ai_experience: {
        type: "number",
        required: true,
        input: true, // Allow user to provide during signup
      },
    },
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },
});
