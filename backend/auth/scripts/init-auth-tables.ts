/**
 * Initialize better-auth database tables
 * Creates all required tables for better-auth if they don't exist
 */

// Load environment variables FIRST
import { config } from 'dotenv';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';
import { existsSync } from 'fs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Try loading from auth/.env first, then fall back to parent backend/.env
const authEnvPath = resolve(__dirname, '..', '.env');
const parentEnvPath = resolve(__dirname, '..', '..', '.env');

const envPath = existsSync(authEnvPath) ? authEnvPath : parentEnvPath;
config({ path: envPath });

// Clean up DATABASE_URL - remove quotes if present
if (process.env.DATABASE_URL) {
  process.env.DATABASE_URL = process.env.DATABASE_URL.replace(/^['"]|['"]$/g, '');
}

import { Pool } from 'pg';

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  max: 1,
});

async function initTables() {
  console.log('\n=== Initializing Better-Auth Tables ===\n');
  
  if (!process.env.DATABASE_URL) {
    console.error('✗ DATABASE_URL is not set!');
    process.exit(1);
  }
  
  const maskedUrl = process.env.DATABASE_URL.replace(/:[^:@]+@/, ':****@');
  console.log(`Database: ${maskedUrl}\n`);
  
  const client = await pool.connect();
  
  try {
    // Create user table
    await client.query(`
      CREATE TABLE IF NOT EXISTS "user" (
        id TEXT PRIMARY KEY,
        email TEXT UNIQUE NOT NULL,
        emailVerified BOOLEAN NOT NULL DEFAULT false,
        name TEXT,
        image TEXT,
        createdAt TIMESTAMP NOT NULL DEFAULT NOW(),
        updatedAt TIMESTAMP NOT NULL DEFAULT NOW(),
        physical_ai_experience INTEGER CHECK (physical_ai_experience >= 0 AND physical_ai_experience <= 10)
      )
    `);
    console.log('✓ Created user table');
    
    // Create session table
    await client.query(`
      CREATE TABLE IF NOT EXISTS session (
        id TEXT PRIMARY KEY,
        userId TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
        expiresAt TIMESTAMP NOT NULL,
        token TEXT UNIQUE NOT NULL,
        ipAddress TEXT,
        userAgent TEXT,
        createdAt TIMESTAMP NOT NULL DEFAULT NOW(),
        updatedAt TIMESTAMP NOT NULL DEFAULT NOW()
      )
    `);
    console.log('✓ Created session table');
    
    // Create account table
    await client.query(`
      CREATE TABLE IF NOT EXISTS account (
        id TEXT PRIMARY KEY,
        userId TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
        accountId TEXT NOT NULL,
        providerId TEXT NOT NULL,
        accessToken TEXT,
        refreshToken TEXT,
        idToken TEXT,
        accessTokenExpiresAt TIMESTAMP,
        refreshTokenExpiresAt TIMESTAMP,
        scope TEXT,
        password TEXT,
        createdAt TIMESTAMP NOT NULL DEFAULT NOW(),
        updatedAt TIMESTAMP NOT NULL DEFAULT NOW(),
        UNIQUE(providerId, accountId)
      )
    `);
    console.log('✓ Created account table');
    
    // Create verification table
    await client.query(`
      CREATE TABLE IF NOT EXISTS verification (
        id TEXT PRIMARY KEY,
        identifier TEXT NOT NULL,
        value TEXT NOT NULL,
        expiresAt TIMESTAMP NOT NULL,
        createdAt TIMESTAMP NOT NULL DEFAULT NOW(),
        updatedAt TIMESTAMP NOT NULL DEFAULT NOW()
      )
    `);
    console.log('✓ Created verification table');
    
    // Check if physical_ai_experience column exists, add if not
    const columnCheck = await client.query(
      `SELECT EXISTS (
        SELECT FROM information_schema.columns 
        WHERE table_schema = 'public' 
        AND table_name = 'user' 
        AND column_name = 'physical_ai_experience'
      )`
    );
    
    if (!columnCheck.rows[0].exists) {
      await client.query(`
        ALTER TABLE "user" 
        ADD COLUMN physical_ai_experience INTEGER 
        CHECK (physical_ai_experience >= 0 AND physical_ai_experience <= 10)
      `);
      console.log('✓ Added physical_ai_experience column to user table');
    } else {
      console.log('✓ Column physical_ai_experience already exists');
    }
    
    console.log('\n=== Better-Auth Tables Initialized ===\n');
    
  } catch (error) {
    console.error('\n✗ Error initializing tables:', error);
    if (error instanceof Error) {
      console.error('Message:', error.message);
      if (error.stack) {
        console.error('Stack:', error.stack);
      }
    }
    process.exit(1);
  } finally {
    client.release();
    await pool.end();
  }
}

// Run initialization
initTables().catch((error) => {
  console.error('Fatal error:', error);
  process.exit(1);
});

