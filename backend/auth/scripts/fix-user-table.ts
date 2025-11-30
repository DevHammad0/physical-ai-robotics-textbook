/**
 * Fix user table column names to match better-auth expectations
 * PostgreSQL converts unquoted identifiers to lowercase, so we need to fix this
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

async function fixUserTable() {
  console.log('\n=== Fixing User Table Column Names ===\n');
  
  if (!process.env.DATABASE_URL) {
    console.error('✗ DATABASE_URL is not set!');
    process.exit(1);
  }
  
  const maskedUrl = process.env.DATABASE_URL.replace(/:[^:@]+@/, ':****@');
  console.log(`Database: ${maskedUrl}\n`);
  
  const client = await pool.connect();
  
  try {
    // Check current columns
    const columnsResult = await client.query(`
      SELECT column_name 
      FROM information_schema.columns 
      WHERE table_schema = 'public' 
      AND table_name = 'user'
      ORDER BY ordinal_position
    `);
    
    const currentColumns = columnsResult.rows.map(r => r.column_name);
    console.log('Current columns:', currentColumns.join(', '));
    console.log('');
    
    // Drop and recreate the table with correct column names (quoted identifiers)
    console.log('Dropping existing user table...');
    await client.query('DROP TABLE IF EXISTS "user" CASCADE');
    
    console.log('Creating user table with correct column names...');
    await client.query(`
      CREATE TABLE "user" (
        "id" TEXT PRIMARY KEY,
        "email" TEXT UNIQUE NOT NULL,
        "emailVerified" BOOLEAN NOT NULL DEFAULT false,
        "name" TEXT,
        "image" TEXT,
        "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
        "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW(),
        "physical_ai_experience" INTEGER CHECK ("physical_ai_experience" >= 0 AND "physical_ai_experience" <= 10)
      )
    `);
    console.log('✓ Created user table with correct column names');
    
    // Recreate session table with correct foreign key
    console.log('Recreating session table...');
    await client.query('DROP TABLE IF EXISTS session CASCADE');
    await client.query(`
      CREATE TABLE session (
        "id" TEXT PRIMARY KEY,
        "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
        "expiresAt" TIMESTAMP NOT NULL,
        "token" TEXT UNIQUE NOT NULL,
        "ipAddress" TEXT,
        "userAgent" TEXT,
        "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
        "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
      )
    `);
    console.log('✓ Created session table');
    
    // Recreate account table
    console.log('Recreating account table...');
    await client.query('DROP TABLE IF EXISTS account CASCADE');
    await client.query(`
      CREATE TABLE account (
        "id" TEXT PRIMARY KEY,
        "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
        "accountId" TEXT NOT NULL,
        "providerId" TEXT NOT NULL,
        "accessToken" TEXT,
        "refreshToken" TEXT,
        "idToken" TEXT,
        "accessTokenExpiresAt" TIMESTAMP,
        "refreshTokenExpiresAt" TIMESTAMP,
        "scope" TEXT,
        "password" TEXT,
        "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
        "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW(),
        UNIQUE("providerId", "accountId")
      )
    `);
    console.log('✓ Created account table');
    
    // Recreate verification table
    console.log('Recreating verification table...');
    await client.query('DROP TABLE IF EXISTS verification CASCADE');
    await client.query(`
      CREATE TABLE verification (
        "id" TEXT PRIMARY KEY,
        "identifier" TEXT NOT NULL,
        "value" TEXT NOT NULL,
        "expiresAt" TIMESTAMP NOT NULL,
        "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
        "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
      )
    `);
    console.log('✓ Created verification table');
    
    console.log('\n=== User Table Fixed ===\n');
    
  } catch (error) {
    console.error('\n✗ Error fixing tables:', error);
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

// Run fix
fixUserTable().catch((error) => {
  console.error('Fatal error:', error);
  process.exit(1);
});

