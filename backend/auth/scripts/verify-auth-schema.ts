/**
 * Verify and initialize better-auth database schema
 * Checks if better-auth tables exist and adds missing columns
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
  max: 1, // Single connection for verification
});

interface TableInfo {
  exists: boolean;
  columns?: string[];
}

async function checkTable(tableName: string): Promise<TableInfo> {
  const client = await pool.connect();
  try {
    // Check if table exists
    const tableCheck = await client.query(
      `SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' 
        AND table_name = $1
      )`,
      [tableName]
    );
    
    const exists = tableCheck.rows[0].exists;
    
    if (!exists) {
      return { exists: false };
    }
    
    // Get column names
    const columnsResult = await client.query(
      `SELECT column_name, data_type 
       FROM information_schema.columns 
       WHERE table_schema = 'public' 
       AND table_name = $1
       ORDER BY ordinal_position`,
      [tableName]
    );
    
    const columns = columnsResult.rows.map(row => row.column_name);
    
    return { exists: true, columns };
  } finally {
    client.release();
  }
}

async function addPhysicalAiExperienceColumn() {
  const client = await pool.connect();
  try {
    // First check if table exists
    const tableCheck = await client.query(
      `SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' 
        AND table_name = 'user'
      )`
    );
    
    if (!tableCheck.rows[0].exists) {
      console.log('⚠ user table does not exist yet - better-auth will create it on first use');
      console.log('  The physical_ai_experience column should be added automatically by better-auth');
      return;
    }
    
    // Check if column already exists
    const columnCheck = await client.query(
      `SELECT EXISTS (
        SELECT FROM information_schema.columns 
        WHERE table_schema = 'public' 
        AND table_name = 'user' 
        AND column_name = 'physical_ai_experience'
      )`
    );
    
    if (columnCheck.rows[0].exists) {
      console.log('✓ Column physical_ai_experience already exists in user table');
      return;
    }
    
    // Add the column with proper type and constraint
    await client.query(`
      ALTER TABLE "user" 
      ADD COLUMN physical_ai_experience INTEGER 
      CHECK (physical_ai_experience >= 0 AND physical_ai_experience <= 10)
    `);
    
    console.log('✓ Added physical_ai_experience column to user table');
  } catch (error) {
    if (error instanceof Error) {
      // If table doesn't exist, better-auth will create it
      if (error.message.includes('does not exist') || error.message.includes('relation "user"')) {
        console.log('⚠ user table does not exist yet - better-auth will create it on first use');
        return;
      }
      // If column already exists (race condition), that's fine
      if (error.message.includes('already exists') || error.message.includes('duplicate')) {
        console.log('✓ Column physical_ai_experience already exists (checked after creation)');
        return;
      }
    }
    throw error;
  } finally {
    client.release();
  }
}

async function verifySchema() {
  console.log('\n=== Verifying Better-Auth Schema ===\n');
  
  if (!process.env.DATABASE_URL) {
    console.error('✗ DATABASE_URL is not set!');
    process.exit(1);
  }
  
  const maskedUrl = process.env.DATABASE_URL.replace(/:[^:@]+@/, ':****@');
  console.log(`Database: ${maskedUrl}\n`);
  
  try {
    // Check better-auth tables
    const tables = ['user', 'session', 'account', 'verification'];
    const tableStatus: Record<string, TableInfo> = {};
    
    for (const table of tables) {
      const info = await checkTable(table);
      tableStatus[table] = info;
      
      if (info.exists) {
        console.log(`✓ Table "${table}" exists`);
        if (info.columns) {
          console.log(`  Columns: ${info.columns.join(', ')}`);
        }
      } else {
        console.log(`⚠ Table "${table}" does not exist`);
      }
    }
    
    console.log('');
    
    // Always try to add physical_ai_experience column if user table exists
    // This handles the case where better-auth created the table but didn't add the column
    if (tableStatus.user?.exists) {
      await addPhysicalAiExperienceColumn();
    } else {
      console.log('⚠ user table does not exist - better-auth will create it on first use');
      console.log('  Note: If better-auth creates the table without the column, run this script again');
      console.log('  Or set VERIFY_SCHEMA=true to auto-verify on server startup');
    }
    
    console.log('\n=== Schema Verification Complete ===\n');
    
  } catch (error) {
    console.error('\n✗ Error verifying schema:', error);
    if (error instanceof Error) {
      console.error('Message:', error.message);
      if (error.stack) {
        console.error('Stack:', error.stack);
      }
    }
    process.exit(1);
  } finally {
    await pool.end();
  }
}

// Run verification
verifySchema().catch((error) => {
  console.error('Fatal error:', error);
  process.exit(1);
});

