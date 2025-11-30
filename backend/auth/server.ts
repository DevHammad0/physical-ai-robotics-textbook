/**
 * Local development server for better-auth
 * Run this alongside the FastAPI backend for local development
 * 
 * Usage: npm start
 * Or: npx tsx server.ts
 */

// IMPORTANT: Load environment variables BEFORE importing any modules that use them
// First try auth/.env, then fall back to parent backend/.env
import { config } from 'dotenv';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';
import { existsSync } from 'fs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Try loading from auth/.env first
const authEnvPath = resolve(__dirname, '.env');
const parentEnvPath = resolve(__dirname, '..', '.env');

// Load from auth/.env if it exists, otherwise from parent .env
const envPath = existsSync(authEnvPath) ? authEnvPath : parentEnvPath;
const result = config({ path: envPath });

if (result.error) {
  console.error('✗ Error loading .env file:', result.error);
} else {
  console.log(`✓ Loaded .env from: ${envPath}`);
}

// Clean up DATABASE_URL - remove quotes if present
if (process.env.DATABASE_URL) {
  process.env.DATABASE_URL = process.env.DATABASE_URL.replace(/^['"]|['"]$/g, '');
}

// Verify DATABASE_URL is set
if (!process.env.DATABASE_URL) {
  console.error('✗ DATABASE_URL is not set!');
  console.error('  Please ensure DATABASE_URL is in backend/.env or backend/auth/.env');
  process.exit(1);
} else {
  // Log DATABASE_URL without exposing password
  const dbUrl = process.env.DATABASE_URL;
  const maskedUrl = dbUrl.replace(/:[^:@]+@/, ':****@');
  console.log(`✓ DATABASE_URL found: ${maskedUrl}`);
}

// Now import modules that depend on environment variables
import { createServer } from 'http';
import { auth } from './lib/auth.js';

// Optional: Verify schema on startup (set VERIFY_SCHEMA=true to enable)
const VERIFY_SCHEMA = process.env.VERIFY_SCHEMA === 'true';

if (VERIFY_SCHEMA) {
  console.log('Schema verification enabled - checking better-auth tables...');
  // Import and run verification (async, non-blocking)
  import('./scripts/verify-auth-schema.js').catch((err) => {
    console.error('Schema verification failed:', err);
    // Don't exit - let server start anyway
  });
}

const PORT = process.env.AUTH_PORT || 8001;
const CORS_ORIGIN = process.env.CORS_ORIGIN || 'http://localhost:3000';

const server = createServer(async (req, res) => {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Origin', CORS_ORIGIN);
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, PUT, DELETE');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, Cookie');
  res.setHeader('Access-Control-Allow-Credentials', 'true');

  // Handle preflight requests
  if (req.method === 'OPTIONS') {
    res.writeHead(200);
    res.end();
    return;
  }

  try {
    // Convert Node.js request to Web API Request
    const url = new URL(req.url || '/', `http://${req.headers.host}`);
    
    // Read request body if present
    let body: string | undefined = undefined;
    if (req.method !== 'GET' && req.method !== 'HEAD') {
      const chunks: Buffer[] = [];
      for await (const chunk of req) {
        chunks.push(chunk);
      }
      body = Buffer.concat(chunks).toString('utf-8');
    }

    // Build headers
    const headers = new Headers();
    Object.entries(req.headers).forEach(([key, value]) => {
      if (value) {
        headers.set(key.toLowerCase(), Array.isArray(value) ? value.join(', ') : String(value));
      }
    });

    // Create Web API Request
    const request = new Request(url, {
      method: req.method,
      headers,
      body,
    });

    // Handle auth requests
    let response: Response;
    try {
      response = await auth.handler(request);
    } catch (authError) {
      // Log detailed error information
      console.error('\n=== Better-Auth Handler Error ===');
      console.error('Method:', req.method);
      console.error('URL:', req.url);
      console.error('Error Type:', authError instanceof Error ? authError.constructor.name : typeof authError);
      console.error('Error Message:', authError instanceof Error ? authError.message : String(authError));
      
      if (authError instanceof Error && authError.stack) {
        console.error('Stack Trace:');
        console.error(authError.stack);
      }
      
      // Log request body if available
      if (body) {
        try {
          const bodyObj = JSON.parse(body);
          console.error('Request Body:', JSON.stringify(bodyObj, null, 2));
        } catch (e) {
          console.error('Request Body (raw):', body.substring(0, 500));
        }
      }
      
      console.error('===================================\n');
      
      // Return detailed error response for debugging
      res.writeHead(500, { 'Content-Type': 'application/json' });
      const errorResponse = {
        error: 'Internal server error',
        message: authError instanceof Error ? authError.message : 'Unknown error',
        type: authError instanceof Error ? authError.constructor.name : typeof authError,
        // Only include stack in development
        ...(process.env.NODE_ENV !== 'production' && authError instanceof Error && authError.stack
          ? { stack: authError.stack.split('\n').slice(0, 10) }
          : {})
      };
      res.end(JSON.stringify(errorResponse, null, 2));
      return;
    }
    
    // Copy response headers
    response.headers.forEach((value, key) => {
      res.setHeader(key, value);
    });

    // Set status and send response
    res.writeHead(response.status, response.statusText);
    const responseBody = await response.text();
    
    // Log non-2xx responses for debugging
    if (response.status >= 400) {
      console.error(`\n=== Auth Response Error (${response.status}) ===`);
      console.error('Method:', req.method);
      console.error('URL:', req.url);
      try {
        const errorBody = JSON.parse(responseBody);
        console.error('Error Response:', JSON.stringify(errorBody, null, 2));
      } catch (e) {
        console.error('Error Response (raw):', responseBody.substring(0, 500));
      }
      console.error('===================================\n');
    }
    
    res.end(responseBody);
  } catch (error) {
    console.error('\n=== Unexpected Server Error ===');
    console.error('Error Type:', error instanceof Error ? error.constructor.name : typeof error);
    console.error('Error Message:', error instanceof Error ? error.message : String(error));
    if (error instanceof Error && error.stack) {
      console.error('Stack Trace:');
      console.error(error.stack);
    }
    console.error('===================================\n');
    
    res.writeHead(500, { 'Content-Type': 'application/json' });
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    res.end(JSON.stringify({ error: 'Internal server error', message: errorMessage }));
  }
});

server.listen(PORT, () => {
  console.log(`\n✓ Better-auth server running on http://localhost:${PORT}`);
  console.log(`  Auth endpoints: http://localhost:${PORT}/api/auth/*`);
  console.log(`  CORS origin: ${CORS_ORIGIN}`);
  if (!VERIFY_SCHEMA) {
    console.log(`  Tip: Set VERIFY_SCHEMA=true to verify database schema on startup\n`);
  } else {
    console.log(`  Schema verification: enabled\n`);
  }
});

