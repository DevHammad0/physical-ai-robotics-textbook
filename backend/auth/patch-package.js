/**
 * Patch better-auth package.json to add kysely-adapter export
 * This is a workaround for local development
 */
import { readFileSync, writeFileSync } from 'fs';
import { join } from 'path';

const packageJsonPath = join(process.cwd(), 'node_modules', 'better-auth', 'package.json');
const packageJson = JSON.parse(readFileSync(packageJsonPath, 'utf-8'));

// Add kysely-adapter export if it doesn't exist
if (!packageJson.exports['./adapters/kysely-adapter']) {
  packageJson.exports['./adapters/kysely-adapter'] = {
    "dev-source": "./src/adapters/kysely-adapter/index.ts",
    "types": "./dist/adapters/kysely-adapter/index.d.mts",
    "default": "./dist/adapters/kysely-adapter/index.mjs"
  };
  
  writeFileSync(packageJsonPath, JSON.stringify(packageJson, null, 2));
  console.log('✓ Patched better-auth package.json to add kysely-adapter export');
} else {
  console.log('✓ kysely-adapter export already exists');
}

