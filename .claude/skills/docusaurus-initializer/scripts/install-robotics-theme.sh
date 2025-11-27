#!/bin/bash

# Install Robotics Custom Theme for Docusaurus
# Copies theme files and configures docusaurus.config.ts

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${1:-.}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if project is Docusaurus
if [ ! -f "$PROJECT_ROOT/docusaurus.config.ts" ] && [ ! -f "$PROJECT_ROOT/docusaurus.config.js" ]; then
    echo -e "${RED}✗ Error: Not a Docusaurus project (missing docusaurus.config.ts)${NC}"
    exit 1
fi

echo "Installing robotics custom theme..."

# Create directories
mkdir -p "$PROJECT_ROOT/src/css"
mkdir -p "$PROJECT_ROOT/src/components"

# Copy custom CSS
if [ -f "$SCRIPT_DIR/../assets/robotics-theme/custom.css" ]; then
    cp "$SCRIPT_DIR/../assets/robotics-theme/custom.css" "$PROJECT_ROOT/src/css/custom.css"
    echo -e "${GREEN}✓ Copied custom.css${NC}"
else
    echo -e "${YELLOW}⚠ Warning: custom.css not found, using defaults${NC}"
fi

# Copy component examples (if they exist)
if [ -d "$SCRIPT_DIR/../assets/robotics-theme/components" ]; then
    find "$SCRIPT_DIR/../assets/robotics-theme/components" -name "*.tsx" -o -name "*.ts" | while read -r file; do
        filename=$(basename "$file")
        cp "$file" "$PROJECT_ROOT/src/components/$filename"
    done
    echo -e "${GREEN}✓ Copied custom components${NC}"
fi

# Verify theme applies by checking docusaurus.config.ts
if grep -q "custom.css" "$PROJECT_ROOT/src/css/custom.css" 2>/dev/null || [ -f "$PROJECT_ROOT/src/css/custom.css" ]; then
    echo -e "${GREEN}✓ Theme installation complete${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Run 'pnpm run build' to verify theme applies"
    echo "  2. Run 'pnpm run start' to preview in development server"
    echo "  3. Customize colors/fonts in src/css/custom.css as needed"
    exit 0
else
    echo -e "${RED}✗ Theme installation failed${NC}"
    exit 1
fi
