"""
Quick diagnostic script to check environment variable configuration.
Run this to verify your .env file is set up correctly.
"""
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from config import settings
    
    print("=" * 60)
    print("Environment Variables Check")
    print("=" * 60)
    
    # Check each required variable
    checks = {
        "OpenAI API Key": ("openai_api_key", settings.openai_api_key),
        "OpenAI Model": ("openai_model", settings.openai_model),
        "Qdrant URL": ("qdrant_url", settings.qdrant_url),
        "Qdrant API Key": ("qdrant_api_key", settings.qdrant_api_key),
        "Database URL": ("database_url", settings.database_url),
    }
    
    all_ok = True
    
    for name, (key, value) in checks.items():
        if value:
            # Mask sensitive values
            if "key" in key.lower() or "password" in key.lower():
                display = f"{value[:10]}...{value[-4:]}" if len(value) > 14 else "***"
            elif key == "database_url":
                # Mask password in database URL
                if "@" in value:
                    parts = value.split("@")
                    if ":" in parts[0]:
                        user_pass = parts[0].split(":")
                        if len(user_pass) >= 3:
                            display = f"{user_pass[0]}:{user_pass[1]}:***@{parts[1][:30]}..."
                        else:
                            display = value[:50] + "..."
                    else:
                        display = value[:50] + "..."
                else:
                    display = value[:50] + "..."
            else:
                display = value[:50] + "..." if len(value) > 50 else value
            
            print(f"✓ {name}: {display}")
        else:
            print(f"✗ {name}: NOT SET")
            all_ok = False
    
    print("\n" + "=" * 60)
    
    if all_ok:
        print("✓ All environment variables are set")
        
        # Validate DATABASE_URL format
        db_url = settings.database_url
        if not db_url.startswith("postgresql://"):
            print("\n⚠ WARNING: DATABASE_URL should start with 'postgresql://'")
            print(f"   Current: {db_url[:50]}...")
        else:
            # Check for required components
            if "@" not in db_url:
                print("\n⚠ WARNING: DATABASE_URL missing '@' (host separator)")
            if "?" not in db_url and "sslmode" not in db_url:
                print("\n⚠ WARNING: DATABASE_URL should include '?sslmode=require' for Neon")
            
            # Try to parse hostname
            try:
                if "@" in db_url:
                    host_part = db_url.split("@")[1].split("/")[0].split("?")[0]
                    print(f"\n✓ Database hostname: {host_part}")
                    
                    # Check if it looks like a Neon URL
                    if "neon.tech" in host_part or "ep-" in host_part:
                        print("✓ Looks like a valid Neon Postgres URL")
                    else:
                        print("⚠ Hostname doesn't look like Neon format (should contain 'neon.tech' or 'ep-')")
            except Exception as e:
                print(f"\n⚠ Could not parse DATABASE_URL: {e}")
        
    else:
        print("✗ Some environment variables are missing")
        print("\nPlease check your .env file in the backend/ directory")
        print("Required variables:")
        print("  - OPENAI_API_KEY")
        print("  - QDRANT_URL")
        print("  - QDRANT_API_KEY")
        print("  - DATABASE_URL")
    
    print("=" * 60)
    
except Exception as e:
    print(f"\n✗ Error loading configuration: {e}")
    print("\nPossible issues:")
    print("  1. .env file doesn't exist in backend/ directory")
    print("  2. .env file has syntax errors")
    print("  3. Missing required environment variables")
    sys.exit(1)

