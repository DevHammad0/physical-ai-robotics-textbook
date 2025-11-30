"""
Test database connection with detailed error reporting.
"""
import asyncio
import asyncpg
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from config import settings


async def test_connection():
    """Test database connection with detailed diagnostics."""
    print("=" * 60)
    print("Database Connection Test")
    print("=" * 60)
    
    db_url = settings.database_url
    
    # Parse and display URL components (masked)
    print(f"\nDatabase URL (masked):")
    if "@" in db_url:
        parts = db_url.split("@")
        if len(parts) == 2:
            user_part = parts[0]
            host_part = parts[1].split("/")[0].split("?")[0]
            
            if ":" in user_part:
                user_pass = user_part.split(":")
                if len(user_pass) >= 3:
                    user = user_pass[2]  # postgresql://user:pass
                    print(f"  User: {user}")
            
            print(f"  Host: {host_part}")
            print(f"  Full hostname: {host_part}")
            
            # Check if hostname looks complete
            if not host_part or host_part == "host":
                print("\n✗ ERROR: Hostname is placeholder or empty!")
                print("   Please update DATABASE_URL in .env with your actual Neon connection string")
                return False
            
            # Check if it ends properly
            if not host_part.endswith(".neon.tech") and not host_part.startswith("ep-"):
                print(f"\n⚠ WARNING: Hostname doesn't look like Neon format")
                print(f"   Expected: ep-xxxxx.region.aws.neon.tech")
                print(f"   Got: {host_part}")
    
    print(f"\nAttempting connection...")
    
    try:
        # Try connection with timeout
        print("  Connecting (timeout: 10s)...")
        conn = await asyncio.wait_for(
            asyncpg.connect(dsn=db_url),
            timeout=10.0
        )
        
        print("✓ Connected successfully!")
        
        # Test query
        version = await conn.fetchval("SELECT version()")
        print(f"✓ Database version: {version.split(',')[0]}")
        
        # Check if tables exist
        tables = await conn.fetch("""
            SELECT tablename FROM pg_tables 
            WHERE schemaname = 'public'
            ORDER BY tablename
        """)
        
        if tables:
            print(f"\n✓ Found {len(tables)} existing tables:")
            for table in tables:
                print(f"  - {table['tablename']}")
        else:
            print("\n✓ No tables found (database is empty - ready for initialization)")
        
        await conn.close()
        print("\n✓ Connection test passed!")
        return True
        
    except asyncio.TimeoutError:
        print("\n✗ Connection timeout (10s)")
        print("\nPossible causes:")
        print("  1. Network connectivity issue")
        print("  2. Neon project is paused (free tier pauses after 5 days idle)")
        print("  3. Firewall blocking connection")
        print("  4. Hostname is incorrect")
        print("\nAction: Check Neon dashboard and resume project if paused")
        return False
        
    except asyncpg.InvalidPasswordError:
        print("\n✗ Authentication failed")
        print("\nPossible causes:")
        print("  1. Password in connection string is incorrect")
        print("  2. Connection string is outdated")
        print("\nAction: Regenerate connection string in Neon dashboard")
        return False
        
    except asyncpg.InvalidCatalogNameError:
        print("\n✗ Database does not exist")
        print("\nAction: Check database name in connection string matches your Neon project")
        return False
        
    except OSError as e:
        if "getaddrinfo failed" in str(e) or e.errno == 11001 or e.errno == 11002:
            print("\n✗ DNS resolution failed")
            print(f"   Error: {e}")
            print("\nPossible causes:")
            print("  1. Hostname in DATABASE_URL is incorrect or incomplete")
            print("  2. Network DNS issues")
            print("  3. Neon project hostname changed")
            print("\nTroubleshooting steps:")
            print("  1. Go to Neon dashboard: https://console.neon.tech")
            print("  2. Select your project")
            print("  3. Click 'Connection Details'")
            print("  4. Copy the 'Pooled connection' string again")
            print("  5. Make sure you copy the ENTIRE string (it's long!)")
            print("  6. Update DATABASE_URL in .env")
            print("\nThe connection string should look like:")
            print("  postgresql://user:pass@ep-xxxxx-xxxxx.region.aws.neon.tech/dbname?sslmode=require")
            return False
        else:
            print(f"\n✗ Connection error: {e}")
            return False
            
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        print(f"   Error type: {type(e).__name__}")
        return False


if __name__ == "__main__":
    success = asyncio.run(test_connection())
    sys.exit(0 if success else 1)

