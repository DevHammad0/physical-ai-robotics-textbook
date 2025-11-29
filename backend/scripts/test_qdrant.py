"""
Test Qdrant Cloud connection with detailed diagnostics.
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from config import settings
from qdrant_client import AsyncQdrantClient
import httpx


async def test_qdrant():
    """Test Qdrant connection with detailed error reporting."""
    print("=" * 60)
    print("Qdrant Cloud Connection Test")
    print("=" * 60)
    
    qdrant_url = settings.qdrant_url
    qdrant_api_key = settings.qdrant_api_key
    
    # Display masked URL
    if qdrant_url:
        print(f"\nQdrant URL: {qdrant_url[:50]}...")
    else:
        print("\n✗ QDRANT_URL not set in .env")
        return False
    
    if not qdrant_api_key:
        print("\n✗ QDRANT_API_KEY not set in .env")
        return False
    
    print(f"API Key: {qdrant_api_key[:10]}...{qdrant_api_key[-4:]}")
    
    # Validate URL format
    if not qdrant_url.startswith("https://"):
        print("\n⚠ WARNING: QDRANT_URL should start with 'https://'")
        print(f"   Current: {qdrant_url[:50]}...")
    
    print("\nAttempting connection...")
    
    try:
        # Create client - Qdrant client handles timeout internally
        # We'll use default timeout and catch timeout errors
        print("  Creating client...")
        
        client = AsyncQdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
        
        print("  Testing connection...")
        # Test connection by getting collections
        collections = await asyncio.wait_for(
            client.get_collections(),
            timeout=30.0
        )
        
        print("✓ Connected successfully!")
        print(f"\n✓ Found {len(collections.collections)} collection(s):")
        for coll in collections.collections:
            print(f"  - {coll.name}")
        
        await client.close()
        print("\n✓ Qdrant connection test passed!")
        return True
        
    except asyncio.TimeoutError:
        print("\n✗ Connection timeout (30s)")
        print("\nPossible causes:")
        print("  1. Network connectivity issue")
        print("  2. Qdrant Cloud cluster is paused or unavailable")
        print("  3. Firewall/proxy blocking connection")
        print("  4. Incorrect QDRANT_URL")
        print("\nTroubleshooting:")
        print("  1. Check Qdrant Cloud dashboard: https://cloud.qdrant.io")
        print("  2. Verify cluster is active (not paused)")
        print("  3. Check QDRANT_URL is correct (should be full URL with https://)")
        print("  4. Try accessing the URL in browser to verify it's reachable")
        return False
        
    except Exception as e:
        error_msg = str(e)
        error_type = type(e).__name__
        print(f"\n✗ Connection failed: {error_type}")
        print(f"   Error message: {error_msg[:200]}")
        
        # Check for underlying exception
        if hasattr(e, '__cause__') and e.__cause__:
            cause = e.__cause__
            print(f"\n   Underlying error: {type(cause).__name__}: {str(cause)[:200]}")
        
        if "ConnectTimeout" in error_msg or "timeout" in error_msg.lower() or "Timeout" in error_type:
            print("\n⚠ This is a connection timeout. Possible causes:")
            print("  1. Network connectivity issue - check your internet connection")
            print("  2. Qdrant cluster is paused - check dashboard: https://cloud.qdrant.io")
            print("  3. Firewall/proxy blocking HTTPS connections")
            print("  4. Incorrect QDRANT_URL format")
            print("\nTroubleshooting steps:")
            print("  1. Open Qdrant dashboard: https://cloud.qdrant.io")
            print("  2. Check if your cluster is 'Active' (not paused)")
            print("  3. Verify QDRANT_URL matches your cluster URL exactly")
            print("  4. Try accessing the URL in browser (should show Qdrant API info)")
        elif "401" in error_msg or "Unauthorized" in error_msg:
            print("\n⚠ Authentication failed. Check your QDRANT_API_KEY")
            print("  1. Go to Qdrant dashboard → Your cluster → API Keys")
            print("  2. Verify the API key is correct")
            print("  3. Regenerate if needed")
        elif "404" in error_msg or "Not Found" in error_msg:
            print("\n⚠ URL not found. Check your QDRANT_URL")
            print("  1. Verify URL format: https://xxx.cloud.qdrant.io")
            print("  2. Check cluster name matches in dashboard")
        else:
            print(f"\n⚠ Unexpected error. Try these steps:")
            print("  1. Verify QDRANT_URL and QDRANT_API_KEY in .env")
            print("  2. Check Qdrant dashboard for cluster status")
            print("  3. Test basic connectivity: ping the hostname")
        
        return False


if __name__ == "__main__":
    success = asyncio.run(test_qdrant())
    sys.exit(0 if success else 1)

