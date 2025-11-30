"""
Auth Service - Helper functions for better-auth session verification
"""
import os
import logging
import httpx
from typing import Optional

logger = logging.getLogger(__name__)


async def get_user_id_from_better_auth_session(
    cookies: dict,
    authorization: Optional[str] = None
) -> Optional[str]:
    """
    Verify better-auth session and extract user ID.
    
    Args:
        cookies: Dictionary of cookies from request
        authorization: Optional authorization header
    
    Returns:
        User ID if session is valid, None otherwise
    """
    try:
        # Get the auth base URL from environment
        # For local dev, auth server runs on port 8001 (or proxied through 8000)
        # Check if we should use the proxy (8000) or direct (8001)
        auth_base_url = os.getenv("BETTER_AUTH_URL")
        if not auth_base_url:
            # Default to port 8001 for direct connection to auth server
            # If proxied through FastAPI, this will be overridden by BETTER_AUTH_URL env var
            auth_base_url = os.getenv("VERCEL_URL", "http://localhost:8001")
        if not auth_base_url.startswith("http"):
            auth_base_url = f"https://{auth_base_url}"
        
        # Build cookie header
        cookie_header = "; ".join([f"{k}={v}" for k, v in cookies.items()])
        
        # Make request to better-auth to verify session
        async with httpx.AsyncClient() as client:
            headers = {}
            if cookie_header:
                headers["Cookie"] = cookie_header
            if authorization:
                headers["Authorization"] = authorization
            
            response = await client.get(
                f"{auth_base_url}/api/auth/get-session",
                headers=headers,
                timeout=5.0,
                follow_redirects=True
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get("user") and data.get("user").get("id"):
                    return data["user"]["id"]
    except Exception as e:
        logger.error(f"Error verifying session with better-auth: {e}")
    
    return None

