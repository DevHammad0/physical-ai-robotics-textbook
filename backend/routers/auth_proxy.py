"""
Auth Proxy Router - Proxies /api/auth/* requests to the better-auth server on port 8001.
This allows all auth requests to go through FastAPI on port 8000.
"""
import logging
import httpx
from fastapi import APIRouter, Request, Response
import os

router = APIRouter()
logger = logging.getLogger(__name__)

# Auth server URL - runs on port 8001 locally
AUTH_SERVER_URL = os.getenv("AUTH_SERVER_URL", "http://localhost:8001")


@router.api_route("/auth/{path:path}", methods=["GET", "POST", "PUT", "DELETE", "PATCH", "HEAD"])
async def proxy_auth(request: Request, path: str):
    """
    Proxy all /api/auth/* requests to the better-auth server.
    
    Preserves:
    - Request method
    - Headers (especially cookies and authorization)
    - Request body
    - Query parameters
    """
    try:
        # Build target URL
        target_url = f"{AUTH_SERVER_URL}/api/auth/{path}"
        
        # Add query parameters if present
        if request.url.query:
            target_url += f"?{request.url.query}"
        
        logger.debug(f"Proxying {request.method} {request.url.path} to {target_url}")
        
        # Get request body if present
        body = None
        if request.method in ["POST", "PUT", "PATCH"]:
            body = await request.body()
        
        # Prepare headers to forward
        headers = {}
        for key, value in request.headers.items():
            # Skip hop-by-hop headers that shouldn't be forwarded
            if key.lower() not in [
                "host",
                "content-length",  # Will be recalculated
                "connection",
                "keep-alive",
                "transfer-encoding",
            ]:
                headers[key] = value
        
        # Ensure origin header is preserved (important for better-auth origin validation)
        # If origin is not present, use the referer or construct from request
        if "origin" not in headers and "referer" in headers:
            # Extract origin from referer if available
            try:
                from urllib.parse import urlparse
                referer = headers.get("referer", "")
                if referer:
                    parsed = urlparse(referer)
                    origin = f"{parsed.scheme}://{parsed.netloc}"
                    headers["origin"] = origin
            except Exception:
                pass
        
        # Make request to auth server
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.request(
                method=request.method,
                url=target_url,
                headers=headers,
                content=body,
                follow_redirects=False,
            )
            
            # Prepare response headers
            response_headers = {}
            for key, value in response.headers.items():
                # Skip hop-by-hop headers
                if key.lower() not in [
                    "content-encoding",
                    "transfer-encoding",
                    "connection",
                    "keep-alive",
                ]:
                    response_headers[key] = value
            
            # Return response with same status code and headers
            return Response(
                content=response.content,
                status_code=response.status_code,
                headers=response_headers,
                media_type=response.headers.get("content-type", "application/json"),
            )
    
    except httpx.RequestError as e:
        logger.error(f"Error proxying auth request: {e}")
        return Response(
            content='{"error": "Auth server unavailable"}',
            status_code=503,
            media_type="application/json",
        )
    except Exception as e:
        logger.error(f"Unexpected error in auth proxy: {e}", exc_info=True)
        return Response(
            content=f'{{"error": "Internal server error: {str(e)}"}}',
            status_code=500,
            media_type="application/json",
        )

