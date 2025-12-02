"""
ChatKit router - Handles ChatKit communication using openai-chatkit SDK.
Provides the main /chatkit endpoint for streaming chat responses.
"""
import logging
from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse, Response

from services.chatkit_server import PhysicalAIChatKitServer
from services.chatkit_store import PostgresStore, InMemoryStore
from chatkit.server import StreamingResult

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize ChatKit server with PostgreSQL-backed store
# Fallback to in-memory store if database not available
try:
    chatkit_store = PostgresStore()
    logger.info("[ChatKit] Using PostgreSQL-backed store")
except Exception as e:
    logger.warning(f"[ChatKit] Failed to initialize PostgreSQL store, using in-memory: {e}")
    chatkit_store = InMemoryStore()

chatkit_server = PhysicalAIChatKitServer(data_store=chatkit_store)


@router.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    Handle POST requests for ChatKit endpoint.
    OPTIONS preflight requests are handled automatically by FastAPI's CORS middleware
    with wildcard headers (allow_credentials=False allows this).
    """
    try:
        # Log everything about the request
        logger.info(f"[ChatKit] === NEW REQUEST ===")
        logger.info(f"[ChatKit] Method: {request.method}")
        logger.info(f"[ChatKit] Headers: {dict(request.headers)}")

        # Get request body
        body = await request.body()
        logger.info(f"[ChatKit] Body size: {len(body)} bytes")

        # Try to parse body as JSON for better logging
        try:
            import json
            body_json = json.loads(body)
            logger.info(f"[ChatKit] Body JSON: {json.dumps(body_json, indent=2)}")
        except:
            logger.info(f"[ChatKit] Body (raw): {body[:500]}")

        # Optional: Extract context from headers and query params (user ID, session info, selected text, etc.)
        selected_text = request.headers.get("X-Selected-Text") or request.query_params.get("selected_text")
        context = {
            "user_id": request.headers.get("X-User-ID"),
            "session_id": request.headers.get("X-Session-ID"),
            "selected_text": selected_text,
        }
        logger.info(f"[ChatKit] Context: {context}")
        if selected_text:
            logger.info(f"[ChatKit] Selected text detected (length: {len(selected_text)}): {selected_text[:100]}...")

        # Process request through ChatKit server
        logger.info("[ChatKit] Calling chatkit_server.process()...")
        result = await chatkit_server.process(body, context)
        logger.info(f"[ChatKit] Result type: {type(result).__name__}")
        logger.info(f"[ChatKit] Is StreamingResult: {isinstance(result, StreamingResult)}")

        # Return appropriate response based on result type
        # CORS headers are handled automatically by FastAPI's CORSMiddleware
        if isinstance(result, StreamingResult):
            logger.info("[ChatKit] Returning StreamingResponse")
            return StreamingResponse(
                result,
                media_type="text/event-stream",
                headers={
                    "Cache-Control": "no-cache",
                    "X-Accel-Buffering": "no",
                }
            )
        else:
            logger.info("[ChatKit] Returning JSON Response")
            return Response(
                content=result.json,
                media_type="application/json"
            )

    except Exception as e:
        logger.error(f"[ChatKit] === ERROR ===")
        logger.error(f"[ChatKit] Error type: {type(e).__name__}")
        logger.error(f"[ChatKit] Error message: {str(e)}")
        logger.error(f"[ChatKit] Traceback:", exc_info=True)

        # CORS headers are handled automatically by FastAPI's CORSMiddleware
        return Response(
            content=f'{{"error": "{str(e)}"}}',
            media_type="application/json",
            status_code=500
        )

