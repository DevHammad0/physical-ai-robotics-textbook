"""
Chat router implementing agent-based RAG pipeline for question answering.
"""
from fastapi import APIRouter, Depends, HTTPException
import time
import logging
import asyncpg

from models.schemas import ChatRequest, ChatResponse
from services.db_service import get_pool, AnalyticsService
from services.agent_service import AgentService
from services.session_service import PostgresSession
from config import settings

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    pool: asyncpg.Pool = Depends(get_pool)
):
    """
    Process chat query using OpenAI Agents SDK with autonomous tool usage.

    The agent autonomously decides when to search the textbook based on the query.

    Steps:
    1. Create PostgresSession for conversation persistence
    2. Run agent with query (agent decides tool usage)
    3. Extract structured sources from tool calls
    4. Log analytics
    5. Return response with sources
    """
    start_time = time.time()

    try:
        # Initialize services
        analytics_service = AnalyticsService(pool)

        # Create or load PostgresSession
        session = PostgresSession(
            conversation_id=request.conversation_id,
            session_id=request.session_id
        )

        # Run agent with session
        agent_result = await AgentService.run_agent(
            query=request.query,
            session=session,
            selected_text=request.selected_text
        )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Log analytics
        await analytics_service.log_query(
            query=request.query,
            response_time_ms=response_time_ms,
            success=True,
            session_id=request.session_id,
            conversation_id=session.conversation_id
        )

        # Return structured response with sources
        return ChatResponse(
            message=agent_result["message"],
            sources=agent_result["sources"],  # Structured sources extracted from tool calls
            conversation_id=session.conversation_id,
            response_time_ms=response_time_ms
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat endpoint error: {e}", exc_info=True)

        # Log failed query
        response_time_ms = int((time.time() - start_time) * 1000)
        try:
            analytics_service = AnalyticsService(pool)
            await analytics_service.log_query(
                query=request.query,
                response_time_ms=response_time_ms,
                success=False,
                session_id=request.session_id,
                error_message=str(e)
            )
        except:
            pass

        raise HTTPException(
            status_code=500,
            detail="An error occurred processing your request. Please try again."
        )
