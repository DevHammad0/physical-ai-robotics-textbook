"""
Agent Service - OpenAI Agents SDK integration for RAG chatbot
Defines the teaching assistant agent with search_textbook tool
"""

import os
import json
from agents import Agent, function_tool, Runner, RunContextWrapper, set_tracing_disabled
from typing import List, Dict, Any, Optional, AsyncIterator, TYPE_CHECKING
from openai.types.responses import ResponseTextDeltaEvent
from services.vector_search import VectorSearchService
from services.llm_service import LLMService
from config import settings

if TYPE_CHECKING:
    from services.session_service import PostgresSession

# Disable tracing
set_tracing_disabled(True)

# Set OpenAI API key for Agents SDK (it reads from OPENAI_API_KEY env var)
# Only set if not already set (allows override via environment)
if not os.getenv("OPENAI_API_KEY"):
    os.environ["OPENAI_API_KEY"] = settings.openai_api_key

# Global variable to track search results for source extraction
_last_search_results: List[Any] = []


@function_tool
async def search_textbook(ctx: RunContextWrapper[Any], query: str) -> str:
    """
    Search the Physical AI & Robotics textbook for relevant information.

    Args:
        query: The search query to find relevant textbook content

    Returns:
        Formatted search results with chapter, lesson, and content information
    """
    global _last_search_results

    # Initialize services
    vector_service = VectorSearchService(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name
    )

    llm_service = LLMService(
        api_key=settings.openai_api_key,
        model=settings.openai_embedding_model
    )

    # Generate embedding for search query
    query_vector = await llm_service.generate_embedding(query)

    # Perform vector search
    results = await vector_service.search(
        query_vector=query_vector,
        top_k=5
    )

    # Store results for source extraction
    _last_search_results = results

    # Format results for agent
    if not results:
        return "No relevant content found in the textbook."

    formatted_results = []
    for i, result in enumerate(results, 1):
        # Results from vector_search are dicts, not objects
        chapter = result.get('chapter', 'Unknown')
        lesson = result.get('lesson', 'Unknown')
        heading = result.get('heading', 'N/A')
        content = result.get('content', '')
        score = result.get('score', 0.0)
        
        formatted_results.append(
            f"{i}. [{chapter} - {lesson}]\n"
            f"   Section: {heading}\n"
            f"   Content: {content}\n"
            f"   Relevance: {score:.2f}\n"
        )

    return "\n".join(formatted_results)


# Define the teaching assistant agent
teaching_agent = Agent(
    name="Physical AI & Robotics Teaching Assistant",
    instructions="""You are an expert teaching assistant for a Physical AI & Robotics textbook.

Your role:
- Answer questions about robotics, ROS 2, simulation, computer vision, and related topics
- Use the search_textbook tool when you need specific information from the textbook
- Provide clear, educational responses with examples when appropriate
- If the user has selected specific text, prioritize that context in your answer
- Cite sources when referencing textbook content

Guidelines:
- Be concise but thorough
- Use technical terminology appropriately
- Provide practical examples when helpful
- If information isn't in the textbook, clearly state that
- Encourage hands-on learning and experimentation""",
    model=settings.openai_model,
    tools=[search_textbook]
)


class AgentService:
    """Service for running the OpenAI Agent with RAG capabilities."""

    @staticmethod
    async def run_agent(
        query: str,
        session: 'PostgresSession',
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Run the teaching agent with the user's query.

        Args:
            query: User's question
            session: PostgresSession for conversation persistence
            selected_text: Optional selected text from user

        Returns:
            Dictionary with agent response, structured sources, and metadata
        """
        global _last_search_results

        # Clear previous search results
        _last_search_results = []

        # Load conversation history from session
        await session.load_history()
        history_messages = session.get_messages()

        # Build input with conversation history and selected text if provided
        # Format conversation history for context
        context_parts = []
        
        # Add conversation history if available
        # Note: history_messages contains previous messages (current message not yet saved)
        if history_messages and len(history_messages) > 0:
            history_context = []
            for msg in history_messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                if role == "user":
                    history_context.append(f"User: {content}")
                elif role == "assistant":
                    history_context.append(f"Assistant: {content}")
            
            if history_context:
                context_parts.append("=== Previous Conversation ===")
                context_parts.append("\n".join(history_context))
                context_parts.append("")  # Empty line separator

        # Add selected text context if provided
        if selected_text:
            context_parts.append(f"=== User-Selected Text Context ===")
            context_parts.append(selected_text)
            context_parts.append("")  # Empty line separator

        # Add current question
        context_parts.append(f"=== Current Question ===")
        context_parts.append(query)

        # Combine all context parts
        full_input = "\n".join(context_parts)

        # Save user message to session (before running agent)
        await session.save_message("user", query)

        # Run the agent with full context including conversation history
        result = await Runner.run(teaching_agent, input=full_input)

        # Save assistant message to session
        await session.save_message("assistant", result.final_output)

        # Extract structured sources from search results
        # Results from vector_search are dicts with keys: score, chapter, lesson, heading, content, url, file_path
        sources = []
        if _last_search_results:
            for result_item in _last_search_results:
                # Map vector_search dict keys to expected source format
                sources.append({
                    "chapter_name": result_item.get("chapter", "Unknown"),
                    "lesson_title": result_item.get("lesson", "Unknown"),
                    "section_heading": result_item.get("heading", "N/A"),
                    "url": result_item.get("url", ""),
                    "relevance_score": float(result_item.get("score", 0.0))
                })

        # Extract tool calls safely (check if attribute exists)
        # Note: tool_calls may not exist on all RunResult objects
        tools_used: List[str] = []
        iterations = 0
        try:
            tool_calls = getattr(result, 'tool_calls', None)
            if tool_calls:
                tools_used = [getattr(call, 'name', 'unknown') for call in tool_calls]
                iterations = len(tool_calls)
        except (AttributeError, TypeError):
            # If tool_calls doesn't exist or has unexpected structure, use empty list
            tools_used = []
            iterations = 0

        return {
            "message": result.final_output,
            "sources": sources,
            "agent_metadata": {
                "tools_used": tools_used,
                "iterations": iterations
            }
        }

    @staticmethod
    async def run_agent_stream(
        query: str,
        session: 'PostgresSession',
        selected_text: Optional[str] = None
    ) -> AsyncIterator[Dict[str, Any]]:
        """
        Run the teaching agent with streaming support.

        Args:
            query: User's question
            session: PostgresSession for conversation persistence
            selected_text: Optional selected text from user

        Yields:
            Dictionary with event type and data:
            - {"type": "text", "data": {"delta": "..."}} for text deltas
            - {"type": "done", "data": {"message": "...", "sources": [...], "conversation_id": "..."}} when complete
        """
        global _last_search_results

        # Clear previous search results
        _last_search_results = []

        # Load conversation history from session
        await session.load_history()
        history_messages = session.get_messages()

        # Build input with conversation history and selected text if provided
        context_parts = []
        
        # Add conversation history if available
        if history_messages and len(history_messages) > 0:
            history_context = []
            for msg in history_messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                if role == "user":
                    history_context.append(f"User: {content}")
                elif role == "assistant":
                    history_context.append(f"Assistant: {content}")
            
            if history_context:
                context_parts.append("=== Previous Conversation ===")
                context_parts.append("\n".join(history_context))
                context_parts.append("")

        # Add selected text context if provided
        if selected_text:
            context_parts.append(f"=== User-Selected Text Context ===")
            context_parts.append(selected_text)
            context_parts.append("")

        # Add current question
        context_parts.append(f"=== Current Question ===")
        context_parts.append(query)

        # Combine all context parts
        full_input = "\n".join(context_parts)

        # Save user message to session (before running agent)
        await session.save_message("user", query)

        # Run the agent with streaming
        result = Runner.run_streamed(teaching_agent, input=full_input)

        # Stream events from the result
        async for event in result.stream_events():
            # Stream text deltas
            if event.type == "raw_response_event":
                if isinstance(event.data, ResponseTextDeltaEvent):
                    yield {
                        "type": "text",
                        "data": {"delta": event.data.delta}
                    }
            # Track tool calls for source extraction
            elif event.type == "run_item_stream_event":
                if hasattr(event, 'item') and event.item.type == "tool_call_output_item":
                    # Tool was called, results are in _last_search_results
                    pass

        # Save assistant message to session
        await session.save_message("assistant", result.final_output)

        # Extract structured sources from search results
        sources = []
        if _last_search_results:
            for result_item in _last_search_results:
                sources.append({
                    "chapter_name": result_item.get("chapter", "Unknown"),
                    "lesson_title": result_item.get("lesson", "Unknown"),
                    "section_heading": result_item.get("heading", "N/A"),
                    "url": result_item.get("url", ""),
                    "relevance_score": float(result_item.get("score", 0.0))
                })

        # Send final message with sources
        yield {
            "type": "done",
            "data": {
                "message": result.final_output,
                "sources": sources,
                "conversation_id": session.conversation_id
            }
        }
