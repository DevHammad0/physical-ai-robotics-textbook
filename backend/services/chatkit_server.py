"""
ChatKit server implementation for Physical AI & Robotics teaching assistant.
Integrates with existing RAG agent service using OpenAI Agents SDK.
"""
import logging
from typing import AsyncIterator, Any, Dict
from chatkit.server import ChatKitServer, ThreadStreamEvent
from chatkit.store import Store
from chatkit.types import ThreadMetadata, UserMessageItem
from chatkit.agents import AgentContext, stream_agent_response, simple_to_agent_input

from services.agent_service import teaching_agent
from config import settings

logger = logging.getLogger(__name__)


class PhysicalAIChatKitServer(ChatKitServer):
    """ChatKit server that uses our existing RAG teaching agent."""

    def __init__(self, data_store: Store, attachment_store=None):
        super().__init__(data_store, attachment_store)

    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: Dict[str, Any]
    ) -> AsyncIterator[ThreadStreamEvent]:
        """
        Process user message and stream responses using our RAG agent.

        Args:
            thread: ChatKit thread metadata
            input_user_message: User's message input
            context: Request context (can contain session info, user ID, etc.)

        Yields:
            ChatKit ThreadStreamEvent instances
        """
        try:
            logger.info(f"[ChatKit] Processing message for thread {thread.id}")

            # Create AgentContext for ChatKit integration
            agent_context = AgentContext(
                thread=thread,
                store=self.store,
                context=context
            )

            # Convert ChatKit user message to agent input
            agent_input = simple_to_agent_input(input_user_message)

            # Run the agent with streaming
            # Use thread ID as session ID for conversation persistence
            agent_result = teaching_agent.run_stream(
                session_id=thread.id,
                input_messages=agent_input
            )

            # Stream agent responses as ChatKit events
            async for event in stream_agent_response(agent_context, agent_result):
                yield event

            logger.info(f"[ChatKit] Response completed for thread {thread.id}")

        except Exception as e:
            logger.error(f"[ChatKit] Error in respond: {e}", exc_info=True)
            # You can yield error events here if needed
            raise
