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
from agents import Runner

from services.agent_service import teaching_agent
from services.session_service import PostgresSession
from services.chatkit_store import chatkit_id_to_uuid, is_chatkit_thread_id
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
            logger.info(f"[ChatKit] === Processing message for thread {thread.id} ===")

            # Log the raw input message
            logger.info(f"[ChatKit] input_user_message type: {type(input_user_message)}")
            logger.info(f"[ChatKit] input_user_message content: {input_user_message}")

            # Create AgentContext for ChatKit integration
            agent_context = AgentContext(
                thread=thread,
                store=self.store,
                request_context=context
            )

            # Convert ChatKit user message to agent input (await the coroutine)
            agent_input = await simple_to_agent_input(input_user_message)
            logger.info(f"[ChatKit] agent_input type: {type(agent_input)}")
            logger.info(f"[ChatKit] agent_input: {agent_input}")

            # Extract text from agent input to pass to Runner
            # agent_input is a list of message dicts with role and content
            user_text = ""
            if agent_input and len(agent_input) > 0:
                # Get the last user message
                for msg in reversed(agent_input):
                    if msg.get("role") == "user":
                        content = msg.get("content", "")
                        if isinstance(content, str):
                            user_text = content
                        elif isinstance(content, list):
                            # Extract text from content array
                            # Content can have items with type "text", "input_text", or just "text" key
                            text_parts = []
                            for part in content:
                                if isinstance(part, dict):
                                    # Check for "text" key regardless of "type" value
                                    # Handles both {"type": "text", "text": "..."} and {"type": "input_text", "text": "..."}
                                    if "text" in part:
                                        text_parts.append(part.get("text", ""))
                                elif isinstance(part, str):
                                    text_parts.append(part)
                            user_text = "\n".join(text_parts)
                        break

            # Log the extracted user text
            logger.info(f"[ChatKit] Extracted user_text: '{user_text}'")
            logger.info(f"[ChatKit] user_text length: {len(user_text)}")

            # Prepend selected text if available in context
            selected_text = context.get("selected_text")
            if selected_text:
                logger.info(f"[ChatKit] Prepending selected text: '{selected_text[:100]}...'")
                user_text = f'Selected text: "{selected_text}"\n\n{user_text}'
                logger.info(f"[ChatKit] Updated user_text with selected text context")

            # Convert ChatKit thread ID to conversation_id for PostgresSession
            # This allows us to reuse AgentService's history loading logic
            if is_chatkit_thread_id(thread.id):
                thread_uuid, _ = chatkit_id_to_uuid(thread.id)
                conversation_id = str(thread_uuid)
                logger.info(f"[ChatKit] Converted thread ID {thread.id} to conversation_id {conversation_id}")
            else:
                conversation_id = thread.id
                logger.info(f"[ChatKit] Using thread ID as conversation_id: {conversation_id}")

            # Create PostgresSession to reuse AgentService's history loading logic
            session = PostgresSession(conversation_id=conversation_id, session_id=thread.id)
            
            # Load conversation history using PostgresSession (same as AgentService)
            await session.load_history()
            history_messages = session.get_messages()
            
            logger.info(f"[ChatKit] Loaded {len(history_messages)} history messages from PostgresSession")
            if len(history_messages) == 0:
                logger.info(f"[ChatKit] No history messages found - this may be a new thread")
            else:
                logger.info(f"[ChatKit] History message roles: {[msg.get('role') for msg in history_messages]}")

            # Build input with conversation history and current question (same format as AgentService)
            context_parts = []
            
            # Add conversation history if available (same format as AgentService.run_agent_stream)
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

            # Add current question
            context_parts.append("=== Current Question ===")
            context_parts.append(user_text)

            # Combine all context parts
            full_input = "\n".join(context_parts)
            
            # Save user message to session (before running agent, same as AgentService)
            await session.save_message("user", user_text)

            # Log what we're passing to the agent
            logger.info(f"[ChatKit] Full input length: {len(full_input)} characters")
            logger.info(f"[ChatKit] Full input preview: {full_input[:200]}..." if len(full_input) > 200 else f"[ChatKit] Full input: {full_input}")
            logger.info(f"[ChatKit] Calling Runner.run_streamed with formatted input")

            # Run the agent with streaming using Runner (keep ChatKit event format)
            agent_result = Runner.run_streamed(teaching_agent, input=full_input)

            # Stream agent responses as ChatKit events
            # Note: stream_agent_response handles saving to ChatKit store
            async for event in stream_agent_response(agent_context, agent_result):
                yield event
            
            # After streaming completes, get final output and save to PostgresSession (same as AgentService)
            # This ensures messages are saved to database for history loading
            try:
                # Access final_output after streaming completes (same pattern as AgentService.run_agent_stream)
                if hasattr(agent_result, 'final_output') and agent_result.final_output:
                    await session.save_message("assistant", agent_result.final_output)
                    logger.info(f"[ChatKit] Saved assistant message to PostgresSession")
                else:
                    logger.warning(f"[ChatKit] Could not get final_output from agent_result")
            except Exception as save_error:
                logger.warning(f"[ChatKit] Failed to save assistant message to PostgresSession: {save_error}", exc_info=True)

            logger.info(f"[ChatKit] Response completed for thread {thread.id}")

        except Exception as e:
            logger.error(f"[ChatKit] Error in respond: {e}", exc_info=True)
            # You can yield error events here if needed
            raise
