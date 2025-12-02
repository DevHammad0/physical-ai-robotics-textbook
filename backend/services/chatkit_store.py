"""
PostgreSQL-backed store for ChatKit threads and items.
Maps ChatKit threads to PostgreSQL conversations and items to messages.
"""
from typing import Dict, List, Any, Optional, Tuple
from chatkit.store import Store, Page
from chatkit.types import ThreadMetadata, ThreadItem, Attachment
import asyncio
import json
import logging
from uuid import UUID, uuid4, uuid5, NAMESPACE_DNS
from datetime import datetime

from services.db_service import get_pool

logger = logging.getLogger(__name__)


def chatkit_id_to_uuid(chatkit_id: str) -> Tuple[UUID, str]:
    """
    Convert ChatKit thread ID to a deterministic UUID for database storage.

    ChatKit generates IDs like 'thr_b8a8abe9' which aren't valid UUIDs.
    We use UUID5 (deterministic namespace-based UUID) to create a consistent
    mapping between ChatKit IDs and database UUIDs.

    Args:
        chatkit_id: ChatKit thread ID (e.g., 'thr_b8a8abe9')

    Returns:
        Tuple of (UUID for database, original ChatKit ID for session_id)
    """
    # Use UUID5 to generate a deterministic UUID from the ChatKit ID
    # This ensures the same ChatKit ID always maps to the same UUID
    db_uuid = uuid5(NAMESPACE_DNS, f"chatkit.thread.{chatkit_id}")
    return db_uuid, chatkit_id


def is_chatkit_thread_id(thread_id: str) -> bool:
    """Check if a thread ID is in ChatKit format (thr_xxxxxxxx)."""
    return thread_id.startswith("thr_") if thread_id else False


def is_chatkit_message_id(message_id: str) -> bool:
    """Check if a message ID is in ChatKit format (msg_xxxxxxxx)."""
    return message_id.startswith("msg_") if message_id else False


def chatkit_message_id_to_uuid(message_id: str) -> UUID:
    """
    Convert ChatKit message ID to a deterministic UUID for database storage.
    
    ChatKit generates message IDs like 'msg_916b57b4' which aren't valid UUIDs.
    We use UUID5 (deterministic namespace-based UUID) to create a consistent
    mapping between ChatKit message IDs and database UUIDs.
    
    Args:
        message_id: ChatKit message ID (e.g., 'msg_916b57b4')
    
    Returns:
        UUID for database storage
    """
    # Use UUID5 to generate a deterministic UUID from the ChatKit message ID
    # This ensures the same ChatKit message ID always maps to the same UUID
    return uuid5(NAMESPACE_DNS, f"chatkit.message.{message_id}")


def serialize_metadata(metadata: Dict[str, Any]) -> str:
    """
    Serialize metadata to JSON, converting datetime objects to ISO strings.

    Args:
        metadata: Dictionary containing metadata values

    Returns:
        JSON string with datetime objects converted to ISO format
    """
    def convert_value(obj):
        """Recursively convert datetime objects to ISO strings."""
        if isinstance(obj, datetime):
            return obj.isoformat()
        elif isinstance(obj, dict):
            return {k: convert_value(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [convert_value(item) for item in obj]
        return obj

    serializable_metadata = convert_value(metadata)
    return json.dumps(serializable_metadata)


class PostgresStore(Store[Any]):
    """PostgreSQL-backed implementation of ChatKit Store."""

    def __init__(self):
        self._lock = asyncio.Lock()

    async def load_thread(self, thread_id: str, context: Any) -> ThreadMetadata:
        """Load thread metadata by ID (maps to conversation)."""
        pool = get_pool()

        # Handle ChatKit thread IDs (e.g., 'thr_xxxxxxxx')
        if is_chatkit_thread_id(thread_id):
            db_uuid, chatkit_id = chatkit_id_to_uuid(thread_id)

            # Try to load by session_id (ChatKit ID) first
            async with pool.acquire() as conn:
                row = await conn.fetchrow(
                    """
                    SELECT id, session_id, user_id, created_at, updated_at, metadata
                    FROM conversations
                    WHERE session_id = $1
                    """,
                    chatkit_id
                )

                if row:
                    # Found existing conversation, return with ChatKit ID
                    metadata_dict = json.loads(row['metadata']) if row['metadata'] else {}
                    # Exclude created_at from metadata_dict to avoid duplicate parameter
                    metadata_dict.pop('created_at', None)
                    return ThreadMetadata(
                        id=chatkit_id,  # Return ChatKit ID to client
                        created_at=row['created_at'].isoformat() if row['created_at'] else datetime.utcnow().isoformat(),
                        **metadata_dict
                    )

                # Create new conversation with ChatKit ID mapping
                created_at = datetime.utcnow()
                async with self._lock:
                    async with pool.acquire() as conn:
                        await conn.execute(
                            """
                            INSERT INTO conversations (id, session_id, created_at)
                            VALUES ($1, $2, $3)
                            ON CONFLICT (id) DO NOTHING
                            """,
                            db_uuid,
                            chatkit_id,
                            created_at
                        )

                return ThreadMetadata(id=chatkit_id, created_at=created_at.isoformat())

        # Handle regular UUID format (backward compatibility)
        async with pool.acquire() as conn:
            try:
                row = await conn.fetchrow(
                    """
                    SELECT id, session_id, user_id, created_at, updated_at, metadata
                    FROM conversations
                    WHERE id = $1
                    """,
                    UUID(thread_id)
                )

                if row:
                    metadata_dict = json.loads(row['metadata']) if row['metadata'] else {}
                    # Exclude created_at from metadata_dict to avoid duplicate parameter
                    metadata_dict.pop('created_at', None)
                    return ThreadMetadata(
                        id=thread_id,
                        created_at=row['created_at'].isoformat() if row['created_at'] else datetime.utcnow().isoformat(),
                        **metadata_dict
                    )
            except (ValueError, TypeError) as e:
                logger.warning(f"Invalid UUID format for thread_id {thread_id}: {e}")

            # Create new thread with UUID
            new_id = uuid4()
            created_at = datetime.utcnow()
            async with self._lock:
                async with pool.acquire() as conn:
                    await conn.execute(
                        """
                        INSERT INTO conversations (id, session_id, created_at)
                        VALUES ($1, $2, $3)
                        ON CONFLICT (id) DO NOTHING
                        """,
                        new_id,
                        thread_id,
                        created_at
                    )

            return ThreadMetadata(id=thread_id, created_at=created_at.isoformat())

    async def save_thread(self, thread: ThreadMetadata, context: Any) -> None:
        """Save thread metadata (maps to conversation)."""
        pool = get_pool()

        # Handle ChatKit thread IDs
        if is_chatkit_thread_id(thread.id):
            db_uuid, chatkit_id = chatkit_id_to_uuid(thread.id)

            async with pool.acquire() as conn:
                # Extract metadata from thread (excluding id and created_at which are separate columns)
                if hasattr(thread, 'model_dump'):
                    thread_dict = thread.model_dump()
                elif hasattr(thread, 'dict'):
                    thread_dict = thread.dict()
                else:
                    thread_dict = getattr(thread, '__dict__', {})
                metadata = {k: v for k, v in thread_dict.items() if k not in ['id', 'created_at']}

                await conn.execute(
                    """
                    INSERT INTO conversations (id, session_id, created_at, updated_at, metadata)
                    VALUES ($1, $2, NOW(), NOW(), $3::jsonb)
                    ON CONFLICT (id) DO UPDATE SET
                        updated_at = NOW(),
                        metadata = $3::jsonb
                    """,
                    db_uuid,
                    chatkit_id,
                    serialize_metadata(metadata)
                )
            return

        # Handle regular UUID format (backward compatibility)
        try:
            thread_uuid = UUID(thread.id)
        except (ValueError, TypeError):
            logger.warning(f"Invalid thread ID format: {thread.id}")
            return

        async with pool.acquire() as conn:
            # Extract metadata from thread (excluding id and created_at which are separate columns)
            if hasattr(thread, 'model_dump'):
                thread_dict = thread.model_dump()
            elif hasattr(thread, 'dict'):
                thread_dict = thread.dict()
            else:
                thread_dict = getattr(thread, '__dict__', {})
            metadata = {k: v for k, v in thread_dict.items() if k not in ['id', 'created_at']}

            await conn.execute(
                """
                INSERT INTO conversations (id, session_id, created_at, updated_at, metadata)
                VALUES ($1, $2, NOW(), NOW(), $3::jsonb)
                ON CONFLICT (id) DO UPDATE SET
                    updated_at = NOW(),
                    metadata = $3::jsonb
                """,
                thread_uuid,
                thread.id,  # Use thread.id as session_id
                serialize_metadata(metadata)
            )

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: Any,
    ) -> Page[ThreadItem]:
        """Load thread items with pagination (maps to messages)."""
        pool = get_pool()

        # Convert ChatKit thread ID to UUID if needed
        if is_chatkit_thread_id(thread_id):
            thread_uuid, _ = chatkit_id_to_uuid(thread_id)
        else:
            try:
                thread_uuid = UUID(thread_id)
            except (ValueError, TypeError):
                logger.warning(f"Invalid thread ID format: {thread_id}")
                return Page(data=[], has_more=False, first_id=None, last_id=None)
        
        async with pool.acquire() as conn:
            # Build query based on order and pagination
            order_clause = "ASC" if order == "asc" else "DESC"
            after_clause = ""
            params: List[Any] = [thread_uuid]
            
            if after:
                try:
                    after_uuid = UUID(after)
                    after_clause = f"AND id {'>' if order == 'asc' else '<'} ${len(params) + 1}"
                    params.append(after_uuid)
                except (ValueError, TypeError):
                    logger.warning(f"Invalid after ID format: {after}")
            
            limit_clause = f"LIMIT ${len(params) + 1}" if limit > 0 else ""
            if limit > 0:
                params.append(limit + 1)  # Fetch one extra to check has_more
            
            query = f"""
                SELECT id, role, content, created_at, metadata
                FROM messages
                WHERE conversation_id = $1 {after_clause}
                ORDER BY created_at {order_clause}
                {limit_clause}
            """
            
            logger.debug(f"[ChatKit Store] Loading thread items: thread_id={thread_id}, thread_uuid={thread_uuid}, order={order}, limit={limit}")
            rows = await conn.fetch(query, *params)
            logger.debug(f"[ChatKit Store] Found {len(rows)} messages in database")
            
            # Convert to ThreadItem objects
            items = []
            for row in rows:
                try:
                    metadata_dict = json.loads(row['metadata']) if row['metadata'] else {}
                    # Map message to ThreadItem
                    # Note: ThreadItem structure depends on Chatkit SDK - adjust as needed
                    item = ThreadItem(
                        id=str(row['id']),
                        type="message",  # or determine from row data
                        role=row['role'],
                        content=[{"type": "text", "text": row['content']}],
                        **metadata_dict
                    )
                    items.append(item)
                except Exception as e:
                    logger.warning(f"Error converting message to ThreadItem: {e}")
                    continue
            
            # Check if there are more items
            has_more = len(items) > limit if limit > 0 else False
            if has_more:
                items = items[:limit]
            
            return Page(
                data=items,
                has_more=has_more,
                first_id=items[0].id if items else None,
                last_id=items[-1].id if items else None
            )

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: Any
    ) -> None:
        """Add a new item to a thread (maps to message)."""
        await self.save_item(thread_id, item, context)

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: Any
    ) -> None:
        """Save/update an item in a thread (maps to message)."""
        pool = get_pool()

        # Convert ChatKit thread ID to UUID if needed
        if is_chatkit_thread_id(thread_id):
            thread_uuid, _ = chatkit_id_to_uuid(thread_id)
        else:
            try:
                thread_uuid = UUID(thread_id)
            except (ValueError, TypeError) as e:
                logger.warning(f"Invalid thread UUID format: {e}")
                return

        # Convert ChatKit message ID to UUID if needed
        try:
            if is_chatkit_message_id(item.id):
                item_uuid = chatkit_message_id_to_uuid(item.id)
                logger.debug(f"Converted ChatKit message ID {item.id} to UUID {item_uuid}")
            else:
                item_uuid = UUID(item.id)
        except (ValueError, TypeError) as e:
            logger.warning(f"Invalid item UUID format: {item.id}, error: {e}")
            return
        
        # Extract content from ThreadItem
        content = ""
        if hasattr(item, 'content') and item.content:
            if isinstance(item.content, list):
                # Extract text from content array
                text_parts = []
                for part in item.content:
                    if isinstance(part, dict) and part.get('type') == 'text':
                        text_parts.append(part.get('text', ''))
                    elif isinstance(part, str):
                        text_parts.append(part)
                content = '\n'.join(text_parts)
            elif isinstance(item.content, str):
                content = item.content
        
        # Extract role
        role = getattr(item, 'role', 'user')
        
        # Extract metadata
        if hasattr(item, 'model_dump'):
            item_dict = item.model_dump()
        elif hasattr(item, 'dict'):
            item_dict = item.dict()
        else:
            item_dict = getattr(item, '__dict__', {})
        metadata = {k: v for k, v in item_dict.items() if k not in ['id', 'content', 'role']}
        
        async with pool.acquire() as conn:
            # Ensure conversation exists
            await conn.execute(
                """
                INSERT INTO conversations (id, session_id, created_at)
                VALUES ($1, $2, NOW())
                ON CONFLICT (id) DO NOTHING
                """,
                thread_uuid,
                thread_id
            )
            
            # Save/update message
            logger.info(f"[ChatKit Store] Saving message: thread_id={thread_id}, item_id={item.id}, role={role}, content_length={len(content)}")
            await conn.execute(
                """
                INSERT INTO messages (id, conversation_id, role, content, created_at, metadata)
                VALUES ($1, $2, $3, $4, NOW(), $5::jsonb)
                ON CONFLICT (id) DO UPDATE SET
                    role = EXCLUDED.role,
                    content = EXCLUDED.content,
                    metadata = EXCLUDED.metadata
                """,
                item_uuid,
                thread_uuid,
                role,
                content,
                serialize_metadata(metadata)
            )
            logger.debug(f"[ChatKit Store] Message saved successfully: item_id={item.id}")

    async def load_item(
        self, thread_id: str, item_id: str, context: Any
    ) -> ThreadItem:
        """Load a specific item from a thread."""
        pool = get_pool()

        # Convert ChatKit thread ID to UUID if needed
        if is_chatkit_thread_id(thread_id):
            thread_uuid, _ = chatkit_id_to_uuid(thread_id)
        else:
            try:
                thread_uuid = UUID(thread_id)
            except (ValueError, TypeError) as e:
                raise ValueError(f"Invalid thread UUID format: {e}")

        # Convert ChatKit message ID to UUID if needed
        try:
            if is_chatkit_message_id(item_id):
                item_uuid = chatkit_message_id_to_uuid(item_id)
                logger.debug(f"Converted ChatKit message ID {item_id} to UUID {item_uuid}")
            else:
                item_uuid = UUID(item_id)
        except (ValueError, TypeError) as e:
            raise ValueError(f"Invalid item UUID format: {item_id}, error: {e}")
        
        async with pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT id, role, content, created_at, metadata
                FROM messages
                WHERE id = $1 AND conversation_id = $2
                """,
                item_uuid,
                thread_uuid
            )
            
            if not row:
                raise ValueError(f"Item {item_id} not found in thread {thread_id}")
            
            metadata_dict = json.loads(row['metadata']) if row['metadata'] else {}
            return ThreadItem(
                id=str(row['id']),
                type="message",
                role=row['role'],
                content=[{"type": "text", "text": row['content']}],
                **metadata_dict
            )

    async def delete_thread(self, thread_id: str, context: Any) -> None:
        """Delete a thread and all its items (cascade deletes messages)."""
        pool = get_pool()

        # Convert ChatKit thread ID to UUID if needed
        if is_chatkit_thread_id(thread_id):
            thread_uuid, _ = chatkit_id_to_uuid(thread_id)
        else:
            try:
                thread_uuid = UUID(thread_id)
            except (ValueError, TypeError):
                logger.warning(f"Invalid thread ID format: {thread_id}")
                return
        
        async with pool.acquire() as conn:
            await conn.execute(
                "DELETE FROM conversations WHERE id = $1",
                thread_uuid
            )

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: Any
    ) -> None:
        """Delete a specific item from a thread."""
        pool = get_pool()

        # Convert ChatKit thread ID to UUID if needed
        if is_chatkit_thread_id(thread_id):
            thread_uuid, _ = chatkit_id_to_uuid(thread_id)
        else:
            try:
                thread_uuid = UUID(thread_id)
            except (ValueError, TypeError):
                logger.warning(f"Invalid thread UUID format")
                return

        try:
            item_uuid = UUID(item_id)
        except (ValueError, TypeError):
            logger.warning(f"Invalid item UUID format")
            return
        
        async with pool.acquire() as conn:
            await conn.execute(
                """
                DELETE FROM messages
                WHERE id = $1 AND conversation_id = $2
                """,
                item_uuid,
                thread_uuid
            )

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: Any,
    ) -> Page[ThreadMetadata]:
        """Load all threads with pagination."""
        pool = get_pool()
        order_clause = "ASC" if order == "asc" else "DESC"
        after_clause = ""
        params: List[Any] = []
        
        if after:
            try:
                after_uuid = UUID(after)
                after_clause = f"WHERE id {'>' if order == 'asc' else '<'} ${len(params) + 1}"
                params.append(after_uuid)
            except (ValueError, TypeError):
                logger.warning(f"Invalid after ID format: {after}")
        
        limit_clause = f"LIMIT ${len(params) + 1}" if limit > 0 else ""
        if limit > 0:
            params.append(limit + 1)  # Fetch one extra to check has_more
        
        async with pool.acquire() as conn:
            query = f"""
                SELECT id, session_id, user_id, created_at, updated_at, metadata
                FROM conversations
                {after_clause}
                ORDER BY created_at {order_clause}
                {limit_clause}
            """
            
            rows = await conn.fetch(query, *params)
            
            threads = []
            for row in rows:
                try:
                    metadata_dict = json.loads(row['metadata']) if row['metadata'] else {}
                    # Exclude created_at from metadata_dict to avoid duplicate parameter
                    metadata_dict.pop('created_at', None)
                    # Return session_id (ChatKit ID) if it looks like a ChatKit thread, otherwise use UUID
                    thread_id = row['session_id'] if row['session_id'] and is_chatkit_thread_id(row['session_id']) else str(row['id'])
                    threads.append(ThreadMetadata(
                        id=thread_id,
                        created_at=row['created_at'].isoformat() if row['created_at'] else datetime.utcnow().isoformat(),
                        **metadata_dict
                    ))
                except Exception as e:
                    logger.warning(f"Error converting conversation to ThreadMetadata: {e}")
                    continue
            
            has_more = len(threads) > limit if limit > 0 else False
            if has_more:
                threads = threads[:limit]
            
            return Page(
                data=threads,
                has_more=has_more,
                first_id=threads[0].id if threads else None,
                last_id=threads[-1].id if threads else None
            )

    async def save_attachment(self, attachment: Attachment, context: Any) -> None:
        """Save an attachment (stored in metadata for now)."""
        # For now, attachments can be stored in message metadata
        # In the future, could create a separate attachments table
        logger.debug(f"Saving attachment {attachment.id}")

    async def load_attachment(
        self, attachment_id: str, context: Any
    ) -> Attachment:
        """Load an attachment by ID."""
        # For now, raise error as attachments not fully implemented
        raise ValueError(f"Attachment {attachment_id} not found")

    async def delete_attachment(self, attachment_id: str, context: Any) -> None:
        """Delete an attachment."""
        logger.debug(f"Deleting attachment {attachment_id}")


# Keep InMemoryStore for backward compatibility
class InMemoryStore(Store[Any]):
    """In-memory implementation of ChatKit Store."""

    def __init__(self):
        self.threads: Dict[str, ThreadMetadata] = {}
        self.items: Dict[str, List[ThreadItem]] = {}
        self.attachments: Dict[str, Attachment] = {}
        self._lock = asyncio.Lock()

    async def load_thread(self, thread_id: str, context: Any) -> ThreadMetadata:
        """Load thread metadata by ID."""
        async with self._lock:
            if thread_id not in self.threads:
                # Create new thread if it doesn't exist
                thread = ThreadMetadata(id=thread_id)
                self.threads[thread_id] = thread
                self.items[thread_id] = []
                return thread
            return self.threads[thread_id]

    async def save_thread(self, thread: ThreadMetadata, context: Any) -> None:
        """Save thread metadata."""
        async with self._lock:
            self.threads[thread.id] = thread
            if thread.id not in self.items:
                self.items[thread.id] = []

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: Any,
    ) -> Page[ThreadItem]:
        """Load thread items with pagination."""
        async with self._lock:
            items = self.items.get(thread_id, [])

            # Simple pagination - return all items for now
            return Page(
                data=items[-limit:] if limit > 0 else items,
                has_more=False,
                first_id=items[0].id if items else None,
                last_id=items[-1].id if items else None
            )

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: Any
    ) -> None:
        """Add a new item to a thread."""
        async with self._lock:
            if thread_id not in self.items:
                self.items[thread_id] = []
            self.items[thread_id].append(item)

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: Any
    ) -> None:
        """Save/update an item in a thread."""
        async with self._lock:
            if thread_id not in self.items:
                self.items[thread_id] = []

            # Find and update existing item, or append new one
            for i, existing_item in enumerate(self.items[thread_id]):
                if existing_item.id == item.id:
                    self.items[thread_id][i] = item
                    return

            # Item not found, add it
            self.items[thread_id].append(item)

    async def load_item(
        self, thread_id: str, item_id: str, context: Any
    ) -> ThreadItem:
        """Load a specific item from a thread."""
        async with self._lock:
            items = self.items.get(thread_id, [])
            for item in items:
                if item.id == item_id:
                    return item
            raise ValueError(f"Item {item_id} not found in thread {thread_id}")

    async def delete_thread(self, thread_id: str, context: Any) -> None:
        """Delete a thread and all its items."""
        async with self._lock:
            self.threads.pop(thread_id, None)
            self.items.pop(thread_id, None)

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: Any
    ) -> None:
        """Delete a specific item from a thread."""
        async with self._lock:
            if thread_id in self.items:
                self.items[thread_id] = [
                    item for item in self.items[thread_id] if item.id != item_id
                ]

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: Any,
    ) -> Page[ThreadMetadata]:
        """Load all threads with pagination."""
        async with self._lock:
            thread_list = list(self.threads.values())
            return Page(
                data=thread_list[-limit:] if limit > 0 else thread_list,
                has_more=False,
                first_id=thread_list[0].id if thread_list else None,
                last_id=thread_list[-1].id if thread_list else None
            )

    async def save_attachment(self, attachment: Attachment, context: Any) -> None:
        """Save an attachment."""
        async with self._lock:
            self.attachments[attachment.id] = attachment

    async def load_attachment(
        self, attachment_id: str, context: Any
    ) -> Attachment:
        """Load an attachment by ID."""
        async with self._lock:
            if attachment_id not in self.attachments:
                raise ValueError(f"Attachment {attachment_id} not found")
            return self.attachments[attachment_id]

    async def delete_attachment(self, attachment_id: str, context: Any) -> None:
        """Delete an attachment."""
        async with self._lock:
            self.attachments.pop(attachment_id, None)
