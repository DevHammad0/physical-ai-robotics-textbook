"""
Simple in-memory store for ChatKit threads and items.
For production, replace with database-backed store.
"""
from typing import Dict, List, Any
from chatkit.store import Store, Page
from chatkit.types import ThreadMetadata, ThreadItem, Attachment
import asyncio


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
