# Implementation Plan: RAG Chatbot for Physical AI & Robotics Textbook

**Feature Branch**: `006-rag-chatbot`  
**Created**: 2025-11-29  
**Status**: Ready for Implementation  
**Spec Reference**: [spec.md](./spec.md)

## Executive Summary

This plan outlines the implementation of a production-ready Retrieval-Augmented Generation (RAG) chatbot embedded in our Docusaurus-based Physical AI & Robotics Textbook. The system will enable students to ask questions about textbook content and receive accurate, contextually-aware answers with source citations in under 2 seconds.

### Architecture Overview

**Technology Stack**:
- **Backend**: FastAPI 0.115+ (Python 3.10+) hosted on Vercel Serverless Functions
- **LLM & Embeddings**: OpenAI API (gpt-3.5-turbo for generation, text-embedding-3-small for vectors)
- **Vector Database**: Qdrant Cloud (free tier, 1GB storage)
- **Conversation Storage**: Neon Serverless Postgres (free tier, 0.5GB)
- **Frontend**: React 19 + TypeScript + Docusaurus 3.9 (GitHub Pages)
- **Package Manager**: pnpm (frontend), pip + venv (backend)
- **Agent Framework**: OpenAI Agents SDK (agentic tool calling)
- **Server Protocol**: ChatKit Python SDK (streaming & session management)
- **Frontend SDK**: ChatKit React (@openai/chatkit-react)

**Key Metrics**:
- ~600 content chunks from 35+ markdown lessons
- <2s p90 latency for chat responses
- <200ms vector search time
- Support for 50+ concurrent users
- $6 total cost for hackathon period

**Agentic Architecture**:
- Teacher Agent autonomously decides when to search textbook
- Vector search as `@function_tool` (not hardcoded pipeline)
- Agentic Loop: User → Agent → Tool Call → Qdrant → Synthesize → Response

---

## 1. Architecture Design

### 1.1 Backend Architecture (FastAPI)

#### Component Structure

```
backend/
├── main.py                 # FastAPI app + CORS + middleware
├── config.py               # Environment variables (pydantic-settings)
├── routers/
│   ├── chat.py            # /api/chat, /api/chat/selection
│   └── health.py          # /api/health
├── services/
│   ├── vector_search.py   # Qdrant client + search
│   ├── llm_service.py     # OpenAI chat completions
│   └── db_service.py      # AsyncPG connection pool
├── models/
│   ├── schemas.py         # Pydantic request/response models
│   └── database.py        # SQLAlchemy table definitions
├── scripts/
│   ├── index_content.py   # Markdown parser + embeddings
│   └── init_db.py         # Create Postgres tables
└── requirements.txt       # Dependencies
```

#### FastAPI Application Setup (`main.py`)

Based on latest FastAPI documentation, we'll use:

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging

from routers import chat, health
from services.db_service import init_db_pool, close_db_pool
from config import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle (startup/shutdown)"""
    # Startup
    logger.info("Initializing database connection pool...")
    await init_db_pool()
    logger.info("Database pool initialized")
    
    yield
    
    # Shutdown
    logger.info("Closing database connection pool...")
    await close_db_pool()
    logger.info("Database pool closed")

app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    lifespan=lifespan
)

# CORS configuration for GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://devhammad0.github.io",
        "http://localhost:3000",  # Local Docusaurus dev
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)

# Include routers
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(health.router, prefix="/api", tags=["health"])

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API", "version": "1.0.0"}
```

#### Configuration Management (`config.py`)

Using pydantic-settings for type-safe environment variables:

```python
from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional

class Settings(BaseSettings):
    # OpenAI
    openai_api_key: str
    openai_model: str = "gpt-3.5-turbo"  # Change to gpt-4, gpt-4o, etc. via .env
    openai_embedding_model: str = "text-embedding-3-small"
    openai_max_tokens: int = 500
    openai_temperature: float = 0.7
    
    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "textbook_content"
    
    # Neon Postgres
    database_url: str  # postgresql://user:pass@host/db?sslmode=require
    db_pool_min_size: int = 2
    db_pool_max_size: int = 10
    
    # Rate limiting
    rate_limit_per_minute: int = 30
    
    # CORS
    cors_origins: list[str] = [
        "https://devhammad0.github.io",
        "http://localhost:3000"
    ]
    
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False
    )

settings = Settings()
```

#### Pydantic Request/Response Models (`models/schemas.py`)

```python
from pydantic import BaseModel, Field, validator
from typing import Optional
from datetime import datetime

class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    conversation_id: Optional[str] = None
    session_id: str
    selected_text: Optional[str] = Field(None, max_length=2000)
    
    @validator('query')
    def query_not_empty(cls, v):
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()

class Source(BaseModel):
    chapter_name: str
    lesson_title: str
    section_heading: str
    url: str
    relevance_score: float

class ChatResponse(BaseModel):
    message: str
    sources: list[Source]
    conversation_id: str
    response_time_ms: int

class HealthResponse(BaseModel):
    status: str
    database: str
    vector_db: str
    llm_api: str
    timestamp: datetime
```

#### Error Handling Strategy

```python
from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse
import logging

logger = logging.getLogger(__name__)

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions with user-friendly messages"""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.detail,
            "status_code": exc.status_code
        }
    )

@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Catch all unhandled exceptions"""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "An unexpected error occurred. Please try again later.",
            "status_code": 500
        }
    )

# Rate limiting error
class RateLimitException(HTTPException):
    def __init__(self):
        super().__init__(
            status_code=429,
            detail="Rate limit exceeded. Please wait 30 seconds before trying again."
        )
```

---

### 1.2 RAG Pipeline Design

#### Content Indexing Pipeline

**Step 1: Markdown Parsing**

```python
import re
import yaml
from pathlib import Path
from typing import Dict, List

class MarkdownParser:
    def __init__(self, docs_dir: str):
        self.docs_dir = Path(docs_dir)
    
    def parse_file(self, file_path: Path) -> Dict:
        """Parse markdown file with YAML frontmatter"""
        content = file_path.read_text(encoding='utf-8')
        
        # Extract YAML frontmatter
        frontmatter = {}
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                frontmatter = yaml.safe_load(parts[1])
                content = parts[2]
        
        # Extract chapter/lesson from path
        parts = file_path.relative_to(self.docs_dir).parts
        chapter = parts[0] if len(parts) > 0 else "unknown"
        lesson = file_path.stem
        
        # Parse headings and content sections
        sections = self._parse_sections(content)
        
        return {
            'file_path': str(file_path),
            'chapter': chapter,
            'lesson': lesson,
            'title': frontmatter.get('title', lesson),
            'sections': sections
        }
    
    def _parse_sections(self, content: str) -> List[Dict]:
        """Split content by headings"""
        sections = []
        current_heading = "Introduction"
        current_content = []
        
        for line in content.split('\n'):
            if line.startswith('##'):  # H2 heading
                if current_content:
                    sections.append({
                        'heading': current_heading,
                        'content': '\n'.join(current_content).strip()
                    })
                current_heading = line.lstrip('#').strip()
                current_content = []
            else:
                current_content.append(line)
        
        # Add last section
        if current_content:
            sections.append({
                'heading': current_heading,
                'content': '\n'.join(current_content).strip()
            })
        
        return sections
```

**Step 2: Chunking Strategy**

Decision: **Heading-based chunking** with 800 token max, 100 token overlap

Rationale:
- Preserves semantic boundaries (respects markdown structure)
- Each chunk corresponds to a meaningful section
- Better context for LLM than arbitrary splits

```python
import tiktoken

class ContentChunker:
    def __init__(self, max_tokens: int = 800, overlap_tokens: int = 100):
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens
        self.encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
    
    def chunk_sections(self, sections: List[Dict]) -> List[Dict]:
        """Create chunks from sections with overlap"""
        chunks = []
        
        for section in sections:
            content = section['content']
            heading = section['heading']
            tokens = self.encoding.encode(content)
            
            if len(tokens) <= self.max_tokens:
                # Section fits in one chunk
                chunks.append({
                    'heading': heading,
                    'content': content,
                    'token_count': len(tokens)
                })
            else:
                # Split section into multiple chunks
                for i in range(0, len(tokens), self.max_tokens - self.overlap_tokens):
                    chunk_tokens = tokens[i:i + self.max_tokens]
                    chunk_text = self.encoding.decode(chunk_tokens)
                    chunks.append({
                        'heading': f"{heading} (part {i // (self.max_tokens - self.overlap_tokens) + 1})",
                        'content': chunk_text,
                        'token_count': len(chunk_tokens)
                    })
        
        return chunks
    
    def count_tokens(self, text: str) -> int:
        """Count tokens in text"""
        return len(self.encoding.encode(text))
```

**Step 3: Embedding Generation**

Using OpenAI async client for batch processing:

```python
from openai import AsyncOpenAI
import asyncio
from typing import List

class EmbeddingGenerator:
    def __init__(self, api_key: str, model: str = "text-embedding-3-small"):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = model
    
    async def generate_embeddings(
        self, 
        texts: List[str], 
        batch_size: int = 100
    ) -> List[List[float]]:
        """Generate embeddings in batches"""
        all_embeddings = []
        
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            response = await self.client.embeddings.create(
                input=batch,
                model=self.model
            )
            embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(embeddings)
            
            # Rate limiting: sleep between batches
            if i + batch_size < len(texts):
                await asyncio.sleep(1)
        
        return all_embeddings
```

**Step 4: Qdrant Collection Setup**

```python
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import (
    VectorParams, Distance, PointStruct
)

class VectorStore:
    def __init__(self, url: str, api_key: str, collection_name: str):
        self.client = AsyncQdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
    
    async def create_collection(self):
        """Create collection with 1536-dimensional vectors (text-embedding-3-small)"""
        await self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=VectorParams(
                size=1536,  # text-embedding-3-small dimension
                distance=Distance.COSINE
            )
        )
    
    async def upsert_chunks(
        self, 
        chunks: List[Dict], 
        embeddings: List[List[float]]
    ):
        """Upsert chunks with embeddings to Qdrant"""
        points = []
        for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            points.append(PointStruct(
                id=idx,
                vector=embedding,
                payload={
                    'chapter': chunk['chapter'],
                    'lesson': chunk['lesson'],
                    'heading': chunk['heading'],
                    'content': chunk['content'],
                    'file_path': chunk['file_path'],
                    'url': chunk['url'],
                    'token_count': chunk['token_count']
                }
            ))
        
        await self.client.upsert(
            collection_name=self.collection_name,
            points=points,
            wait=True
        )
```

#### Query Processing Pipeline

**Flow**: Query → Embedding → Vector Search → Top-K Chunks → Context Building → LLM Generation → Response

**Step 1: Vector Search Service** (`services/vector_search.py`)

```python
from qdrant_client import AsyncQdrantClient
from typing import List, Dict
import logging

logger = logging.getLogger(__name__)

class VectorSearchService:
    def __init__(self, url: str, api_key: str, collection_name: str):
        self.client = AsyncQdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
    
    async def search(
        self, 
        query_vector: List[float], 
        top_k: int = 5
    ) -> List[Dict]:
        """Search for similar chunks"""
        try:
            results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True
            )
            
            chunks = []
            for result in results:
                chunks.append({
                    'score': result.score,
                    'chapter': result.payload['chapter'],
                    'lesson': result.payload['lesson'],
                    'heading': result.payload['heading'],
                    'content': result.payload['content'],
                    'url': result.payload['url']
                })
            
            return chunks
        except Exception as e:
            logger.error(f"Vector search failed: {e}")
            return []
```

**Step 2: LLM Service** (`services/llm_service.py`)

```python
from openai import AsyncOpenAI
from typing import List, Dict, Optional
import logging

logger = logging.getLogger(__name__)

class LLMService:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = model
    
    def build_system_prompt(self) -> str:
        """System prompt for chatbot behavior"""
        return """You are an AI teaching assistant for a Physical AI & Robotics textbook. Your role is to:

1. Answer questions accurately based ONLY on the provided textbook content
2. Cite specific chapters/lessons when referencing information
3. Use clear, educational language appropriate for students
4. If a question is outside the textbook scope, politely redirect: "This question is outside the textbook scope. Try asking about: ROS 2, Gazebo, Navigation, or AI Integration."
5. For unclear questions, ask for clarification
6. Keep responses concise (under 300 words)

Always base your answers on the retrieved context provided below."""
    
    def build_context(
        self, 
        chunks: List[Dict], 
        max_tokens: int = 3000
    ) -> str:
        """Build context from retrieved chunks"""
        context_parts = []
        token_count = 0
        
        for chunk in chunks:
            chunk_text = f"\n---\nChapter: {chunk['chapter']}\nLesson: {chunk['lesson']}\nSection: {chunk['heading']}\n\n{chunk['content']}\n"
            chunk_tokens = len(chunk_text.split())  # Approximate
            
            if token_count + chunk_tokens > max_tokens:
                break
            
            context_parts.append(chunk_text)
            token_count += chunk_tokens
        
        return "\n".join(context_parts)
    
    def build_conversation_context(
        self, 
        conversation_history: List[Dict], 
        max_messages: int = 5
    ) -> List[Dict]:
        """Build message history for LLM (last N messages)"""
        return conversation_history[-max_messages:]
    
    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict],
        conversation_history: List[Dict] = None,
        max_tokens: int = 500,
        temperature: float = 0.7
    ) -> Optional[str]:
        """Generate LLM response with error handling"""
        try:
            system_prompt = self.build_system_prompt()
            context = self.build_context(retrieved_chunks)
            
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "system", "content": f"Retrieved Context:\n{context}"}
            ]
            
            # Add conversation history
            if conversation_history:
                messages.extend(self.build_conversation_context(conversation_history))
            
            # Add current query
            messages.append({"role": "user", "content": query})
            
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature,
                n=1
            )
            
            return response.choices[0].message.content
        
        except Exception as e:
            logger.error(f"LLM generation failed: {e}")
            return None
    
    async def generate_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for query"""
        try:
            response = await self.client.embeddings.create(
                input=text,
                model="text-embedding-3-small"
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            return None
```

**Step 3: Context Window Management**

Token budget allocation:
- System prompt: ~200 tokens
- Retrieved context: ~3000 tokens (5 chunks × 600 tokens avg)
- Conversation history: ~1000 tokens (5 messages × 200 tokens avg)
- User query: ~100 tokens
- Response: ~500 tokens
- **Total: ~4800 tokens** (within gpt-3.5-turbo 16K limit with buffer)

**Step 4: Source Citation Extraction**

```python
def extract_sources(chunks: List[Dict], top_n: int = 3) -> List[Dict]:
    """Extract top-N unique sources from chunks"""
    seen = set()
    sources = []
    
    for chunk in chunks:
        key = (chunk['chapter'], chunk['lesson'])
        if key not in seen:
            sources.append({
                'chapter_name': chunk['chapter'],
                'lesson_title': chunk['lesson'],
                'section_heading': chunk['heading'],
                'url': chunk['url'],
                'relevance_score': chunk['score']
            })
            seen.add(key)
        
        if len(sources) >= top_n:
            break
    
    return sources
```

---

### 1.3 Database Schema Design

#### Postgres Tables (Neon)

```sql
-- Conversations table
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NULL,  -- NULL for anonymous users
    session_id VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_user_id ON conversations(user_id) WHERE user_id IS NOT NULL;
CREATE INDEX idx_conversations_created_at ON conversations(created_at);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    selected_text TEXT NULL,  -- For text-selection queries
    sources JSONB DEFAULT '[]'::jsonb,  -- Array of source objects
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_messages_conversation_id ON messages(conversation_id);
CREATE INDEX idx_messages_created_at ON messages(created_at);

-- Chat analytics table
CREATE TABLE chat_analytics (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query TEXT NOT NULL,
    response_time_ms INTEGER NOT NULL,
    success BOOLEAN NOT NULL,
    error_message TEXT NULL,
    session_id VARCHAR(255) NOT NULL,
    conversation_id UUID NULL REFERENCES conversations(id) ON DELETE SET NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_chat_analytics_session_id ON chat_analytics(session_id);
CREATE INDEX idx_chat_analytics_created_at ON chat_analytics(created_at);
CREATE INDEX idx_chat_analytics_success ON chat_analytics(success);

-- Users table (bonus feature)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255) NOT NULL,
    experience_level INTEGER CHECK (experience_level BETWEEN 0 AND 10),
    learning_goals TEXT,
    interests TEXT[],
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);
```

#### Database Service (`services/db_service.py`)

Using AsyncPG with connection pooling:

```python
import asyncpg
from typing import Optional, Dict, List
import logging
import json
from datetime import datetime
from uuid import UUID, uuid4

logger = logging.getLogger(__name__)

# Global pool
_pool: Optional[asyncpg.Pool] = None

async def init_db_pool():
    """Initialize connection pool on startup"""
    global _pool
    from config import settings
    
    _pool = await asyncpg.create_pool(
        dsn=settings.database_url,
        min_size=settings.db_pool_min_size,
        max_size=settings.db_pool_max_size,
        command_timeout=60,
        max_inactive_connection_lifetime=300.0
    )
    logger.info("Database connection pool initialized")

async def close_db_pool():
    """Close connection pool on shutdown"""
    global _pool
    if _pool:
        await _pool.close()
        logger.info("Database connection pool closed")

def get_pool() -> asyncpg.Pool:
    """Get database pool (dependency injection)"""
    if _pool is None:
        raise RuntimeError("Database pool not initialized")
    return _pool

class ConversationService:
    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool
    
    async def create_or_get_conversation(
        self, 
        session_id: str, 
        conversation_id: Optional[str] = None
    ) -> str:
        """Create new conversation or get existing"""
        async with self.pool.acquire() as conn:
            if conversation_id:
                # Check if exists
                result = await conn.fetchrow(
                    "SELECT id FROM conversations WHERE id = $1",
                    UUID(conversation_id)
                )
                if result:
                    return conversation_id
            
            # Create new conversation
            new_id = await conn.fetchval(
                """
                INSERT INTO conversations (session_id)
                VALUES ($1)
                RETURNING id
                """,
                session_id
            )
            return str(new_id)
    
    async def add_message(
        self,
        conversation_id: str,
        role: str,
        content: str,
        selected_text: Optional[str] = None,
        sources: List[Dict] = None
    ) -> str:
        """Add message to conversation"""
        async with self.pool.acquire() as conn:
            message_id = await conn.fetchval(
                """
                INSERT INTO messages 
                (conversation_id, role, content, selected_text, sources)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id
                """,
                UUID(conversation_id),
                role,
                content,
                selected_text,
                json.dumps(sources or [])
            )
            
            # Update conversation timestamp
            await conn.execute(
                """
                UPDATE conversations
                SET updated_at = NOW()
                WHERE id = $1
                """,
                UUID(conversation_id)
            )
            
            return str(message_id)
    
    async def get_conversation_history(
        self,
        conversation_id: str,
        limit: int = 10
    ) -> List[Dict]:
        """Get recent messages from conversation"""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT role, content, sources, created_at
                FROM messages
                WHERE conversation_id = $1
                ORDER BY created_at DESC
                LIMIT $2
                """,
                UUID(conversation_id),
                limit
            )
            
            # Reverse to chronological order
            messages = []
            for row in reversed(rows):
                messages.append({
                    'role': row['role'],
                    'content': row['content'],
                    'sources': json.loads(row['sources']) if row['sources'] else [],
                    'created_at': row['created_at'].isoformat()
                })
            
            return messages

class AnalyticsService:
    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool
    
    async def log_query(
        self,
        query: str,
        response_time_ms: int,
        success: bool,
        session_id: str,
        conversation_id: Optional[str] = None,
        error_message: Optional[str] = None
    ):
        """Log query for analytics"""
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO chat_analytics
                (query, response_time_ms, success, error_message, session_id, conversation_id)
                VALUES ($1, $2, $3, $4, $5, $6)
                """,
                query,
                response_time_ms,
                success,
                error_message,
                session_id,
                UUID(conversation_id) if conversation_id else None
            )
```

---

### 1.4 Frontend Architecture (React + Docusaurus)

#### Component Structure

```
book-source/
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   │   ├── ChatWidget.tsx
│   │   │   ├── ChatWidget.module.css
│   │   │   ├── MessageList.tsx
│   │   │   ├── MessageInput.tsx
│   │   │   └── SourceCitation.tsx
│   │   └── TextSelectionHandler.tsx
│   ├── theme/
│   │   └── Root.tsx                # Global component injection
│   └── utils/
│       ├── api.ts                  # Backend API calls
│       └── storage.ts              # localStorage management
└── docusaurus.config.js
```

#### ChatWidget Component (`ChatWidget.tsx`)

```typescript
import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatWidget.module.css';
import { sendChatMessage, ChatRequest, ChatResponse } from '../../utils/api';
import { saveConversation, loadConversation } from '../../utils/storage';
import MessageList from './MessageList';
import MessageInput from './MessageInput';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter_name: string;
    lesson_title: string;
    url: string;
  }>;
  created_at: string;
}

export default function ChatWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [sessionId] = useState(() => 
    localStorage.getItem('chatbot_session_id') || crypto.randomUUID()
  );
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);

  // Initialize session ID
  useEffect(() => {
    localStorage.setItem('chatbot_session_id', sessionId);
  }, [sessionId]);

  // Load conversation from localStorage
  useEffect(() => {
    if (isOpen) {
      const saved = loadConversation(sessionId);
      if (saved) {
        setMessages(saved.messages);
        setConversationId(saved.conversation_id);
      }
    }
  }, [isOpen, sessionId]);

  // Save conversation to localStorage
  useEffect(() => {
    if (conversationId && messages.length > 0) {
      saveConversation(sessionId, {
        conversation_id: conversationId,
        messages,
        updated_at: new Date().toISOString()
      });
    }
  }, [conversationId, messages, sessionId]);

  const handleSendMessage = async (messageText: string) => {
    if (!messageText.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: crypto.randomUUID(),
      role: 'user',
      content: messageText,
      created_at: new Date().toISOString()
    };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const request: ChatRequest = {
        query: messageText,
        conversation_id: conversationId,
        session_id: sessionId,
        selected_text: selectedText
      };

      const response: ChatResponse = await sendChatMessage(request);

      // Add assistant message
      const assistantMessage: Message = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: response.message,
        sources: response.sources,
        created_at: new Date().toISOString()
      };
      setMessages(prev => [...prev, assistantMessage]);
      setConversationId(response.conversation_id);
      setSelectedText(null); // Clear selected text after use
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        created_at: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearHistory = () => {
    if (confirm('Clear conversation history?')) {
      setMessages([]);
      setConversationId(null);
      localStorage.removeItem(`chatbot_conversation_${sessionId}`);
    }
  };

  return (
    <>
      {/* Floating button */}
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        💬
      </button>

      {/* Chat widget */}
      {isOpen && (
        <div className={styles.chatWidget}>
          <div className={styles.header}>
            <h3>AI Teaching Assistant</h3>
            <div className={styles.headerActions}>
              <button onClick={handleClearHistory} title="Clear history">🗑️</button>
              <button onClick={() => setIsOpen(false)} title="Close">✕</button>
            </div>
          </div>

          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <span>📄 Context: "{selectedText.substring(0, 50)}..."</span>
              <button onClick={() => setSelectedText(null)}>✕</button>
            </div>
          )}

          <MessageList messages={messages} />
          <MessageInput 
            onSend={handleSendMessage} 
            disabled={isLoading}
            isLoading={isLoading}
          />
        </div>
      )}
    </>
  );
}
```

#### API Integration (`utils/api.ts`)

```typescript
const API_BASE_URL = process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000';

export interface ChatRequest {
  query: string;
  conversation_id?: string | null;
  session_id: string;
  selected_text?: string | null;
}

export interface ChatResponse {
  message: string;
  sources: Array<{
    chapter_name: string;
    lesson_title: string;
    section_heading: string;
    url: string;
    relevance_score: number;
  }>;
  conversation_id: string;
  response_time_ms: number;
}

export async function sendChatMessage(request: ChatRequest): Promise<ChatResponse> {
  const response = await fetch(`${API_BASE_URL}/api/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    if (response.status === 429) {
      throw new Error('Rate limit exceeded. Please wait 30 seconds.');
    }
    throw new Error(`API error: ${response.statusText}`);
  }

  return response.json();
}

export async function checkHealth(): Promise<any> {
  const response = await fetch(`${API_BASE_URL}/api/health`);
  if (!response.ok) {
    throw new Error(`Health check failed: ${response.statusText}`);
  }
  return response.json();
}
```

#### Text Selection Handler (`utils/textSelection.ts`)

```typescript
export function initTextSelectionHandler(onSelection: (text: string) => void) {
  let selectionButton: HTMLElement | null = null;

  function handleSelection() {
    // Remove existing button
    if (selectionButton) {
      selectionButton.remove();
      selectionButton = null;
    }

    const selection = window.getSelection();
    if (!selection || selection.isCollapsed) return;

    const selectedText = selection.toString().trim();
    if (selectedText.length === 0 || selectedText.length > 2000) return;

    // Get selection position
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Create "Ask AI" button
    selectionButton = document.createElement('button');
    selectionButton.textContent = '🤖 Ask AI about this';
    selectionButton.className = 'text-selection-ai-button';
    selectionButton.style.position = 'absolute';
    selectionButton.style.top = `${rect.bottom + window.scrollY + 5}px`;
    selectionButton.style.left = `${rect.left + window.scrollX}px`;
    selectionButton.style.zIndex = '9999';
    selectionButton.style.padding = '8px 12px';
    selectionButton.style.backgroundColor = '#007bff';
    selectionButton.style.color = 'white';
    selectionButton.style.border = 'none';
    selectionButton.style.borderRadius = '4px';
    selectionButton.style.cursor = 'pointer';
    selectionButton.style.fontSize = '14px';

    selectionButton.onclick = () => {
      onSelection(selectedText);
      if (selectionButton) {
        selectionButton.remove();
        selectionButton = null;
      }
      selection.removeAllRanges();
    };

    document.body.appendChild(selectionButton);
  }

  // Add event listeners
  document.addEventListener('mouseup', handleSelection);
  document.addEventListener('touchend', handleSelection);

  // Cleanup function
  return () => {
    document.removeEventListener('mouseup', handleSelection);
    document.removeEventListener('touchend', handleSelection);
    if (selectionButton) {
      selectionButton.remove();
    }
  };
}
```

#### localStorage Management (`utils/storage.ts`)

```typescript
const STORAGE_KEY_PREFIX = 'chatbot_conversation_';
const EXPIRY_DAYS = 7;

interface StoredConversation {
  conversation_id: string;
  messages: Array<any>;
  updated_at: string;
}

export function saveConversation(sessionId: string, data: StoredConversation): void {
  const key = `${STORAGE_KEY_PREFIX}${sessionId}`;
  const expiry = new Date();
  expiry.setDate(expiry.getDate() + EXPIRY_DAYS);
  
  const stored = {
    ...data,
    expiry: expiry.toISOString()
  };
  
  localStorage.setItem(key, JSON.stringify(stored));
}

export function loadConversation(sessionId: string): StoredConversation | null {
  const key = `${STORAGE_KEY_PREFIX}${sessionId}`;
  const stored = localStorage.getItem(key);
  
  if (!stored) return null;
  
  try {
    const data = JSON.parse(stored);
    
    // Check expiry
    if (new Date(data.expiry) < new Date()) {
      localStorage.removeItem(key);
      return null;
    }
    
    return data;
  } catch {
    return null;
  }
}
```

---

### 1.5 Deployment Architecture

#### Backend Deployment (Vercel)

**`vercel.json` configuration**:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "backend/main.py",
      "use": "@vercel/python",
      "config": {
        "maxLambdaSize": "15mb"
      }
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "backend/main.py"
    },
    {
      "src": "/(.*)",
      "dest": "backend/main.py"
    }
  ],
  "env": {
    "OPENAI_API_KEY": "@openai-api-key",
    "QDRANT_URL": "@qdrant-url",
    "QDRANT_API_KEY": "@qdrant-api-key",
    "DATABASE_URL": "@database-url"
  }
}
```

**Deployment steps**:
```bash
# Install Vercel CLI
npm install -g vercel

# Login
vercel login

# Set environment variables
vercel env add OPENAI_API_KEY
vercel env add QDRANT_URL
vercel env add QDRANT_API_KEY
vercel env add DATABASE_URL

# Deploy
vercel --prod
```

#### Frontend Deployment (GitHub Pages)

Already configured via GitHub Actions. Add environment variable:

```yaml
# .github/workflows/deploy.yml
env:
  REACT_APP_CHAT_API_URL: ${{ secrets.CHAT_API_URL }}
```

---

### 1.6 Agentic Architecture (Alternative to Manual RAG)

#### Why Agentic vs Manual Pipeline

**Manual RAG Pipeline** (Current Plan):
- Linear flow: Query → Embed → Search → Context → LLM → Response
- Always searches vector DB for every query
- Hardcoded retrieval strategy

**Agentic Architecture** (Enhanced Approach):
- Agent autonomously decides: "Do I need to search? Or can I answer directly?"
- Multi-turn reasoning: Agent can search multiple times in one conversation
- Extensible: Easy to add new tools (calculator, code runner, quiz generator)
- Better error handling: Agent can retry or use fallback strategies

#### Teacher Agent with Search Tool

[`backend/agent.py`](backend/agent.py) - Define agent with textbook search tool:

```python
from agents import Agent, function_tool, RunContextWrapper
from services.vector_search import VectorSearchService
from services.embedding_service import EmbeddingService

@function_tool(
    description_override=(
        "Search the Physical AI & Robotics textbook for relevant information. "
        "Use this when the user asks about ROS 2, Gazebo, Navigation, or AI Integration topics."
    )
)
async def search_textbook(ctx: RunContextWrapper, query: str) -> str:
    """The agent autonomously calls this when it needs textbook info"""
    # Generate embedding
    embedding_service = EmbeddingService(api_key=settings.openai_api_key)
    query_embedding = await embedding_service.generate_embedding(query)
    
    # Search Qdrant
    vector_search = VectorSearchService(...)
    chunks = await vector_search.search(query_vector=query_embedding, top_k=5)
    
    # Format results for agent
    results = []
    for i, chunk in enumerate(chunks, 1):
        results.append(
            f"\n--- Result {i} (relevance: {chunk['score']:.2f}) ---\n"
            f"Chapter: {chunk['chapter']}\nLesson: {chunk['lesson']}\n"
            f"Content:\n{chunk['content']}\n"
        )
    return "\n".join(results)

# Define Teacher Agent
teacher_agent = Agent(
    name="Teacher",
    model=settings.openai_model,  # Configurable via .env (gpt-3.5-turbo, gpt-4, etc.)
    instructions="""You are an AI teaching assistant for the Physical AI & Robotics textbook.
    
Use the search_textbook tool when students ask about specific topics.
Provide clear, educational explanations based on retrieved content.
Cite specific chapters/lessons when answering.""",
    tools=[search_textbook],
    temperature=settings.openai_temperature,  # Also from .env
    max_tokens=settings.openai_max_tokens,   # Also from .env
)
```

#### Agentic Loop vs RAG Pipeline

**Data Flow Comparison:**

```
MANUAL RAG (Current):
User Query → Embedding → Vector Search → Top-5 Chunks → LLM → Response

AGENTIC (Enhanced):
User Query → Agent Reasoning → [Decide: Search needed?]
                ↓ (if yes)
          Tool Call: search_textbook(query)
                ↓
          Qdrant Search → Top-5 Chunks
                ↓
          Tool Output returned to Agent
                ↓
          Agent synthesizes answer + sources
                ↓
          Response streamed to user
```

#### Integration with Current Plan

The agentic approach can be implemented **in parallel** with the manual RAG:

1. **Phase 4 (RAG Pipeline)**: Implement as planned (manual pipeline)
2. **Phase 4.5 (NEW)**: Add agentic layer on top:
   - Implement `teacher_agent` (1 hour)
   - Wrap manual RAG as tool (30 min)
   - Test agent decision-making (30 min)
3. **Phase 5 (Frontend)**: Can use either:
   - **Option A**: Manual fetch API (current plan)
   - **Option B**: ChatKit React (if ChatKit SDK becomes available)

This gives you **both approaches**: use manual RAG for now, upgrade to agentic when ready.

---

## 2. Implementation Phases

### Phase 1: Environment Setup (30-45 min)

**Tasks**:
1. Create API accounts
   - OpenAI: Get API key from platform.openai.com
   - Qdrant Cloud: Create cluster at cloud.qdrant.io
   - Neon: Create serverless Postgres at neon.tech
   - Vercel: Sign up at vercel.com

2. Backend project structure
   ```bash
   mkdir backend
   cd backend
   python -m venv venv
   source venv/bin/activate  # Windows: venv\Scripts\activate
   pip install fastapi uvicorn[standard] openai qdrant-client asyncpg pydantic-settings pyyaml tiktoken
   pip freeze > requirements.txt
   ```

3. Create `.env` file
   ```env
   OPENAI_API_KEY=sk-...
   QDRANT_URL=https://xxx.qdrant.io
   QDRANT_API_KEY=...
   DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
   ```

4. Test connections
   ```python
   # test_connections.py
   import asyncio
   from openai import AsyncOpenAI
   from qdrant_client import AsyncQdrantClient
   import asyncpg

   async def test_all():
       # Test OpenAI
       client = AsyncOpenAI(api_key="...")
       response = await client.chat.completions.create(
           model="gpt-3.5-turbo",
           messages=[{"role": "user", "content": "Hello"}]
       )
       print("✓ OpenAI connected")
       
       # Test Qdrant
       qdrant = AsyncQdrantClient(url="...", api_key="...")
       collections = await qdrant.get_collections()
       print("✓ Qdrant connected")
       
       # Test Neon
       conn = await asyncpg.connect("...")
       version = await conn.fetchval("SELECT version()")
       await conn.close()
       print("✓ Neon connected")

   asyncio.run(test_all())
   ```

---

### Phase 2: Backend Foundation (3-4 hours)

**Tasks**:
1. Implement configuration module (`config.py`)
2. Create database schema (`scripts/init_db.py`)
3. Implement database service with connection pooling (`services/db_service.py`)
4. Create Pydantic models (`models/schemas.py`)
5. Implement health check endpoint (`routers/health.py`)
6. Set up FastAPI app with CORS (`main.py`)
7. Local testing with `uvicorn main:app --reload`

**Validation**:
- `GET /api/health` returns status for all services
- CORS headers present in responses
- Database connection pool initialized on startup

---

### Phase 3: Content Indexing (2-3 hours)

**Tasks**:
1. Implement markdown parser (`scripts/index_content.py`)
2. Implement chunking algorithm
3. Create Qdrant collection
4. Generate embeddings for all chunks
5. Upsert chunks to Qdrant
6. Verify indexing completeness

**Script execution**:
```bash
python scripts/index_content.py --docs-dir ../book-source/docs --collection textbook_content
```

**Expected output**:
- ~600 chunks indexed
- All chunks have metadata (chapter, lesson, heading, url)
- Vector search works: `python scripts/test_search.py "What is ROS 2?"`

---

### Phase 4: RAG Pipeline (4-5 hours)

**Tasks**:
1. Implement vector search service (`services/vector_search.py`)
2. Implement LLM service (`services/llm_service.py`)
3. Implement chat endpoint (`routers/chat.py`)
   - Query embedding generation
   - Vector search (top-5 chunks)
   - Context building
   - LLM generation
   - Source extraction
   - Database logging
4. Implement text-selection endpoint (`/api/chat/selection`)
5. Add error handling and rate limiting
6. Integration testing

**Testing**:
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2 Humble?",
    "session_id": "test-session"
  }'
```

**Expected response**:
```json
{
  "message": "ROS 2 Humble is...",
  "sources": [
    {
      "chapter_name": "chapter1",
      "lesson_title": "Introduction to ROS 2",
      "url": "https://devhammad0.github.io/physical-ai-robotics-textbook/chapter1/intro"
    }
  ],
  "conversation_id": "uuid",
  "response_time_ms": 1500
}
```

---

### Phase 5: Frontend Integration (3-4 hours)

**Tasks**:
1. Create ChatWidget component
2. Create MessageList, MessageInput, SourceCitation subcomponents
3. Implement API integration (`utils/api.ts`)
4. Implement localStorage persistence (`utils/storage.ts`)
5. Implement text selection handler
6. Add styles with light/dark mode support
7. Integrate into Docusaurus Root.tsx
8. Local testing

**Docusaurus integration** (`src/theme/Root.tsx`):
```tsx
import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

---

### Phase 6: Deployment (2-3 hours)

**Tasks**:
1. Deploy backend to Vercel
   ```bash
   vercel --prod
   ```
2. Update frontend `.env` with backend URL
3. Deploy frontend via GitHub Actions
4. End-to-end testing on production
5. Performance validation (<2s response time)

**Production URLs**:
- Backend: `https://rag-chatbot-api.vercel.app`
- Frontend: `https://devhammad0.github.io/physical-ai-robotics-textbook/`

---

### Phase 7: Polish & Testing (1-2 hours)

**Tasks**:
1. Error handling refinement
2. Loading states & animations
3. Mobile responsiveness testing
4. Accessibility (keyboard navigation, ARIA labels)
5. Demo preparation
   - Test 5 different question types
   - Verify text selection works
   - Check conversation history persistence
   - Test out-of-scope question handling

---

### Phase 8 (OPTIONAL): Agentic Enhancement (2-3 hours)

**Prerequisites**: Phases 1-7 complete (manual RAG working)

**Tasks**:
1. Install OpenAI Agents SDK:
   ```bash
   pip install openai-agents
   ```
2. Create `backend/agent.py` with Teacher Agent + search_textbook tool
3. Create `backend/routers/agent_chat.py` (alternative endpoint):
   ```python
   @router.post("/api/agent/chat")
   async def agent_chat(request: ChatRequest):
       from agents import Runner
       from agent import teacher_agent
       
       # Run agent with streaming
       result = Runner.run_streamed(teacher_agent, request.query)
       
       # Process agent output
       async for event in result.stream_events():
           if event.type == "message_output_item":
               # Extract final message
               pass
       
       return response
   ```
4. Test agent vs manual RAG:
   - Compare response quality
   - Measure latency difference
   - Verify agent calls tool correctly
5. Frontend: Add toggle to switch between `/api/chat` (manual) and `/api/agent/chat` (agentic)

**Benefits**:
- Agent decides when to search (saves vector DB calls for simple queries)
- Multi-turn reasoning (agent can search multiple times)
- Extensible (add more tools: calculator, code runner)

**Trade-offs**:
- Agent reasoning adds ~0.5-1s latency
- ~100-200 extra tokens per query ($0.0001-0.0002 more per query)
- Still within Vercel 10s timeout and <$6 budget

---

## 3. Risk Analysis & Mitigation

### Risk 1: OpenAI API Rate Limits
- **Likelihood**: Medium
- **Impact**: High (chatbot stops working)
- **Mitigation**:
  - Set usage limits in OpenAI dashboard ($5 hard limit)
  - Implement exponential backoff retry (max 3 attempts)
  - Fallback to retrieval-only mode if LLM fails
  - Log all rate limit errors for monitoring
- **Detection**: HTTP 429 status code, log quota warnings

**Implementation**:
```python
async def generate_with_retry(self, messages, max_retries=3):
    for attempt in range(max_retries):
        try:
            response = await self.client.chat.completions.create(...)
            return response
        except openai.RateLimitError as e:
            if attempt == max_retries - 1:
                logger.error("Rate limit exceeded, falling back")
                return None
            wait_time = 2 ** attempt
            await asyncio.sleep(wait_time)
```

### Risk 2: Vercel Cold Starts
- **Likelihood**: High (free tier)
- **Impact**: Medium (first request takes 2-5s)
- **Mitigation**:
  - Accept tradeoff for free tier
  - Show "Connecting..." message on first use
  - Set max-lambda-size to 15mb (faster cold starts)
- **Alternative**: Upgrade to Vercel Pro ($20/month) if budget allows

### Risk 3: CORS Configuration Issues
- **Likelihood**: Medium
- **Impact**: High (frontend can't call backend)
- **Mitigation**:
  - Test CORS with curl before frontend integration
  - Whitelist exact GitHub Pages URL: `https://devhammad0.github.io`
  - Include OPTIONS method for preflight requests
- **Detection**: Browser console errors, OPTIONS preflight failures

**Testing**:
```bash
curl -X OPTIONS https://api.example.com/api/chat \
  -H "Origin: https://devhammad0.github.io" \
  -H "Access-Control-Request-Method: POST" \
  -v
```

### Risk 4: Qdrant Free Tier Storage Limits
- **Likelihood**: Low
- **Impact**: Medium (can't index all content)
- **Mitigation**:
  - Optimize chunking (reduce overlap: 100 tokens instead of 200)
  - Monitor collection size in Qdrant dashboard
  - Estimate: 600 chunks × 1536 dimensions × 4 bytes = ~4MB (well under 1GB)
- **Current estimate**: 600 chunks = ~4MB (well under 1GB limit)

### Risk 5: Token Limit Overflow (GPT-3.5-turbo 16K tokens)
- **Likelihood**: Medium (long conversations + large context)
- **Impact**: Medium (request fails)
- **Mitigation**:
  - Implement token counting with tiktoken library
  - Truncate context if exceeds 12K tokens (4K buffer)
  - Prioritize recent messages over old ones
  - Limit conversation history to 5 messages
- **Detection**: OpenAI API error "context_length_exceeded"

**Implementation**:
```python
import tiktoken

encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")

def count_tokens(text: str) -> int:
    return len(encoding.encode(text))

def truncate_context(context: str, max_tokens: int = 3000) -> str:
    tokens = encoding.encode(context)
    if len(tokens) <= max_tokens:
        return context
    return encoding.decode(tokens[:max_tokens])
```

---

## 4. Architecture Decision Records (ADRs)

### ADR-001: LLM Model Selection

**Context**: Need to choose between GPT-3.5-turbo and GPT-4 for answer generation.

**Decision**: Use **GPT-3.5-turbo** ($0.0005/1K input tokens, $0.0015/1K output tokens)

**Rationale**:
- 15x cheaper than GPT-4 ($0.01/1K input tokens)
- Faster responses (~1s vs ~3s)
- Sufficient quality with provided context (RAG reduces hallucination)
- Budget-friendly for hackathon ($5 covers ~3K queries)

**Tradeoffs**:
- GPT-4 has better reasoning and fewer hallucinations
- GPT-3.5 may struggle with complex multi-step questions
- Acceptable tradeoff for MVP scope

**Consequences**:
- Monitor answer quality manually during testing
- Can upgrade to GPT-4 later if budget allows
- Model is configurable via `OPENAI_MODEL` env variable (change in one place: `.env`)

---

### ADR-002: Chunking Strategy

**Context**: Need to split markdown content into searchable chunks.

**Decision**: **Heading-based chunking** with 800 token max, 100 token overlap

**Rationale**:
- Preserves natural semantic boundaries (respects markdown structure)
- Each chunk corresponds to a meaningful section
- Better context for LLM than arbitrary splits
- Overlap ensures context continuity

**Alternatives Considered**:
- **Fixed-token splits**: Simple but breaks semantic boundaries
- **Semantic chunking (LangChain)**: More accurate but slower and complex
- **Paragraph-based**: Too granular, loses heading context

**Tradeoffs**:
- Variable chunk sizes (200-800 tokens)
- May split very long sections into multiple chunks
- Acceptable for textbook structure (well-organized headings)

**Consequences**:
- Each chunk includes heading metadata for context
- Source citations point to section level (not arbitrary character positions)

---

### ADR-003: Conversation Storage

**Context**: Need to persist conversation history for multi-turn conversations.

**Decision**: **localStorage** for anonymous users, **Postgres** for authenticated (bonus)

**Rationale**:
- Zero backend cost for anonymous users
- Instant persistence (no network latency)
- Privacy-friendly (data stays on device)
- 7-day auto-expiry via JavaScript

**Alternatives Considered**:
- **Backend-only storage**: Requires authentication, slower, costs money
- **SessionStorage**: Lost on tab close (bad UX)
- **Cookies**: 4KB limit, complex expiry management

**Tradeoffs**:
- Not synced across devices (acceptable for MVP)
- Lost if user clears browser data
- 5-10MB localStorage limit (enough for ~1000 messages)

**Consequences**:
- Frontend manages persistence logic
- Backend logs all queries for analytics
- Upgrade path: sync to Postgres when user authenticates

---

### ADR-004: Backend Hosting

**Context**: Need to host FastAPI backend with async capabilities.

**Decision**: **Vercel Serverless Functions**

**Rationale**:
- Zero-config deployment (`vercel --prod`)
- Auto-scaling (handle traffic spikes)
- Free HTTPS and custom domains
- Native Python support (@vercel/python)
- One-command deploy workflow

**Alternatives Considered**:
- **Railway**: $5/month minimum, better for long-running processes
- **Render**: Free tier has 15-minute spin-down (unacceptable)
- **AWS Lambda**: Complex setup, requires API Gateway config

**Tradeoffs**:
- Cold starts (2-5s first request after idle)
- 10s execution timeout (acceptable for <2s target)
- Stateless (can't maintain persistent connections)

**Consequences**:
- Accept cold start latency for first user
- Design stateless request handlers
- Use Neon Postgres connection pooling efficiently

---

### ADR-005: Source Citation Strategy

**Context**: Need to provide source citations for LLM answers.

**Decision**: **Top-3 unique (chapter, lesson) pairs** from retrieved chunks

**Rationale**:
- Balances comprehensiveness vs UI clutter
- Encourages exploration (clickable links to lessons)
- Deduplicates chunks from same lesson
- Matches natural textbook structure

**Alternatives Considered**:
- **All retrieved chunks**: Too many links (5+), overwhelms UI
- **Single top source**: May miss relevant sections
- **Top-5 with duplicates**: Redundant if 3 chunks from same lesson

**Tradeoffs**:
- May miss some relevant sections (mitigated by retrieval quality)
- Users must click to see full section (acceptable)

**Consequences**:
- Source list shows max 3 items
- Each source is clickable link to Docusaurus page
- Relevance score shown for transparency

---

### ADR-006: Agentic Architecture (Optional Enhancement)

**Context**: Decide whether to implement agent-based reasoning on top of manual RAG pipeline.

**Decision**: **Implement as optional Phase 8** (after manual RAG is working)

**Rationale**:
- OpenAI Agents SDK provides autonomous tool calling (agent decides when to search)
- Better for multi-turn conversations (agent remembers context naturally)
- Future-proof: easy to add new tools (calculator, quiz generator, code runner)
- Can compare agent vs manual performance side-by-side

**Implementation Strategy**:
- Phase 1-7: Build manual RAG (proven, predictable)
- Phase 8: Add agentic layer as alternative endpoint (`/api/agent/chat`)
- Frontend: Toggle between manual and agent modes
- Production: Start with manual, switch to agent after validation

**Alternatives Considered**:
- **Agent-only (no manual RAG)**: Risky - agent might not work well, no fallback
- **ChatKit SDK integration**: Wait for ChatKit Python SDK to be released (currently placeholder)

**Tradeoffs**:
- **Benefit**: Autonomous decision-making, better conversations, extensible
- **Cost**: +0.5-1s latency, +$0.0001-0.0002 per query, learning curve
- **Risk**: Agent might make wrong tool calls (mitigated by clear tool descriptions)

**Consequences**:
- Deliver manual RAG first (lower risk, proven approach)
- Evaluate agent enhancement in Phase 8
- Can remove agent if it doesn't improve experience
- Provides clear comparison for future projects

---

## 5. Testing Strategy

### Unit Tests

**Test Chunking Algorithm**:
```python
def test_chunking():
    chunker = ContentChunker(max_tokens=800, overlap_tokens=100)
    sections = [{'heading': 'Test', 'content': 'A' * 2000}]
    chunks = chunker.chunk_sections(sections)
    
    assert len(chunks) > 1  # Should split long section
    assert all(c['token_count'] <= 800 for c in chunks)  # Max token limit
    assert chunks[0]['heading'].startswith('Test')  # Heading preserved
```

**Test Token Counting**:
```python
def test_token_counting():
    chunker = ContentChunker()
    text = "Hello world this is a test"
    tokens = chunker.count_tokens(text)
    assert tokens > 0
    assert tokens < len(text.split())  # Tokens < words (subword encoding)
```

**Test URL Generation**:
```python
def test_url_generation():
    chunk = {
        'chapter': 'chapter1',
        'lesson': 'intro',
        'file_path': 'docs/chapter1/intro.md'
    }
    url = generate_docusaurus_url(chunk)
    assert url == 'https://devhammad0.github.io/physical-ai-robotics-textbook/chapter1/intro'
```

---

### Integration Tests

**Test Vector Search**:
```python
async def test_vector_search():
    search = VectorSearchService(url, api_key, collection)
    query_vector = [0.1] * 1536  # Mock embedding
    results = await search.search(query_vector, top_k=5)
    
    assert len(results) <= 5
    assert all('score' in r for r in results)
    assert all('chapter' in r for r in results)
```

**Test LLM Service**:
```python
async def test_llm_generation():
    llm = LLMService(api_key)
    chunks = [
        {'chapter': 'ch1', 'lesson': 'intro', 'heading': 'ROS 2', 
         'content': 'ROS 2 is...', 'score': 0.9}
    ]
    response = await llm.generate_response(
        query="What is ROS 2?",
        retrieved_chunks=chunks
    )
    
    assert response is not None
    assert 'ROS 2' in response
    assert len(response) < 1000  # Reasonable length
```

**Test Database Operations**:
```python
async def test_conversation_crud():
    pool = await asyncpg.create_pool(...)
    service = ConversationService(pool)
    
    # Create conversation
    conv_id = await service.create_or_get_conversation(session_id="test")
    assert conv_id is not None
    
    # Add message
    msg_id = await service.add_message(
        conv_id, role="user", content="Test query"
    )
    assert msg_id is not None
    
    # Get history
    history = await service.get_conversation_history(conv_id)
    assert len(history) == 1
    assert history[0]['content'] == "Test query"
```

---

### End-to-End Tests

**Test General Q&A Flow**:
```python
async def test_chat_endpoint():
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/api/chat", json={
            "query": "What is ROS 2?",
            "session_id": "test-session"
        })
        
        assert response.status_code == 200
        data = response.json()
        assert 'message' in data
        assert 'sources' in data
        assert len(data['sources']) > 0
        assert 'conversation_id' in data
        assert data['response_time_ms'] < 3000
```

**Test Text-Selection Flow**:
```python
async def test_text_selection():
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/api/chat", json={
            "query": "Explain this",
            "session_id": "test-session",
            "selected_text": "Behavior trees enable hierarchical decision-making"
        })
        
        assert response.status_code == 200
        data = response.json()
        assert 'behavior tree' in data['message'].lower()
```

**Test Conversation History**:
```python
async def test_conversation_history():
    session_id = "test-session"
    
    # First message
    response1 = await send_message("What is URDF?", session_id)
    conv_id = response1['conversation_id']
    
    # Follow-up message (should use history)
    response2 = await send_message("Show me an example", session_id, conv_id)
    assert 'URDF' in response2['message'] or 'robot' in response2['message']
```

**Test Error Scenarios**:
```python
async def test_rate_limiting():
    # Send 31 requests in quick succession
    session_id = "test-session"
    for i in range(31):
        response = await send_message(f"Test {i}", session_id)
        if i >= 30:
            assert response.status_code == 429  # Rate limited

async def test_empty_query():
    response = await send_message("", "test-session")
    assert response.status_code == 422  # Validation error

async def test_long_query():
    long_query = "A" * 1001
    response = await send_message(long_query, "test-session")
    assert response.status_code == 422  # Exceeds max length
```

---

### Performance Tests

**Response Time Test**:
```python
import time

async def test_response_time():
    queries = [
        "What is ROS 2?",
        "How do I create a Gazebo world?",
        "What are behavior trees?",
        # ... 20 more queries
    ]
    
    times = []
    for query in queries:
        start = time.time()
        await send_message(query, "perf-test")
        end = time.time()
        times.append((end - start) * 1000)
    
    p90 = sorted(times)[int(len(times) * 0.9)]
    p99 = sorted(times)[int(len(times) * 0.99)]
    
    assert p90 < 2000, f"P90 latency {p90}ms exceeds 2s target"
    assert p99 < 3000, f"P99 latency {p99}ms exceeds 3s acceptable limit"
```

**Concurrent Users Test**:
```python
import asyncio

async def test_concurrent_users():
    async def user_session(user_id):
        session_id = f"load-test-{user_id}"
        for i in range(5):  # 5 queries per user
            await send_message(f"Query {i}", session_id)
    
    # 50 concurrent users
    tasks = [user_session(i) for i in range(50)]
    start = time.time()
    await asyncio.gather(*tasks)
    end = time.time()
    
    total_queries = 50 * 5
    avg_time = (end - start) / total_queries
    assert avg_time < 2.0, "Average response time exceeds 2s under load"
```

**Vector Search Speed Test**:
```python
async def test_vector_search_speed():
    search = VectorSearchService(...)
    query_vector = [0.1] * 1536
    
    times = []
    for i in range(100):
        start = time.time()
        await search.search(query_vector, top_k=5)
        end = time.time()
        times.append((end - start) * 1000)
    
    avg_time = sum(times) / len(times)
    assert avg_time < 200, f"Average search time {avg_time}ms exceeds 200ms"
```

---

### Manual Tests

**Mobile Responsiveness**:
- [ ] Chat widget opens full-screen on mobile (<768px)
- [ ] Touch selection works for "Ask AI" button
- [ ] Scroll behavior works correctly on small screens
- [ ] Text input resizes for on-screen keyboard

**Accessibility**:
- [ ] Tab key navigates through all interactive elements
- [ ] Enter key sends message
- [ ] Escape key closes chat widget
- [ ] Screen reader announces new messages
- [ ] ARIA labels present on all buttons

**Cross-Browser**:
- [ ] Chrome (Windows/Mac)
- [ ] Firefox (Windows/Mac)
- [ ] Safari (Mac/iOS)
- [ ] Edge (Windows)

---

## 6. Success Criteria

### Measurable Outcomes

✅ **SC-001**: Students can ask any question and receive answer with citations in <2s (p90)  
✅ **SC-002**: 90%+ answer relevance (validated via 20 test questions)  
✅ **SC-003**: Text selection feature works on any page  
✅ **SC-004**: Conversation history persists across page navigation and refresh  
✅ **SC-005**: All 35+ markdown lessons indexed (~600 chunks)  
✅ **SC-006**: Vector search returns top-5 chunks in <200ms  
✅ **SC-007**: System handles 50+ concurrent users without degradation  
✅ **SC-008**: 99% uptime during demo period  
✅ **SC-009**: Chat widget loads without blocking page render  
✅ **SC-010**: Mobile responsive (full-screen on small devices)  
✅ **SC-011**: Full keyboard navigation support  
✅ **SC-012**: Health check endpoint reports "healthy" for all services  
✅ **SC-013**: Demo completes in <2 minutes  
✅ **SC-014**: Zero console errors during normal operation  
✅ **SC-015**: Out-of-scope questions receive graceful redirection  
✅ **SC-016**: Total cost under $6 for hackathon period  

---

## 7. Definition of Done

A task is considered **DONE** when:

1. **Code Complete**: All planned code is written and follows best practices
2. **Tests Pass**: Unit, integration, and E2E tests pass
3. **Deployed**: Code is deployed to production environment
4. **Documented**: README updated, inline comments added
5. **Validated**: Manual testing confirms expected behavior
6. **Performance**: Meets latency and throughput requirements
7. **No Regressions**: Existing features still work
8. **Code Review**: PR approved by at least one reviewer (if team > 1)

---

## 8. Critical File List

### Backend Files

```
backend/
├── main.py                     # FastAPI app + CORS + lifespan
├── config.py                   # Pydantic settings
├── requirements.txt            # Python dependencies
├── vercel.json                 # Vercel deployment config
├── .env.example                # Environment variable template
├── routers/
│   ├── __init__.py
│   ├── chat.py                 # Chat endpoints
│   └── health.py               # Health check endpoint
├── services/
│   ├── __init__.py
│   ├── vector_search.py        # Qdrant client + search
│   ├── llm_service.py          # OpenAI completions + embeddings
│   └── db_service.py           # AsyncPG connection pool + CRUD
├── models/
│   ├── __init__.py
│   ├── schemas.py              # Pydantic request/response models
│   └── database.py             # SQLAlchemy table definitions
└── scripts/
    ├── index_content.py        # Markdown parser + indexing
    ├── init_db.py              # Create Postgres tables
    └── test_connections.py     # Test API connectivity
```

### Frontend Files

```
book-source/
├── .env.example                # Frontend env vars
├── src/
│   ├── components/
│   │   └── ChatWidget/
│   │       ├── ChatWidget.tsx
│   │       ├── ChatWidget.module.css
│   │       ├── MessageList.tsx
│   │       ├── MessageInput.tsx
│   │       └── SourceCitation.tsx
│   ├── theme/
│   │   └── Root.tsx            # Docusaurus global wrapper
│   └── utils/
│       ├── api.ts              # Backend API calls
│       ├── storage.ts          # localStorage management
│       └── textSelection.ts    # Text selection handler
└── docusaurus.config.js        # Add env var for API URL
```

### Database Files

```
backend/scripts/schema.sql      # SQL DDL for all tables + indexes
```

### Deployment Files

```
vercel.json                     # Vercel config
.github/workflows/deploy.yml    # GitHub Actions workflow (update env vars)
```

---

## 9. Implementation Timeline

| Phase | Tasks | Duration | Dependencies |
|-------|-------|----------|--------------|
| **Phase 1** | Environment Setup | 30-45 min | None |
| **Phase 2** | Backend Foundation | 3-4 hours | Phase 1 |
| **Phase 3** | Content Indexing | 2-3 hours | Phase 2 |
| **Phase 4** | RAG Pipeline | 4-5 hours | Phase 3 |
| **Phase 5** | Frontend Integration | 3-4 hours | Phase 4 |
| **Phase 6** | Deployment | 2-3 hours | Phase 5 |
| **Phase 7** | Polish & Testing | 1-2 hours | Phase 6 |
| **Total** |  | **16-22 hours** | Sequential |

**Estimated completion**: 2-3 full working days for one developer

---

## 10. Cost Breakdown

| Service | Tier | Cost | Usage Estimate |
|---------|------|------|----------------|
| OpenAI API | Pay-as-you-go | ~$3 | 3K queries (embeddings + completions) |
| Qdrant Cloud | Free | $0 | <1GB storage, <1M API calls |
| Neon Postgres | Free | $0 | <0.5GB storage, <10K queries |
| Vercel | Hobby | $0 | <100GB bandwidth |
| GitHub Pages | Free | $0 | Static hosting |
| **Total** |  | **~$3-6** | Hackathon demo period |

**Notes**:
- OpenAI cost is variable based on query volume
- Set hard limit at $5 in OpenAI dashboard
- All other services remain on free tier
- Production scale would require paid plans (~$50/month)

---

## 11. Next Steps

After plan approval, proceed with:

1. **Create task breakdown** (`/sp.tasks`)
2. **Set up development environment** (Phase 1)
3. **Begin implementation** (Phases 2-7)
4. **Deploy to production** (Phase 6)
5. **Prepare demo** (Phase 7)

---

## Appendix A: API Endpoint Specifications

### `POST /api/chat`

**Description**: Send chat query and receive AI-generated response with sources

**Request**:
```json
{
  "query": "What is ROS 2 Humble?",
  "conversation_id": "uuid-optional",
  "session_id": "uuid-required",
  "selected_text": "optional-highlighted-text"
}
```

**Response** (200 OK):
```json
{
  "message": "ROS 2 Humble is the 8th release...",
  "sources": [
    {
      "chapter_name": "Chapter 1: ROS 2 Basics",
      "lesson_title": "Introduction to ROS 2",
      "section_heading": "What is ROS 2?",
      "url": "https://devhammad0.github.io/.../chapter1/intro",
      "relevance_score": 0.92
    }
  ],
  "conversation_id": "uuid",
  "response_time_ms": 1523
}
```

**Error Responses**:
- `400 Bad Request`: Invalid request format
- `422 Unprocessable Entity`: Validation error (empty query, too long)
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Backend processing error

---

### `GET /api/health`

**Description**: Check health status of all backend services

**Response** (200 OK):
```json
{
  "status": "healthy",
  "database": "connected",
  "vector_db": "connected",
  "llm_api": "connected",
  "timestamp": "2025-11-29T10:30:00Z"
}
```

**Error Response** (503 Service Unavailable):
```json
{
  "status": "unhealthy",
  "database": "connected",
  "vector_db": "error: connection timeout",
  "llm_api": "connected",
  "timestamp": "2025-11-29T10:30:00Z"
}
```

---

## Appendix B: Environment Variables

### Backend (`.env`)

```env
# OpenAI
OPENAI_API_KEY=sk-proj-...
OPENAI_MODEL=gpt-3.5-turbo  # Change to gpt-4, gpt-4o, gpt-4o-mini to switch models
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_MAX_TOKENS=500
OPENAI_TEMPERATURE=0.7

# Qdrant
QDRANT_URL=https://xxx-xxx-xxx.qdrant.io
QDRANT_API_KEY=xxx
QDRANT_COLLECTION_NAME=textbook_content

# Neon Postgres
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
DB_POOL_MIN_SIZE=2
DB_POOL_MAX_SIZE=10

# Rate Limiting
RATE_LIMIT_PER_MINUTE=30

# CORS
CORS_ORIGINS=["https://devhammad0.github.io", "http://localhost:3000"]

# OpenAI Agents (Optional - Phase 8)
ENABLE_AGENT_MODE=false  # Toggle agent vs manual RAG
```

### Frontend (`.env`)

```env
REACT_APP_CHAT_API_URL=https://rag-chatbot-api.vercel.app
```

---

## Appendix C: Sample Test Questions

**General Q&A**:
1. What is ROS 2 Humble?
2. How do I create a Gazebo simulation world?
3. What are behavior trees used for?
4. Explain the difference between global and local planning
5. How do I integrate AI models with ROS 2?

**Text Selection**:
1. Highlight: "Behavior trees enable hierarchical decision-making"
   Query: "Explain this in simpler terms"
2. Highlight: "URDF files define robot kinematics"
   Query: "Show me an example"

**Multi-Turn**:
1. Q1: "What is URDF?"
   Q2: "Can you give an example?"
   Q3: "How do I visualize it in RViz?"

**Out-of-Scope**:
1. "What is quantum computing?" → Redirect message
2. "Who won the 2024 election?" → Redirect message

**Edge Cases**:
1. Empty query → Error message
2. 1001-character query → Validation error
3. 31st request in 1 minute → Rate limit error

---

## Appendix D: Agentic vs Manual RAG Comparison

### Performance Comparison

| Metric | Manual RAG | Agentic RAG | Winner |
|--------|-----------|-------------|---------|
| **Latency (p90)** | 1.5-2s | 2-2.5s | Manual (faster) |
| **Cost per query** | $0.0015 | $0.0017 | Manual (cheaper) |
| **Answer quality** | Good | Better | Agent (context-aware) |
| **Multi-turn conv** | Manual context | Natural | Agent (better) |
| **Extensibility** | Hardcoded | Add tools easily | Agent (flexible) |
| **Debugging** | Predictable | Agent decisions opaque | Manual (easier) |

### When to Use Each

**Use Manual RAG when:**
- Budget is tight (every penny counts)
- Latency must be <2s p90
- Simple Q&A only (no multi-turn)
- Easy debugging needed

**Use Agentic RAG when:**
- Quality > speed
- Multi-turn conversations important
- Want to add more tools later (calculator, quiz, code runner)
- Budget allows ~15% higher cost

**Note**: Both approaches support easy model switching via `OPENAI_MODEL` environment variable:
- Start with `gpt-3.5-turbo` (cheap, fast)
- Upgrade to `gpt-4o-mini` (balanced)
- Switch to `gpt-4o` or `gpt-4` (best quality)
- Change happens in one place: `.env` file

### Example Agent Interaction

**User:** "What is ROS 2 Humble?"

**Agent Internal Reasoning:**
```
The user is asking about ROS 2 Humble, which is textbook content.
I should use the search_textbook tool to get accurate information.
```

**Agent Action:**
```json
{
  "tool_call": "search_textbook",
  "arguments": {"query": "ROS 2 Humble"}
}
```

**Tool Output:**
```
--- Result 1 (relevance: 0.94) ---
Chapter: Chapter 1: ROS 2 Basics
Lesson: Introduction to ROS 2
Content: ROS 2 Humble is the 8th release...
```

**Agent Response:**
```
ROS 2 Humble is the 8th release of ROS 2, providing improved performance 
and stability. According to Chapter 1: Introduction to ROS 2, Humble includes...

Sources: Chapter 1: ROS 2 Basics - Introduction to ROS 2
```

**User:** "How do I install it?"

**Agent Internal Reasoning:**
```
The user is asking a follow-up about installation. Context from previous turn:
we were discussing ROS 2 Humble. I should search for installation instructions.
```

**Agent Action:**
```json
{
  "tool_call": "search_textbook",
  "arguments": {"query": "ROS 2 Humble installation"}
}
```

This demonstrates the agentic loop with conversation awareness.

---

**Plan Status**: ✅ Ready for Implementation  
**Next Command**: `/sp.tasks` to generate task breakdown
