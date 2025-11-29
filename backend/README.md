# RAG Chatbot Backend

FastAPI backend for the Physical AI & Robotics Textbook RAG chatbot.

## Architecture

- **FastAPI**: Modern async Python web framework
- **OpenAI Agents SDK**: Agentic AI framework with autonomous tool usage
- **OpenAI API**: GPT-3.5-turbo for generation, text-embedding-3-small for embeddings
- **Qdrant**: Vector database for semantic search
- **Neon Postgres**: Serverless PostgreSQL for conversation storage
- **Vercel**: Serverless deployment platform
- **uv**: Fast Python package installer and project manager

### Agent-Based RAG Pipeline

The chatbot uses the **OpenAI Agents SDK** to create an intelligent teaching assistant that autonomously decides when to search the textbook:

1. **Agent**: `teaching_agent` in `services/agent_service.py`
   - Configured with textbook-specific instructions
   - Has access to `search_textbook` function tool
   - Autonomously decides when to search vs. use prior knowledge

2. **Custom Session**: `PostgresSession` in `services/session_service.py`
   - Integrates conversation history with Postgres
   - Loads/saves messages automatically
   - Enables multi-turn conversations with context

3. **Function Tool**: `search_textbook`
   - Generates embeddings for search queries
   - Performs vector search in Qdrant
   - Returns formatted results to agent
   - Tracks results for structured source extraction

## Prerequisites

- **Python 3.10+**
- **uv** - Install from https://docs.astral.sh/uv/getting-started/installation/
  ```bash
  # Windows (PowerShell)
  powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

  # macOS/Linux
  curl -LsSf https://astral.sh/uv/install.sh | sh
  ```

## Setup

### 1. Install Dependencies

```bash
cd backend
uv sync
```

This automatically creates a virtual environment and installs all dependencies from `pyproject.toml`.

### 2. Configure Environment Variables

Create `.env` file from `.env.example`:

```bash
cp .env.example .env
```

Update `.env` with your API keys:
- Get OpenAI API key from https://platform.openai.com
- Create Qdrant Cloud cluster at https://cloud.qdrant.io
- Create Neon Postgres database at https://neon.tech

### 3. Initialize Database

```bash
uv run python scripts/init_db.py
```

### 4. Index Textbook Content

```bash
uv run python scripts/index_content.py --docs-dir ../book-source/docs
```

This will:
- Parse all markdown files in `book-source/docs/`
- Create ~600 content chunks
- Generate embeddings using OpenAI
- Upload to Qdrant vector database

### 5. Run Development Server

```bash
uv run uvicorn main:app --reload
```

Visit http://localhost:8000/docs for interactive API documentation.

## Deployment to Vercel

### 1. Install Vercel CLI

```bash
npm install -g vercel
```

### 2. Set Environment Variables

```bash
vercel env add OPENAI_API_KEY production
vercel env add OPENAI_MODEL production
vercel env add OPENAI_EMBEDDING_MODEL production
vercel env add OPENAI_MAX_TOKENS production
vercel env add OPENAI_TEMPERATURE production
vercel env add QDRANT_URL production
vercel env add QDRANT_API_KEY production
vercel env add QDRANT_COLLECTION_NAME production
vercel env add DATABASE_URL production
vercel env add DB_POOL_MIN_SIZE production
vercel env add DB_POOL_MAX_SIZE production
vercel env add RATE_LIMIT_PER_MINUTE production
vercel env add CORS_ORIGINS production
```

### 3. Create `requirements.txt` for Vercel

Vercel doesn't support `pyproject.toml` directly, so generate a requirements.txt:

```bash
uv pip compile pyproject.toml -o requirements.txt
```

### 4. Deploy

```bash
vercel --prod
```

After deployment, your API will be available at `https://your-project.vercel.app`

## API Endpoints

### POST /api/chat

Process chat query with RAG pipeline.

**Request:**
```json
{
  "query": "What is ROS 2 Humble?",
  "session_id": "uuid",
  "conversation_id": "optional-uuid",
  "selected_text": "optional highlighted text"
}
```

**Response:**
```json
{
  "message": "ROS 2 Humble is...",
  "sources": [
    {
      "chapter_name": "Chapter 1",
      "lesson_title": "Introduction",
      "section_heading": "ROS 2 Overview",
      "url": "https://...",
      "relevance_score": 0.95
    }
  ],
  "conversation_id": "uuid",
  "response_time_ms": 1500
}
```

### GET /api/health

Check backend services health.

**Response:**
```json
{
  "status": "healthy",
  "database": "connected",
  "vector_db": "connected",
  "llm_api": "configured",
  "timestamp": "2025-11-29T..."
}
```

## Project Structure

```
backend/
├── main.py              # FastAPI app with CORS and lifespan
├── config.py            # Pydantic settings
├── pyproject.toml       # uv project configuration
├── uv.lock              # Locked dependencies
├── vercel.json          # Vercel deployment config
├── .venv/               # Virtual environment (auto-created by uv)
├── routers/
│   ├── chat.py         # Chat endpoint with agent-based RAG
│   └── health.py       # Health check endpoint
├── services/
│   ├── agent_service.py    # OpenAI Agents SDK integration
│   ├── session_service.py  # PostgresSession for conversation history
│   ├── db_service.py       # AsyncPG connection pooling
│   ├── vector_search.py    # Qdrant client (used by search_textbook tool)
│   └── llm_service.py      # OpenAI client (used for embeddings)
├── models/
│   └── schemas.py      # Pydantic request/response models
└── scripts/
    ├── init_db.py      # Database initialization
    ├── index_content.py # Content indexing script
    └── schema.sql      # PostgreSQL schema
```

## Testing

### Start Development Server

```bash
uv run uvicorn main:app --reload
```

### Health Check

```bash
curl http://localhost:8000/api/health
```

### Chat Request

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2 Humble?",
    "session_id": "test-session"
  }'
```

**What Happens:**
1. Agent receives the query
2. Agent autonomously decides to call `search_textbook` tool
3. Tool searches Qdrant for relevant textbook content
4. Agent synthesizes answer from search results
5. Structured sources extracted and returned

**Agent Decision-Making:**
- Simple questions: Agent may answer without searching (uses prior knowledge)
- Specific textbook questions: Agent searches for accurate information
- Multi-turn conversations: Agent maintains context from previous messages

### Run Python Scripts

```bash
# Initialize database
uv run python scripts/init_db.py

# Index content
uv run python scripts/index_content.py --docs-dir ../book-source/docs

# Test with custom arguments
uv run python scripts/index_content.py --docs-dir ../book-source/docs --collection my_collection
```

## Troubleshooting

### uv Command Not Found

Ensure uv is installed and in your PATH:
```bash
# Check installation
uv --version

# Reinstall if needed
curl -LsSf https://astral.sh/uv/install.sh | sh  # Unix
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"  # Windows
```

### Virtual Environment Issues

```bash
# Remove and recreate virtual environment
rm -rf .venv
uv sync
```

### Database Connection Errors

- Verify `DATABASE_URL` in `.env`
- Check Neon dashboard for connection details
- Ensure database is not paused (free tier)
- Test connection: `uv run python scripts/init_db.py`

### Qdrant Connection Errors

- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Check Qdrant Cloud dashboard
- Ensure collection exists: `uv run python scripts/index_content.py`

### OpenAI API Errors

- Verify `OPENAI_API_KEY` is valid (starts with `sk-`)
- Check usage limits in OpenAI dashboard
- Monitor rate limiting (429 errors)
- Test: `uv run python -c "from config import settings; print(settings.openai_api_key[:10])"`

### Import Errors

If you get module import errors, ensure you're using `uv run`:
```bash
# ✓ Correct
uv run python scripts/init_db.py

# ✗ Wrong (won't find modules)
python scripts/init_db.py
```

## Performance

- **Target Latency**: <3s p90 for chat responses (includes agent reasoning time)
- **Vector Search**: <200ms for top-5 retrieval
- **Agent Iterations**: Typically 1-2 tool calls per query
- **Concurrent Users**: Handles 50+ simultaneous requests
- **Database Pool**: 2-10 connections (configurable)

## How Agent-Based RAG Works

### Traditional RAG vs. Agent-Based RAG

**Traditional RAG (Manual Pipeline):**
```
User Query → Always Embed → Always Search → Always Generate → Response
```
- Every query triggers vector search (even for simple questions)
- No reasoning about whether search is needed
- Fixed pipeline execution

**Agent-Based RAG (Current Implementation):**
```
User Query → Agent Reasoning → [Conditionally] search_textbook Tool → Generate → Response
```
- Agent decides when to search based on query complexity
- Can answer simple questions without retrieval
- Multiple tool calls possible for complex queries
- Better multi-turn conversation handling

### Example Scenarios

**Scenario 1: Simple Question (No Search)**
```
Query: "What does ROS stand for?"
Agent Decision: Answer directly (common knowledge)
Tool Calls: None
Response: "ROS stands for Robot Operating System..."
```

**Scenario 2: Textbook-Specific Question (Searches)**
```
Query: "How do I configure Nav2 stack parameters?"
Agent Decision: Search textbook for specific Nav2 configuration
Tool Calls: search_textbook("Nav2 stack parameters configuration")
Response: [Synthesized answer with textbook sources]
```

**Scenario 3: Multi-Turn Conversation**
```
Turn 1:
  Query: "Tell me about ROS 2 Humble"
  Agent: Searches textbook, provides overview

Turn 2:
  Query: "How do I install it?"
  Agent: Maintains context, searches for installation instructions
  Response: "To install ROS 2 Humble (from previous context)..."
```

## License

See root LICENSE file.
