# Physical AI & Robotics Textbook - Backend API

FastAPI backend providing RAG-powered AI chatbot for the Physical AI & Robotics interactive textbook.

## Tech Stack

- **FastAPI** - Async Python web framework
- **OpenAI Agents SDK** - Autonomous RAG agent with tool calling
- **Qdrant Cloud** - Vector database for semantic search
- **Neon Postgres** - Serverless conversation storage
- **Vercel** - Serverless deployment
- **uv** - Fast Python package manager

## Quick Start

### Prerequisites

- Python 3.10+
- [uv package manager](https://docs.astral.sh/uv/getting-started/installation/)

```bash
# Install uv (Windows)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# Install uv (macOS/Linux)
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Setup

1. **Install dependencies**
   ```bash
   cd backend
   uv sync
   ```

2. **Configure environment**
   ```bash
   cp .env.example .env
   # Edit .env with your API keys (see Configuration section)
   ```

3. **Initialize database**
   ```bash
   uv run python scripts/init_db.py
   ```

4. **Index textbook content**
   ```bash
   uv run python scripts/index_content.py --docs-dir ../book-source/docs
   ```
   Creates ~524 searchable content chunks from markdown files.

5. **Start development server**
   ```bash
   uv run uvicorn main:app --reload
   ```
   API docs available at http://localhost:8000/docs

## Configuration

Required environment variables in `.env`:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-...              # Get from https://platform.openai.com
OPENAI_MODEL=gpt-3.5-turbo              # Model for chat responses
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_MAX_TOKENS=500
OPENAI_TEMPERATURE=0.7

# Qdrant Vector Database
QDRANT_URL=https://xxx.qdrant.io       # Get from https://cloud.qdrant.io
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=textbook_content

# Neon Postgres
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require  # Get from https://neon.tech
DB_POOL_MIN_SIZE=2
DB_POOL_MAX_SIZE=10

# API Configuration
RATE_LIMIT_PER_MINUTE=30
CORS_ORIGINS=https://yourdomain.com,http://localhost:3000
```

## API Endpoints

### `POST /api/chat`
Chat with AI teaching assistant using RAG.

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

### `GET /api/health`
Health check for all services.

**Response:**
```json
{
  "status": "healthy",
  "database": "connected",
  "vector_db": "connected",
  "llm_api": "configured"
}
```

Full API documentation available at `/docs` endpoint.

## Architecture

### RAG Agent Flow

```
User Query → Agent (teaching_agent)
              ↓
         Evaluates question
              ↓
    Calls search_textbook tool
              ↓
    Qdrant vector search (524 chunks)
              ↓
    Returns top-5 relevant chunks
              ↓
    Agent synthesizes response
              ↓
    Response + Source citations
```

### Agent Guardrails

- **Scope limitation**: Only answers robotics/ROS 2/AI topics
- **RAG enforcement**: Always searches textbook for technical questions
- **Off-topic handling**: Politely declines non-robotics questions

### Project Structure

```
backend/
├── main.py                  # FastAPI app entry point
├── config.py                # Environment configuration
├── routers/                 # API endpoints
│   ├── chat.py             # RAG chat endpoint
│   └── health.py           # Health check
├── services/
│   ├── agent_service.py    # OpenAI Agent with RAG tool
│   ├── session_service.py  # Conversation persistence
│   ├── vector_search.py    # Qdrant integration
│   └── llm_service.py      # OpenAI embeddings
├── models/
│   └── schemas.py          # API request/response models
└── scripts/
    ├── init_db.py          # Database setup
    └── index_content.py    # Content indexing
```

## Deployment

### Deploy to Vercel

1. **Install Vercel CLI**
   ```bash
   npm install -g vercel
   ```

2. **Set environment variables via dashboard**
   - Go to https://vercel.com/dashboard
   - Select project → Settings → Environment Variables
   - Add all variables from `.env` file
   - Set for **Production** environment

3. **Deploy**
   ```bash
   vercel --prod
   ```

### Update Qdrant Cluster

If you created a new Qdrant cluster:

1. Update `.env` locally with new credentials
2. Reindex content: `uv run python scripts/index_content.py --docs-dir ../book-source/docs`
3. Update Vercel environment variables
4. Redeploy: `vercel --prod`

## Testing

```bash
# Start server
uv run uvicorn main:app --reload

# Health check
curl http://localhost:8000/api/health

# Test chat (should use RAG)
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "session_id": "test"}'

# Test off-topic (should decline)
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is the weather?", "session_id": "test"}'
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **uv command not found** | Reinstall: `curl -LsSf https://astral.sh/uv/install.sh \| sh` |
| **Virtual env issues** | Reset: `rm -rf .venv && uv sync` |
| **Database connection error** | Check `DATABASE_URL` in Neon dashboard; ensure DB not paused |
| **Qdrant 404 error** | Verify `QDRANT_URL` and `QDRANT_API_KEY`; rerun indexing script |
| **OpenAI API error** | Verify key starts with `sk-`; check usage limits at platform.openai.com |
| **Module import errors** | Always use `uv run python script.py`, not `python script.py` |
| **Agent not using RAG** | Check Vercel env vars updated; redeploy after changes |
| **Agent answers off-topic** | Latest code has guardrails; ensure deployed version is current |

### Verify Services

```bash
# Check all services
curl http://localhost:8000/api/health

# Expected: {"status":"healthy","database":"connected","vector_db":"connected","llm_api":"configured"}
```

### Test Qdrant Connection

```bash
uv run python scripts/test_qdrant.py
```

## Performance Metrics

- **Chat response**: <3s p90 (includes agent reasoning + RAG)
- **Vector search**: <200ms for top-5 results
- **Indexed content**: 524 chunks from 4 chapters
- **Concurrent requests**: 50+ simultaneous users

## Key Features

✅ **Autonomous RAG Agent** - Decides when to search textbook vs. use general knowledge
✅ **Scope Guardrails** - Only answers robotics/ROS 2/AI questions
✅ **Source Citations** - All responses include textbook references
✅ **Multi-turn Context** - Maintains conversation history
✅ **Text Selection Support** - Prioritizes user-highlighted content
✅ **Serverless Deployment** - Scales automatically on Vercel

## License

MIT License - See root LICENSE file

---

**Questions?** Check `/api/docs` for interactive API documentation or review the [troubleshooting](#troubleshooting) section.
