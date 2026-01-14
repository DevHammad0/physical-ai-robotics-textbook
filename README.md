# Physical AI & Robotics Textbook

An interactive, AI-powered textbook for learning robotics, ROS 2, simulation, and embodied intelligence with a RAG-powered teaching assistant.

**🌐 Live Site:** https://devhammad0.github.io/physical-ai-robotics-textbook

## Overview

This project combines a comprehensive robotics curriculum with an intelligent AI chatbot that helps students learn by answering questions based on the textbook content.

### Key Features

- **📚 Comprehensive Curriculum** - 4 chapters covering ROS 2, Gazebo simulation, autonomous navigation, and AI integration
- **🤖 AI Teaching Assistant** - RAG-powered chatbot that answers questions using textbook content
- **🌍 Multilingual Support** - Available in English and Urdu
- **✨ Interactive Learning** - Text selection support, conversation history, and source citations
- **🎯 Scope Guardrails** - Chatbot focused exclusively on robotics and AI topics
- **📱 Modern UI** - Built with Docusaurus, React, and TypeScript

## Project Structure

```
physical-ai-robotics-textbook/
├── book-source/          # Docusaurus frontend
│   ├── docs/            # Textbook content (Markdown)
│   ├── src/             # React components & pages
│   └── i18n/            # Translations (Urdu)
├── backend/             # FastAPI RAG chatbot
│   ├── routers/         # API endpoints
│   ├── services/        # Agent, vector search, LLM
│   ├── models/          # Pydantic schemas
│   └── scripts/         # DB init & content indexing
└── README.md           # This file
```

## Tech Stack

### Frontend (Docusaurus)
- **Docusaurus 4** - Documentation framework
- **React 19** - UI components
- **TypeScript** - Type safety
- **Spline** - 3D animations
- **Better Auth** - Authentication

### Backend (FastAPI)
- **FastAPI** - Async Python web framework
- **OpenAI Agents SDK** - Autonomous RAG agent
- **Qdrant Cloud** - Vector database (524 indexed chunks)
- **Neon Postgres** - Conversation storage
- **Vercel** - Serverless deployment

## Quick Start

### Prerequisites

- **Node.js 18+** and **pnpm**
- **Python 3.10+** and **uv**
- API keys: OpenAI, Qdrant, Neon Postgres

### 1. Frontend Setup

```bash
cd book-source

# Install dependencies
pnpm install

# Start development server
pnpm start
```

Visit http://localhost:3000

### 2. Backend Setup

```bash
cd backend

# Install dependencies
uv sync

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Initialize database
uv run python scripts/init_db.py

# Index textbook content
uv run python scripts/index_content.py --docs-dir ../book-source/docs

# Start API server
uv run uvicorn main:app --reload
```

Visit http://localhost:8000/docs for API documentation

## Curriculum

### Chapter 1: ROS 2 Fundamentals
Introduction to ROS 2 Humble, nodes, topics, publishers, subscribers, services, actions, and launch files.

### Chapter 2: Gazebo Modeling
Robot modeling with URDF, physics simulation, sensor integration, and multi-robot systems.

### Chapter 3: Autonomous Navigation
SLAM, visual navigation, Isaac Sim, Nav2 stack, obstacle avoidance, and sensor fusion.

### Chapter 4: AI Integration
Vision-Language-Action models, voice interfaces (Whisper), computer vision, LLM planning, manipulation, and safety systems.

## Deployment

### Frontend (GitHub Pages)

```bash
cd book-source
pnpm deploy
```

Deploys to: https://devhammad0.github.io/physical-ai-robotics-textbook

### Backend (Vercel)

```bash
cd backend

# Set environment variables in Vercel dashboard
# Deploy
vercel --prod
```

See `backend/README.md` for detailed deployment instructions.

## AI Chatbot

The RAG-powered teaching assistant:

✅ **Searches 524 indexed chunks** from the textbook
✅ **Provides source citations** for every answer
✅ **Maintains conversation context** across multiple turns
✅ **Scope-limited** to robotics, ROS 2, and AI topics
✅ **Politely declines** off-topic questions

### Example Queries

```
✓ "What is ROS 2 Humble?"
✓ "How do I create a publisher node?"
✓ "Explain autonomous navigation with Nav2"
✗ "What's the weather today?" (declined)
```

## Development

### Frontend Development

```bash
cd book-source
pnpm start              # Start dev server
pnpm build              # Build for production
pnpm serve              # Preview production build
pnpm deploy             # Deploy to GitHub Pages
```

### Backend Development

```bash
cd backend
uv run uvicorn main:app --reload              # Start with auto-reload
uv run python scripts/index_content.py        # Reindex content
uv run python scripts/test_qdrant.py          # Test Qdrant connection
```

### Environment Variables

See `.env.example` files in:
- `backend/.env.example` - Backend configuration
- `book-source/.env` - Frontend configuration

## Testing

### Backend API

```bash
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

## Documentation

- **Frontend:** See `book-source/README.md` (if exists)
- **Backend:** See `backend/README.md`
- **API Docs:** http://localhost:8000/docs (when running)

## Contributing

This project was created for a hackathon. Contributions are welcome!

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT License - See LICENSE file for details

## Acknowledgments

- **OpenAI** - GPT models and Agents SDK
- **Qdrant** - Vector database
- **Docusaurus** - Documentation framework
- **Vercel** - Serverless deployment
- **Neon** - Serverless Postgres

---

**Built for the Hackathon 2026** | [Live Demo](https://devhammad0.github.io/physical-ai-robotics-textbook) | [Report Issue](https://github.com/devhammad0/physical-ai-robotics-textbook/issues)
