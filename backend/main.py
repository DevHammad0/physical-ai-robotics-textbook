"""
FastAPI main application with CORS, lifespan management, and routers.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging

from routers import chat, health, chatkit, personalization, auth_proxy
from services.db_service import init_db_pool, close_db_pool
from config import settings

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle (startup/shutdown)."""
    # Startup
    logger.info("Starting RAG Chatbot API...")
    try:
        await init_db_pool(
            database_url=settings.database_url,
            min_size=settings.db_pool_min_size,
            max_size=settings.db_pool_max_size
        )
        logger.info("✓ Database connection pool initialized")
    except Exception as e:
        logger.error(f"✗ Failed to initialize database pool: {e}")

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")
    await close_db_pool()
    logger.info("✓ Database connection pool closed")


# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Robotics Textbook",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
# Authentication requires credentials (cookies/sessions), so we must:
# 1. Set allow_credentials=True
# 2. Use explicit origins (no wildcards)
# Ref: https://fastapi.tiangolo.com/tutorial/cors/
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,  # ["http://localhost:3000", "https://devhammad0.github.io"]
    allow_credentials=True,  # Required for authentication with cookies/sessions
    allow_methods=["*"],      # Allow all methods
    allow_headers=["*"],      # Allow all headers
)

# Include routers
# Auth proxy - must be included first to catch /api/auth/* before other routes
app.include_router(auth_proxy.router, prefix="/api", tags=["auth"])
app.include_router(health.router, prefix="/api", tags=["health"])
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(chatkit.router, prefix="/api", tags=["chatkit"])
app.include_router(personalization.router, prefix="/api", tags=["personalization"])


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API for Physical AI & Robotics Textbook",
        "version": "1.0.0",
        "docs": "/docs"
    }


# For local development
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
