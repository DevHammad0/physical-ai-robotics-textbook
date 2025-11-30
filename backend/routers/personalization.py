"""
Personalization router - API endpoints for lesson personalization
"""
from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel
from typing import Optional
import logging
import asyncpg
from services.personalization_service import personalization_service
from services.db_service import get_pool
import os
from pathlib import Path

logger = logging.getLogger(__name__)

router = APIRouter()


class PersonalizeRequest(BaseModel):
    lesson_path: str


class PersonalizeResponse(BaseModel):
    personalized_content: str
    experience_level: int
    cached: bool = False


async def get_user_id_from_session(
    http_request: Request
) -> Optional[str]:
    """
    Extract user ID from better-auth session.
    
    Better-auth uses cookies for session management.
    We need to verify the session with better-auth's API.
    """
    from services.auth_service import get_user_id_from_better_auth_session
    
    try:
        cookies = http_request.cookies
        authorization = http_request.headers.get("authorization")
        return await get_user_id_from_better_auth_session(cookies, authorization)
    except Exception as e:
        logger.error(f"Error verifying session with better-auth: {e}")
        return None


async def get_user_experience_level(
    user_id: Optional[str],
    pool: asyncpg.Pool
) -> Optional[int]:
    """Get user's experience level from database."""
    if not user_id:
        return None
    
    try:
        async with pool.acquire() as conn:
            # Query better-auth's "user" table (singular, quoted because it's a reserved word)
            # better-auth uses TEXT for user IDs, not UUID
            # Try with quoted column name first, then without quotes if that fails
            result = await conn.fetchrow(
                """
                SELECT physical_ai_experience 
                FROM "user" 
                WHERE id = $1
                """,
                user_id
            )
            if result:
                experience = result.get('physical_ai_experience')
                logger.info(f"Found experience level for user {user_id}: {experience}")
                # Return None if experience is None, otherwise return the integer value
                return int(experience) if experience is not None else None
            else:
                logger.warning(f"No user found with id: {user_id}")
    except Exception as e:
        logger.error(f"Error fetching user experience level for user {user_id}: {e}", exc_info=True)
        # Try alternative query without quotes on column name
        try:
            async with pool.acquire() as conn:
                result = await conn.fetchrow(
                    """
                    SELECT physical_ai_experience 
                    FROM "user" 
                    WHERE id = $1
                    """,
                    user_id
                )
                if result:
                    experience = result.get('physical_ai_experience')
                    logger.info(f"Found experience level (retry) for user {user_id}: {experience}")
                    return int(experience) if experience is not None else None
        except Exception as e2:
            logger.error(f"Retry query also failed: {e2}")
    
    return None


def read_lesson_content(lesson_path: str) -> Optional[str]:
    """
    Read lesson markdown content from file system.
    
    Args:
        lesson_path: Path to lesson file (e.g., "01-chapter-1-ros2-fundamentals/00-intro.md")
    
    Returns:
        Markdown content or None if file not found
    """
    try:
        # Construct path relative to book-source/docs
        # lesson_path might be like "01-chapter-1-ros2-fundamentals/00-intro.md"
        docs_dir = Path(__file__).parent.parent.parent / "book-source" / "docs"
        lesson_file = docs_dir / lesson_path
        
        if lesson_file.exists() and lesson_file.is_file():
            return lesson_file.read_text(encoding='utf-8')
        else:
            logger.warning(f"Lesson file not found: {lesson_file}")
            return None
    except Exception as e:
        logger.error(f"Error reading lesson file: {e}")
        return None


async def get_cached_personalized_content(
    user_id: str,
    lesson_path: str,
    pool: asyncpg.Pool
) -> Optional[str]:
    """Get cached personalized content if available."""
    try:
        async with pool.acquire() as conn:
            result = await conn.fetchrow(
                """
                SELECT personalized_content, experience_level
                FROM personalized_lessons
                WHERE user_id = $1 AND lesson_path = $2
                """,
                user_id,
                lesson_path
            )
            if result:
                return result['personalized_content']
    except Exception as e:
        logger.error(f"Error fetching cached content: {e}")
    
    return None


async def cache_personalized_content(
    user_id: str,
    lesson_path: str,
    original_content: str,
    personalized_content: str,
    experience_level: int,
    pool: asyncpg.Pool
):
    """Cache personalized content in database."""
    try:
        async with pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO personalized_lessons 
                (user_id, lesson_path, original_content, personalized_content, experience_level)
                VALUES ($1, $2, $3, $4, $5)
                ON CONFLICT (user_id, lesson_path)
                DO UPDATE SET
                    personalized_content = EXCLUDED.personalized_content,
                    original_content = EXCLUDED.original_content,
                    experience_level = EXCLUDED.experience_level,
                    updated_at = NOW()
                """,
                user_id,
                lesson_path,
                original_content,
                personalized_content,
                experience_level
            )
    except Exception as e:
        logger.error(f"Error caching personalized content: {e}")


@router.post("/personalize/lesson", response_model=PersonalizeResponse)
async def personalize_lesson(
    request: PersonalizeRequest,
    http_request: Request,
    pool: asyncpg.Pool = Depends(get_pool)
):
    """
    Personalize a lesson based on user's experience level.
    
    Requires authentication - user must be signed in.
    """
    # Get user ID from session
    user_id = await get_user_id_from_session(http_request)
    
    if not user_id:
        raise HTTPException(
            status_code=401,
            detail="Authentication required. Please sign in to personalize content."
        )
    
    # Get user's experience level
    experience_level = await get_user_experience_level(user_id, pool)
    
    if experience_level is None:
        # Log more details for debugging
        logger.warning(f"User {user_id} does not have physical_ai_experience set. Checking user record...")
        try:
            async with pool.acquire() as conn:
                user_record = await conn.fetchrow(
                    """
                    SELECT id, email, physical_ai_experience 
                    FROM "user" 
                    WHERE id = $1
                    """,
                    user_id
                )
                if user_record:
                    logger.info(f"User record found: id={user_record.get('id')}, email={user_record.get('email')}, experience={user_record.get('physical_ai_experience')}")
                else:
                    logger.warning(f"No user record found for id: {user_id}")
        except Exception as e:
            logger.error(f"Error checking user record: {e}")
        
        raise HTTPException(
            status_code=400,
            detail="User experience level not found. Please update your profile to set your Physical AI & Robotics experience level (0-10)."
        )
    
    # Check cache first
    cached_content = await get_cached_personalized_content(user_id, request.lesson_path, pool)
    if cached_content:
        logger.info(f"Returning cached personalized content for user {user_id}, lesson {request.lesson_path}")
        return PersonalizeResponse(
            personalized_content=cached_content,
            experience_level=experience_level,
            cached=True
        )
    
    # Read original lesson content
    original_content = read_lesson_content(request.lesson_path)
    if not original_content:
        raise HTTPException(
            status_code=404,
            detail=f"Lesson not found: {request.lesson_path}"
        )
    
    # Generate personalized content
    try:
        result = await personalization_service.personalize_lesson(
            original_content=original_content,
            experience_level=experience_level,
            lesson_path=request.lesson_path
        )
        
        # Cache the result
        await cache_personalized_content(
            user_id=user_id,
            lesson_path=request.lesson_path,
            original_content=original_content,
            personalized_content=result["personalized_content"],
            experience_level=experience_level,
            pool=pool
        )
        
        return PersonalizeResponse(
            personalized_content=result["personalized_content"],
            experience_level=experience_level,
            cached=False
        )
    
    except Exception as e:
        logger.error(f"Error personalizing lesson: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Failed to personalize lesson: {str(e)}"
        )

