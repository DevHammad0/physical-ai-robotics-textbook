"""
Profile Router - Handle user profile updates
"""
import logging
from fastapi import APIRouter, Request, HTTPException, Depends
from pydantic import BaseModel
import asyncpg

from services.db_service import get_pool
from services.auth_service import get_user_id_from_better_auth_session

router = APIRouter()
logger = logging.getLogger(__name__)


class UpdateProfileRequest(BaseModel):
    name: str
    physical_ai_experience: int


@router.post("/auth/update-profile")
async def update_profile(
    request: Request,
    profile_data: UpdateProfileRequest,
    pool: asyncpg.Pool = Depends(get_pool)
):
    """
    Update user profile (name and experience level).
    Requires authentication via better-auth session cookie.
    """
    try:
        # Get user ID from session using the same method as personalization
        user_id = await get_user_id_from_better_auth_session(
            cookies=request.cookies,
            authorization=request.headers.get("authorization")
        )

        if not user_id:
            raise HTTPException(
                status_code=401,
                detail="Authentication required. Please sign in."
            )

        # Update user profile in database
        async with pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE "user"
                SET name = $1, physical_ai_experience = $2
                WHERE id = $3
                """,
                profile_data.name,
                profile_data.physical_ai_experience,
                user_id
            )

        logger.info(f"Profile updated for user {user_id}")

        return {
            "success": True,
            "message": "Profile updated successfully"
        }

    except HTTPException:
        raise
    except asyncpg.PostgresError as e:
        logger.error(f"Database error updating profile: {e}")
        raise HTTPException(status_code=500, detail="Database error")
    except Exception as e:
        logger.error(f"Error updating profile: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")
