"""
Create personalized_lessons table for caching personalized lesson content.
This table uses TEXT for user_id to match better-auth's user table.
"""
import asyncio
import asyncpg
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import settings


async def create_table():
    """Create personalized_lessons table."""
    if not settings.database_url:
        print("✗ Error: DATABASE_URL not set in .env file")
        sys.exit(1)
    
    # Mask password in URL for display
    if '@' in settings.database_url:
        parts = settings.database_url.split('@')
        if ':' in parts[0]:
            user_pass = parts[0].split(':')
            if len(user_pass) >= 3:
                masked_url = f"{user_pass[0]}:****@{parts[1]}"
            else:
                masked_url = settings.database_url
        else:
            masked_url = settings.database_url
    else:
        masked_url = settings.database_url
    print(f"Connecting to database: {masked_url[:50]}...")
    
    try:
        conn = await asyncpg.connect(dsn=settings.database_url)
        print("✓ Connected to database")
        
        # Create table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS personalized_lessons (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id TEXT NOT NULL,
                lesson_path VARCHAR(500) NOT NULL,
                original_content TEXT NOT NULL,
                personalized_content TEXT NOT NULL,
                experience_level INTEGER NOT NULL,
                created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                UNIQUE(user_id, lesson_path)
            )
        """)
        print("✓ Created personalized_lessons table")
        
        # Create indexes
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_personalized_lessons_user_id 
            ON personalized_lessons(user_id)
        """)
        print("✓ Created index on user_id")
        
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_personalized_lessons_lesson_path 
            ON personalized_lessons(lesson_path)
        """)
        print("✓ Created index on lesson_path")
        
        await conn.close()
        print("\n✓ Table creation complete")
        
    except Exception as e:
        print(f"\n✗ Error creating table: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(create_table())

