-- Create personalized_lessons table for caching personalized lesson content
-- This table uses TEXT for user_id to match better-auth's user table (which uses TEXT IDs, not UUID)

CREATE TABLE IF NOT EXISTS personalized_lessons (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT NOT NULL,  -- TEXT to match better-auth's user.id (not UUID)
    lesson_path VARCHAR(500) NOT NULL,
    original_content TEXT NOT NULL,
    personalized_content TEXT NOT NULL,
    experience_level INTEGER NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    UNIQUE(user_id, lesson_path)
);

CREATE INDEX IF NOT EXISTS idx_personalized_lessons_user_id ON personalized_lessons(user_id);
CREATE INDEX IF NOT EXISTS idx_personalized_lessons_lesson_path ON personalized_lessons(lesson_path);

