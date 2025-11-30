"""
Personalization Service - Generates personalized lesson content based on user experience level
"""
import os
import logging
from typing import Dict, Any, Optional
from agents import Agent, Runner
from services.llm_service import LLMService
from config import settings

logger = logging.getLogger(__name__)

# Set OpenAI API key if not already set
if not os.getenv("OPENAI_API_KEY"):
    os.environ["OPENAI_API_KEY"] = settings.openai_api_key


def get_personalization_instructions(experience_level: int) -> str:
    """
    Get personalization instructions based on experience level.
    
    Args:
        experience_level: User's experience level (0-10)
    
    Returns:
        Instructions string for the agent
    """
    if experience_level <= 3:
        # Beginner
        return """You are personalizing educational content for a BEGINNER (experience level {}/10).

Guidelines for personalization:
- Add MORE explanations and context for every concept
- Break down complex ideas into simpler, step-by-step explanations
- Include more examples and analogies to help understanding
- Add "Why this matters" sections to explain relevance
- Include common beginner mistakes and how to avoid them
- Use simpler language and avoid jargon when possible
- Add more visual descriptions and mental models
- Include encouragement and motivation throughout

Maintain the same structure, code examples, and learning objectives.
Only adjust the explanations, complexity, and depth of concepts.""".format(experience_level)
    
    elif experience_level <= 7:
        # Intermediate
        return """You are personalizing educational content for an INTERMEDIATE learner (experience level {}/10).

Guidelines for personalization:
- Keep balanced complexity - not too simple, not too advanced
- Provide moderate explanations - assume some background knowledge
- Include practical examples and use cases
- Focus on understanding relationships between concepts
- Add some advanced tips but keep core content accessible
- Include best practices and common patterns
- Reference related concepts without over-explaining basics

Maintain the same structure, code examples, and learning objectives.
Only adjust the explanations, complexity, and depth of concepts.""".format(experience_level)
    
    else:
        # Expert
        return """You are personalizing educational content for an EXPERT (experience level {}/10).

Guidelines for personalization:
- Be CONCISE - skip basic explanations and definitions
- Focus on advanced concepts, edge cases, and optimizations
- Assume deep understanding of fundamentals
- Include advanced patterns, architectures, and best practices
- Add performance considerations and scalability insights
- Reference advanced topics and related systems
- Focus on what's different or unique about this topic
- Skip introductory material and get straight to the point

Maintain the same structure, code examples, and learning objectives.
Only adjust the explanations, complexity, and depth of concepts.""".format(experience_level)


# Create personalization agent
personalization_agent = Agent(
    name="Lesson Personalization Agent",
    instructions="""You are an expert educational content personalization agent for Physical AI & Robotics.

Your task is to personalize lesson content based on the user's experience level while maintaining:
- The same overall structure and flow
- All code examples (unchanged)
- All learning objectives
- All section headings and organization

You will receive:
1. The user's experience level (0-10)
2. The original lesson content in markdown format
3. Specific personalization guidelines based on experience level

Generate a personalized version that adjusts ONLY:
- Explanation depth and complexity
- Amount of context and background
- Level of detail in descriptions
- Use of technical jargon
- Number of examples and analogies

Return the complete personalized lesson in markdown format, ready to be displayed.""",
    model=settings.openai_model,
)


class PersonalizationService:
    """Service for generating personalized lesson content."""

    @staticmethod
    async def personalize_lesson(
        original_content: str,
        experience_level: int,
        lesson_path: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate personalized lesson content based on user experience level.

        Args:
            original_content: Original markdown content of the lesson
            experience_level: User's experience level (0-10)
            lesson_path: Optional path to the lesson for logging

        Returns:
            Dictionary with personalized content and metadata
        """
        try:
            # Get personalization instructions
            personalization_instructions = get_personalization_instructions(experience_level)

            # Build the prompt for the agent
            prompt = f"""{personalization_instructions}

---

ORIGINAL LESSON CONTENT:

{original_content}

---

Please generate the personalized version of this lesson. Return ONLY the markdown content, no additional commentary."""

            # Run the agent
            logger.info(f"Personalizing lesson (experience: {experience_level}/10, path: {lesson_path})")
            result = await Runner.run(personalization_agent, input=prompt)

            personalized_content = result.final_output

            # Clean up the response (remove any markdown code blocks if agent wrapped it)
            if personalized_content.startswith("```"):
                # Extract content from markdown code block
                lines = personalized_content.split("\n")
                if lines[0].startswith("```"):
                    # Find closing ```
                    end_idx = None
                    for i, line in enumerate(lines[1:], 1):
                        if line.strip() == "```":
                            end_idx = i
                            break
                    if end_idx:
                        personalized_content = "\n".join(lines[1:end_idx])

            return {
                "personalized_content": personalized_content.strip(),
                "experience_level": experience_level,
                "original_length": len(original_content),
                "personalized_length": len(personalized_content),
            }

        except Exception as e:
            logger.error(f"Error personalizing lesson: {e}", exc_info=True)
            raise


# Global instance
personalization_service = PersonalizationService()

