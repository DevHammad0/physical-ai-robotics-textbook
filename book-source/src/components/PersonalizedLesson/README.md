# PersonalizedLesson Component

This component adds Original/Personalized tabs to lesson pages, allowing authenticated users to view personalized content based on their experience level.

## Usage in MDX Files

To add personalization tabs to a lesson, wrap the content with the `PersonalizedLesson` component:

```mdx
import PersonalizedLesson from '@site/src/components/PersonalizedLesson';

<PersonalizedLesson 
  lessonPath="01-chapter-1-ros2-fundamentals/01-what-is-ros2.md"
>

# Your Lesson Title

Your lesson content here...

</PersonalizedLesson>
```

## How It Works

1. **Original Tab**: Always shows the original markdown content (public, no auth required)
2. **Personalized Tab**: 
   - If not authenticated: Shows sign-in prompt
   - If authenticated: Generates personalized content based on user's experience level (0-10)
   - Caches personalized content in localStorage and database

## Component Props

- `lessonPath` (required): Path to the lesson file relative to `book-source/docs/`
- `originalContent`: The original markdown content (automatically extracted from children)

## Example

```mdx
---
title: "What is ROS 2?"
---

import PersonalizedLesson from '@site/src/components/PersonalizedLesson';

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/01-what-is-ros2.md">

# What is ROS 2?

ROS 2 is the Robot Operating System 2...

</PersonalizedLesson>
```

