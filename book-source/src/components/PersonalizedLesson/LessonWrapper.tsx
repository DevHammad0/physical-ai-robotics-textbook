import React from "react";
import PersonalizedLesson from "./index";

interface LessonWrapperProps {
  children: React.ReactNode;
  lessonPath?: string;
}

/**
 * Wrapper component for lesson pages that adds personalization tabs.
 * 
 * Usage in MDX:
 * ```mdx
 * import LessonWrapper from '@site/src/components/PersonalizedLesson/LessonWrapper';
 * 
 * <LessonWrapper lessonPath="01-chapter-1-ros2-fundamentals/00-intro.md">
 * 
 * # Your lesson content here
 * 
 * </LessonWrapper>
 * ```
 */
export default function LessonWrapper({
  children,
  lessonPath,
}: LessonWrapperProps) {
  // Extract lesson path from current location if not provided
  const currentPath =
    lessonPath ||
    (typeof window !== "undefined"
      ? window.location.pathname.replace(/^\//, "").replace(/\/$/, "") + ".md"
      : "");

  // Convert children to markdown string
  const contentString =
    typeof children === "string"
      ? children
      : React.Children.toArray(children)
          .map((child) => {
            if (typeof child === "string") {
              return child;
            }
            // For React elements, try to extract text content
            if (React.isValidElement(child)) {
              const props = child.props as { children?: React.ReactNode };
              return React.Children.toArray(props?.children || [])
                .map((c) => (typeof c === "string" ? c : ""))
                .join("");
            }
            return "";
          })
          .join("");

  return (
    <PersonalizedLesson
      originalContent={contentString}
      lessonPath={currentPath}
    />
  );
}

