/**
 * MDXComponents - Global component registration for Docusaurus MDX
 * Components registered here can be used directly in MDX files without imports
 */
import React from 'react';
// Import the original mapper to extend it
import MDXComponents from '@theme-original/MDXComponents';
// Import our custom components
import PersonalizedLesson from '@site/src/components/PersonalizedLesson';

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Register custom components globally
  PersonalizedLesson: PersonalizedLesson,
};
