/**
 * Root component for Docusaurus theme
 * This component wraps all pages and allows us to add global components
 */
import React from 'react';
// Using Chatkit built-in UI
import ChatKitWidget from '@site/src/components/ChatKitWidget';
// Keep old ChatWidget for safety (commented out)
// import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }): React.ReactElement {
  // Set backend URL for production (GitHub Pages)
  // This ensures the backend URL is available before any components try to use it
  if (typeof window !== 'undefined' && !(window as any).__BACKEND_URL__) {
    const origin = window.location.origin;
    // Only set for production (not localhost)
    if (!origin.includes("localhost") && !origin.includes("127.0.0.1")) {
      (window as any).__BACKEND_URL__ = 'https://backendhackathon.vercel.app';
    }
  }

  return (
    <>
      {children}
      <ChatKitWidget />
      {/* Old ChatWidget kept for safety - uncomment to rollback */}
      {/* <ChatWidget /> */}
    </>
  );
}

