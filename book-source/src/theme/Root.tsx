/**
 * Root component for Docusaurus theme
 * This component wraps all pages and allows us to add global components
 */
import React, { useState, useMemo, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// Using Chatkit built-in UI
import ChatKitWidget from '@site/src/components/ChatKitWidget';
import TextSelectionHandler from '@site/src/components/TextSelectionHandler';
import { ChatKitContext } from '@site/src/components/ChatKitWidget/ChatKitContext';
// Keep old ChatWidget for safety (commented out)
// import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }): React.ReactElement {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  // Check if we're on the homepage (accounting for baseUrl)
  // Homepage paths: "/", "/physical-ai-robotics-textbook", "/physical-ai-robotics-textbook/"
  const isHomePage =
    location.pathname === '/' ||
    location.pathname === '' ||
    location.pathname === baseUrl ||
    location.pathname === baseUrl.replace(/\/$/, '') ||
    location.pathname === baseUrl + '/';

  // Set backend URL for production (GitHub Pages)
  // This ensures the backend URL is available before any components try to use it
  if (typeof window !== 'undefined' && !(window as any).__BACKEND_URL__) {
    const origin = window.location.origin;
    // Only set for production (not localhost)
    if (!origin.includes("localhost") && !origin.includes("127.0.0.1")) {
      (window as any).__BACKEND_URL__ = 'https://backendhackathon.vercel.app';
    }
  }

  // State for managing ChatKit with text selection
  const [isOpen, setIsOpen] = useState(false);
  const [prefillText, setPrefillText] = useState<string | null>(null);

  // Context value for TextSelectionHandler to communicate with ChatKitWidget
  const contextValue = useMemo(() => ({
    openWithText: (text: string) => {
      setPrefillText(text);
      setIsOpen(true);
    },
    isOpen,
  }), [isOpen]);

  // Clear prefill after it's used
  const handlePrefillComplete = () => {
    setPrefillText(null);
  };

  // Close chat when navigating to homepage
  useEffect(() => {
    if (isHomePage && isOpen) {
      setIsOpen(false);
    }
  }, [isHomePage]);

  return (
    <ChatKitContext.Provider value={contextValue}>
      {children}
      {/* Hide chatbot on homepage only */}
      {!isHomePage && (
        <>
          <TextSelectionHandler />
          <ChatKitWidget
            isOpen={isOpen}
            onOpenChange={setIsOpen}
            prefillText={prefillText}
            onPrefillComplete={handlePrefillComplete}
          />
        </>
      )}
      {/* Old ChatWidget kept for safety - uncomment to rollback */}
      {/* <ChatWidget /> */}
    </ChatKitContext.Provider>
  );
}

