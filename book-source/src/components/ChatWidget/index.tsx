/**
 * ChatWidget - Floating chat widget for Docusaurus site
 * Uses OpenAI ChatKit React SDK
 */
import React, { useState, useEffect, useRef } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import styles from './styles.module.css';

// Backend API URL - default to local development server
// In production, this can be set via window.__BACKEND_URL__
const BACKEND_URL = (typeof window !== 'undefined' && (window as any).__BACKEND_URL__)
  || 'http://localhost:8000';

interface ChatWidgetProps {
  className?: string;
}

export default function ChatWidget({ className }: ChatWidgetProps): React.JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [colorMode, setColorMode] = useState<'light' | 'dark'>('light');
  const [isReady, setIsReady] = useState(false);
  const initTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // Get color mode from DOM (works even when ColorModeProvider isn't available)
  useEffect(() => {
    const getColorMode = (): 'light' | 'dark' => {
      if (typeof window === 'undefined') return 'light';
      const htmlElement = document.documentElement;
      const theme = htmlElement.getAttribute('data-theme');
      return theme === 'dark' ? 'dark' : 'light';
    };

    // Set initial color mode
    setColorMode(getColorMode());

    // Watch for theme changes using MutationObserver
    const observer = new MutationObserver(() => {
      setColorMode(getColorMode());
    });

    if (typeof window !== 'undefined') {
      observer.observe(document.documentElement, {
        attributes: true,
        attributeFilter: ['data-theme'],
      });
    }

    return () => {
      observer.disconnect();
    };
  }, []);

  // Generate or retrieve session ID from localStorage
  useEffect(() => {
    try {
      let storedSessionId = localStorage.getItem('chatkit_session_id');
      if (!storedSessionId) {
        storedSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem('chatkit_session_id', storedSessionId);
      }
      setSessionId(storedSessionId);
    } catch (e) {
      console.warn('Failed to access localStorage:', e);
      setSessionId(`session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`);
    }
  }, []);

  // Initialize ChatKit hook (must be called unconditionally per React rules)
  const chatKitConfig = useChatKit({
    api: {
      domainKey: 'domain_pk_local_dev',
      url: `${BACKEND_URL}/api/chatkit`,
    },
    theme: {
      colorScheme: colorMode === 'dark' ? 'dark' : 'light',
      color: {
        accent: {
          primary: '#25c2a0', // Docusaurus primary color
          level: 2,
        },
      },
      radius: 'round',
      density: 'normal',
    },
    composer: {
      placeholder: 'Ask anything about Physical AI & Robotics...',
    },
    startScreen: {
      greeting: 'Welcome! I\'m your Physical AI & Robotics teaching assistant. How can I help you today?',
      prompts: [
        {
          label: 'ROS 2 Basics',
          prompt: 'Tell me about ROS 2 fundamentals',
        },
        {
          label: 'Gazebo Simulation',
          prompt: 'How do I set up Gazebo simulation?',
        },
        {
          label: 'Navigation Stack',
          prompt: 'Explain the Nav2 navigation stack',
        },
      ],
    },
    onError: ({ error }) => {
      console.error('[ChatKit] === ERROR EVENT ===');
      console.error('[ChatKit] Error object:', error);
      console.error('[ChatKit] Error message:', error?.message);
      console.error('[ChatKit] Error stack:', error?.stack);

      // Check for specific error types
      if (error?.message?.includes('Failed to fetch')) {
        console.error('[ChatKit] Network error - cannot reach backend');
        setError(`Cannot reach backend at ${BACKEND_URL}/api/chatkit`);
      } else if (error?.message?.includes('CORS')) {
        console.error('[ChatKit] CORS error detected');
        setError('CORS configuration error');
      } else {
        setError(error?.message || 'Unknown initialization error');
      }
      setIsReady(false);
    },
    onReady: () => {
      console.log('[ChatKit] === READY EVENT ===');
      console.log('[ChatKit] Successfully initialized!');
      console.log('[ChatKit] Backend:', BACKEND_URL);
      console.log('[ChatKit] Session ID:', sessionId);
      setError(null);
      setIsReady(true);
      // Clear timeout since we're ready
      if (initTimeoutRef.current) {
        clearTimeout(initTimeoutRef.current);
        initTimeoutRef.current = null;
      }
    },
  });

  // Timeout handler for initialization
  useEffect(() => {
    // Clear any existing timeout first
    if (initTimeoutRef.current) {
      clearTimeout(initTimeoutRef.current);
      initTimeoutRef.current = null;
    }

    if (isOpen && !isReady && !error) {
      // Set a timeout to detect if ChatKit is stuck
      const timeout = setTimeout(() => {
        console.warn('[ChatKit] Initialization timeout after 15 seconds');
        console.warn('[ChatKit] This usually means:');
        console.warn('  1. getClientSecret was never called, OR');
        console.warn('  2. ChatKit rejected the client_secret silently, OR');
        console.warn('  3. Backend returned an invalid token');
        setError('Chat initialization is taking longer than expected. Please check the browser console for details.');
        setIsReady(false);
        initTimeoutRef.current = null;
      }, 15000); // 15 second timeout

      initTimeoutRef.current = timeout;

      return () => {
        if (initTimeoutRef.current) {
          clearTimeout(initTimeoutRef.current);
          initTimeoutRef.current = null;
        }
      };
    }

    return undefined;
  }, [isOpen, isReady, error]);

  const toggleChat = () => {
    if (!isOpen) {
      // Reset ready state when opening to ensure fresh initialization
      setIsReady(false);
      setError(null);
      // Clear any existing timeout
      if (initTimeoutRef.current) {
        clearTimeout(initTimeoutRef.current);
        initTimeoutRef.current = null;
      }
    }
    setIsOpen(!isOpen);
  };

  // Don't render if sessionId is not available yet
  if (!sessionId) {
    return <></>;
  }

  const { control } = chatKitConfig;

  // Safety check: ensure control exists
  if (!control) {
    return <></>;
  }

  return (
    <div className={`${styles.chatWidget} ${className || ''}`}>
      {isOpen ? (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3 className={styles.chatTitle}>AI Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          <div className={styles.chatContent}>
            {error ? (
              <div style={{ padding: '20px', color: 'var(--ifm-color-danger)', textAlign: 'center' }}>
                <p style={{ marginBottom: '12px' }}>Chat widget error: {error}</p>
                <button
                  onClick={() => {
                    setError(null);
                    setIsReady(false);
                  }}
                  style={{
                    padding: '8px 16px',
                    backgroundColor: 'var(--ifm-color-primary)',
                    color: 'white',
                    border: 'none',
                    borderRadius: '4px',
                    cursor: 'pointer',
                  }}
                >
                  Retry
                </button>
              </div>
            ) : !isReady ? (
              <div style={{ padding: '20px', textAlign: 'center' }}>
                <p>Initializing chat...</p>
                <p style={{ fontSize: '12px', color: 'var(--ifm-color-secondary)' }}>
                  Connecting to backend...
                </p>
              </div>
            ) : (
              // Render ChatKit only when ready
              <ChatKit control={control} className={styles.chatKitContainer} />
            )}
          </div>
        </div>
      ) : (
        <button
          className={styles.chatButton}
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}
    </div>
  );
}

