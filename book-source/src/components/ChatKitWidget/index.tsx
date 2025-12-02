/**
 * ChatKitWidget - Chatkit built-in UI component for Docusaurus site
 * Uses OpenAI Chatkit with custom server endpoint
 */
import React, { useEffect, useState, useRef } from 'react';
import { useChatKit, ChatKit } from '@openai/chatkit-react';
import { FiMessageCircle } from 'react-icons/fi';
import styles from './styles.module.css';

// Get backend URL helper
const getBackendUrl = (): string => {
  if (typeof window !== 'undefined') {
    return (window as any).__BACKEND_URL__ || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
};

interface ChatKitWidgetProps {
  className?: string;
}

export default function ChatKitWidget({ className }: ChatKitWidgetProps): React.JSX.Element {
  const [isReady, setIsReady] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState(false);
  const chatContainerRef = useRef<HTMLDivElement>(null);
  const backendUrl = getBackendUrl();
  const chatkitEndpoint = `${backendUrl}/api/chatkit`;

  // Initialize Chatkit with custom server endpoint
  // For custom backends, use CustomApiConfig with 'url' and 'domainKey' properties
  // Note: useChatKit must be called unconditionally (React hook rules)
  const { control } = useChatKit({
    api: {
      url: chatkitEndpoint, // CustomApiConfig requires 'url', not 'endpoint'
      domainKey: 'domain_pk_local_dev', // Domain key for custom backend (matches backend config)
    },
    // Optional: Add theming to match Docusaurus
    theme: {
      colorScheme: 'light' as const,
    },
    // Optional: Configure header with minimize button
    header: {
      title: {
        text: 'Teaching Assistant',
      },
      rightAction: {
        icon: 'close',
        onClick: () => setIsOpen(false),
      },
    },
    // Optional: Configure composer (input area)
    composer: {
      placeholder: 'Ask anything about Physical AI & Robotics...',
    },
  });

  // Wait for component to be ready
  useEffect(() => {
    // Small delay to ensure DOM is ready
    const timer = setTimeout(() => {
      setIsReady(true);
    }, 100);
    return () => clearTimeout(timer);
  }, []);

  // Log for debugging
  useEffect(() => {
    if (typeof window !== 'undefined') {
      console.log('[ChatKitWidget] Initializing...', {
        endpoint: chatkitEndpoint,
        backendUrl,
        isReady,
        hasControl: !!control,
      });
    }
  }, [chatkitEndpoint, backendUrl, isReady, control]);

  // Error handling via useEffect
  useEffect(() => {
    if (control && typeof control === 'object' && 'error' in control) {
      setError('Chatkit control error');
    }
  }, [control]);

  // Click outside to close functionality
  useEffect(() => {
    if (!isOpen) return;

    const handleClickOutside = (event: MouseEvent) => {
      if (
        chatContainerRef.current &&
        !chatContainerRef.current.contains(event.target as Node)
      ) {
        setIsOpen(false);
      }
    };

    // Add event listener with a small delay to avoid immediate closing when opening
    const timeoutId = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 100);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Escape key to close functionality
  useEffect(() => {
    if (!isOpen) return;

    const handleEscapeKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        setIsOpen(false);
      }
    };

    document.addEventListener('keydown', handleEscapeKey);
    return () => {
      document.removeEventListener('keydown', handleEscapeKey);
    };
  }, [isOpen]);

  // Debug: Show container even if Chatkit isn't ready
  if (typeof window !== 'undefined' && process.env.NODE_ENV === 'development') {
    console.log('[ChatKitWidget] Render state:', {
      isReady,
      hasControl: !!control,
      endpoint: chatkitEndpoint,
    });
  }

  // Render floating bubble button when closed
  if (!isOpen) {
    return (
      <button
        className={styles.chatBubble}
        onClick={() => setIsOpen(true)}
        aria-label="Open chat"
        title="Open Teaching Assistant"
        data-testid="chatkit-bubble"
      >
        <FiMessageCircle className={styles.bubbleIcon} />
      </button>
    );
  }

  // Render chat interface when open
  return (
    <>
      {/* Backdrop overlay */}
      <div 
        className={`${styles.backdrop} ${styles.backdropOpen}`}
        onClick={() => setIsOpen(false)}
        aria-hidden="true"
      />
      
      {/* Chat container */}
      <div 
        ref={chatContainerRef}
        className={`${styles.chatkitContainer} ${className || ''} ${isOpen ? styles.open : ''}`}
        data-testid="chatkit-widget"
      >
        {!isReady && (
          <div style={{ padding: '20px', background: '#f0f0f0' }}>
            <p>Initializing Chatkit...</p>
            <p style={{ fontSize: '12px' }}>Endpoint: {chatkitEndpoint}</p>
          </div>
        )}
        {isReady && control && <ChatKit control={control} />}
        {isReady && !control && (
          <div style={{ padding: '20px', background: '#fff3cd' }}>
            <p>Chatkit control not available</p>
            <p style={{ fontSize: '12px' }}>Check browser console for errors</p>
          </div>
        )}
      </div>
    </>
  );
}

