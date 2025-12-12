/**
 * ChatKitWidget - Chatkit built-in UI component for Docusaurus site
 * Uses OpenAI Chatkit with custom server endpoint
 */
import React, { useEffect, useState, useRef, useMemo } from 'react';
import { useChatKit, ChatKit } from '@openai/chatkit-react';
import { FiMessageCircle, FiLock } from 'react-icons/fi';
import { useSession } from '@site/src/lib/auth-client';
import styles from './styles.module.css';

// Read domain key from environment variable (injected by Webpack DefinePlugin)
const CHATKIT_DOMAIN_KEY: string = process.env.CHATKIT_DOMAIN_KEY || 'domain_pk_local_dev';

// Get backend URL helper
const getBackendUrl = (): string => {
  if (typeof window !== 'undefined') {
    return (window as any).__BACKEND_URL__ || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
};

interface ChatKitWidgetProps {
  className?: string;
  isOpen?: boolean;              // External control of open state
  onOpenChange?: (isOpen: boolean) => void;  // Notify parent
  prefillText?: string | null;   // Text to prefill
  onPrefillComplete?: () => void; // Callback when done
}

export default function ChatKitWidget({
  className,
  isOpen: isOpenProp,
  onOpenChange,
  prefillText,
  onPrefillComplete
}: ChatKitWidgetProps): React.JSX.Element {
  const [isReady, setIsReady] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [internalIsOpen, setInternalIsOpen] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const chatContainerRef = useRef<HTMLDivElement>(null);
  const selectedTextRef = useRef<string | null>(null);

  // Auth state
  const { data: session, isPending: isAuthPending } = useSession();
  const isAuthenticated = !!session?.user;

  // Store selected text in ref (ensure undefined becomes null)
  useEffect(() => {
    selectedTextRef.current = prefillText ?? null;
  }, [prefillText]);

  // Controlled/uncontrolled pattern for isOpen state
  const isOpen = isOpenProp ?? internalIsOpen;
  const setIsOpen = (value: boolean) => {
    if (isOpenProp === undefined) {
      setInternalIsOpen(value);
    }
    onOpenChange?.(value);
  };
  const backendUrl = getBackendUrl();
  const chatkitEndpoint = `${backendUrl}/api/chatkit`;

  // Initialize Chatkit with custom server endpoint (stable config - never changes)
  const { control } = useChatKit({
    api: {
      url: chatkitEndpoint,
      domainKey: CHATKIT_DOMAIN_KEY,
    },
    theme: {
      colorScheme: 'dark' as const,
      color: {
        accent: {
          primary: '#1677ff', // Match website primary blue
          level: 2,
        },
        surface: {
          background: '#181818',
          foreground: '#303030'
        },
      },
      radius: 'pill',
      density: 'normal',
      typography: {
        fontFamily: "'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif",
      },
    },
    header: {
      title: {
        text: 'Textbook Companion',
      },
      rightAction: {
        icon: 'close',
        onClick: () => setIsOpen(false),
      },
    },
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

  // Log only errors in development (guard against missing `process` in browser)
  useEffect(() => {
    const isDev =
      typeof process !== 'undefined' &&
      process.env &&
      process.env.NODE_ENV === 'development';

    if (typeof window !== 'undefined' && isDev) {
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

  // Note: We display the selected text in a context banner instead of trying to prefill
  // the composer, because ChatKit uses a cross-origin iframe which cannot be accessed
  // due to browser security restrictions (Same-Origin Policy)

  // Debug logging removed - only errors are logged now

  // Render floating bubble button when closed
  if (!isOpen) {
    return (
      <button
        className={styles.chatBubble}
        onClick={() => setIsOpen(true)}
        aria-label="Open Textbook Companion"
        title="Open Textbook Companion"
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
        className={`${styles.chatkitContainer} ${className || ''} ${isOpen ? styles.open : ''} ${prefillText ? styles.hasContext : ''}`}
        data-testid="chatkit-widget"
      >
        {/* Show selected text context if available (only for authenticated users) */}
        {prefillText && isAuthenticated && (
          <div className={styles.contextBanner}>
            <div className={styles.contextHeader}>
              <span className={styles.contextLabel}>Selected text:</span>
              <button
                className={styles.contextClose}
                onClick={(e: React.MouseEvent<HTMLButtonElement>) => {
                  e.stopPropagation();
                  onPrefillComplete?.();
                }}
                aria-label="Clear context"
              >
                ×
              </button>
            </div>
            <div className={styles.contextText}>"{prefillText}"</div>
            <div className={styles.contextActions}>
              <button
                className={styles.copyButton}
                onClick={async (e: React.MouseEvent<HTMLButtonElement>) => {
                  e.stopPropagation();
                  // Store button reference BEFORE async operation
                  const btn = e.currentTarget as HTMLButtonElement;
                  const originalText = btn.textContent;
                  const formattedText = `Selected text: "${prefillText}"\n\nPlease explain this.`;
                  try {
                    await navigator.clipboard.writeText(formattedText);
                    // Button reference is still valid because we stored it before await
                    if (btn) {
                      btn.textContent = '✓ Copied!';
                      setTimeout(() => {
                        if (btn) {
                          btn.textContent = originalText;
                        }
                      }, 2000);
                    }
                  } catch (err) {
                    console.error('Failed to copy:', err);
                    if (btn) {
                      btn.textContent = '❌ Copy failed';
                      setTimeout(() => {
                        if (btn) {
                          btn.textContent = originalText;
                        }
                      }, 2000);
                    }
                  }
                }}
              >
                📋 Copy message template
              </button>
              <span className={styles.contextHint}>Then paste in chat below</span>
            </div>
          </div>
        )}

        {/* Show sign-in prompt for unauthenticated users */}
        {!isAuthenticated ? (
          <div className={styles.signInPrompt}>
            {/* Header matching ChatKit style */}
            <div className={styles.signInHeader}>
              <div className={styles.signInHeaderLeft}>
                <FiMessageCircle size={20} className={styles.signInHeaderIcon} />
                <span className={styles.signInHeaderTitle}>Textbook Companion</span>
              </div>
              <button
                className={styles.signInCloseButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close"
              >
                ×
              </button>
            </div>

            <div className={styles.signInContent}>
              <div className={styles.signInIcon}>
                <FiLock size={40} />
              </div>
              <h3 className={styles.signInTitle}>Sign In Required</h3>
              <p className={styles.signInDescription}>
                Sign in to chat with the AI assistant and get personalized help with Physical AI & Robotics.
              </p>
              <button
                className={styles.signInButton}
                onClick={() => {
                  // Trigger sign-in modal by dispatching custom event
                  window.dispatchEvent(new CustomEvent('open-auth-modal'));
                  setIsOpen(false);
                }}
              >
                <FiLock size={16} />
                Sign In to Chat
              </button>
            </div>
            <div className={styles.signInFooter}>
              <input
                type="text"
                className={styles.signInDisabledInput}
                placeholder="Sign in to ask questions..."
                disabled
              />
              <button className={styles.signInDisabledSend} disabled>
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" />
                </svg>
              </button>
            </div>
          </div>
        ) : (
          <div className={styles.chatkitWrapper}>
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
        )}
      </div>
    </>
  );
}

