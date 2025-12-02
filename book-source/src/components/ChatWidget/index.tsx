/**
 * ChatWidget - Chatkit-integrated chat widget for Docusaurus site
 * Uses OpenAI Chatkit with custom server endpoint
 */
import React, { useState, useEffect, useRef, useCallback } from 'react';
import styles from './styles.module.css';
import { createTextSelectionHandler, getCurrentSelection } from './utils/textSelection';
import type { Message, TextSelection } from './types';
import { 
  FiCopy, 
  FiTrash2, 
  FiX, 
  FiSend, 
  FiRefreshCw,
  FiMessageCircle,
  FiExternalLink,
  FiCheck
} from 'react-icons/fi';

// Declare ChatKit global type
declare global {
  interface Window {
    ChatKit?: any;
    __BACKEND_URL__?: string;
  }
}

interface ChatWidgetProps {
  className?: string;
}

export default function ChatWidget({ className }: ChatWidgetProps): React.JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [threadId, setThreadId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [textSelection, setTextSelection] = useState<TextSelection | null>(null);
  const [showAskButton, setShowAskButton] = useState(false);
  const [askButtonPosition, setAskButtonPosition] = useState({ x: 0, y: 0 });
  const [showToast, setShowToast] = useState(false);
  const [toastMessage, setToastMessage] = useState('');
  const [copiedMessageId, setCopiedMessageId] = useState<number | null>(null);
  const [showClearConfirm, setShowClearConfirm] = useState(false);
  const [chatkitInstance, setChatkitInstance] = useState<any>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const askButtonRef = useRef<HTMLDivElement>(null);
  const chatkitContainerRef = useRef<HTMLDivElement>(null);
  const pendingSelectedTextRef = useRef<string | null>(null);

  // Get backend URL
  const getBackendUrl = () => {
    if (typeof window !== 'undefined') {
      return (window as any).__BACKEND_URL__ || 'http://localhost:8000';
    }
    return 'http://localhost:8000';
  };

  // Initialize Chatkit
  useEffect(() => {
    if (typeof window === 'undefined' || !window.ChatKit) {
      // Wait for Chatkit script to load
      const checkChatkit = setInterval(() => {
        if (window.ChatKit) {
          clearInterval(checkChatkit);
          initializeChatkit();
        }
      }, 100);

      // Timeout after 10 seconds
      setTimeout(() => {
        clearInterval(checkChatkit);
        if (!window.ChatKit) {
          console.error('ChatKit script failed to load');
          setError('Failed to load chat interface. Please refresh the page.');
        }
      }, 10000);

      return () => clearInterval(checkChatkit);
    } else {
      initializeChatkit();
      return undefined; // Explicit return for TypeScript
    }
  }, []);

  const initializeChatkit = () => {
    try {
      const backendUrl = getBackendUrl();
      const chatkitEndpoint = `${backendUrl}/api/chatkit`;

      // Initialize Chatkit with custom server
      const chatkit = new window.ChatKit({
        api: {
          endpoint: chatkitEndpoint
        },
        // Use headless mode - we'll handle UI ourselves
        container: chatkitContainerRef.current || undefined,
        // Disable default UI
        ui: false,
      });

      setChatkitInstance(chatkit);

      // Listen for thread updates
      if (chatkit.onThreadUpdate) {
        chatkit.onThreadUpdate((thread: any) => {
          if (thread?.id) {
            setThreadId(thread.id);
          }
        });
      }

      // Listen for message updates
      if (chatkit.onMessageUpdate) {
        chatkit.onMessageUpdate((message: any) => {
          // Handle incoming messages
          handleChatkitMessage(message);
        });
      }

      // Load existing thread or create new one
      loadOrCreateThread(chatkit);
    } catch (err) {
      console.error('Failed to initialize Chatkit:', err);
      setError('Failed to initialize chat. Please refresh the page.');
    }
  };

  const loadOrCreateThread = async (chatkit: any) => {
    try {
      // Try to load existing thread from localStorage
      const storedThreadId = localStorage.getItem('chatkit_thread_id');
      if (storedThreadId && chatkit.loadThread) {
        await chatkit.loadThread(storedThreadId);
        setThreadId(storedThreadId);
      } else if (chatkit.createThread) {
        // Create new thread
        const thread = await chatkit.createThread();
        if (thread?.id) {
          setThreadId(thread.id);
          localStorage.setItem('chatkit_thread_id', thread.id);
        }
      }
    } catch (err) {
      console.error('Failed to load/create thread:', err);
    }
  };

  const handleChatkitMessage = (message: any) => {
    // Convert Chatkit message to our Message format
    const role = message.role || 'assistant';
    let content = '';
    
    // Extract content from Chatkit message format
    if (message.content) {
      if (Array.isArray(message.content)) {
        content = message.content
          .filter((part: any) => part.type === 'text')
          .map((part: any) => part.text)
          .join('\n');
      } else if (typeof message.content === 'string') {
        content = message.content;
      }
    }

    // Extract sources from metadata if available
    const sources = message.metadata?.sources || message.sources || [];

    if (content) {
      setMessages((prev) => {
        // Check if message already exists
        const exists = prev.some((m, idx) => 
          idx === prev.length - 1 && m.role === role && m.content === content
        );
        if (exists) {
          // Update existing message
          return prev.map((m, idx) => 
            idx === prev.length - 1 && m.role === role
              ? { ...m, content, sources }
              : m
          );
        } else {
          // Add new message
          return [...prev, { role, content, sources, timestamp: new Date() }];
        }
      });
    }
  };

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current && !selectedText) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [isOpen, selectedText]);

  // Auto-paste selected text into input when chat opens
  useEffect(() => {
    const textToPaste = pendingSelectedTextRef.current || selectedText;
    
    if (isOpen && textToPaste && !inputValue.trim() && pendingSelectedTextRef.current) {
      const checkAndPaste = () => {
        if (inputRef.current) {
          const formattedText = `"${textToPaste}"\n\n`;
          setInputValue(formattedText);
          pendingSelectedTextRef.current = null;
          
          setTimeout(() => {
            if (inputRef.current) {
              inputRef.current.focus();
              const length = inputRef.current.value.length;
              inputRef.current.setSelectionRange(length, length);
            }
          }, 100);
        } else {
          setTimeout(checkAndPaste, 50);
        }
      };
      
      setTimeout(checkAndPaste, 100);
    }
  }, [isOpen, selectedText, inputValue]);

  // Text selection handler
  useEffect(() => {
    const handler = createTextSelectionHandler();
    
    const unsubscribe = handler.onSelectionChange((selection) => {
      if (selection) {
        setTextSelection(selection);
        setAskButtonPosition(selection.position);
        setShowAskButton(true);
      } else {
        setShowAskButton(false);
        setTextSelection(null);
      }
    });

    const handleClick = (e: MouseEvent) => {
      if (
        askButtonRef.current &&
        !askButtonRef.current.contains(e.target as Node) &&
        !window.getSelection()?.toString().trim()
      ) {
        setShowAskButton(false);
      }
    };

    document.addEventListener('click', handleClick);

    return () => {
      unsubscribe();
      document.removeEventListener('click', handleClick);
    };
  }, []);

  // Update button position on scroll/resize
  useEffect(() => {
    if (!showAskButton) {
      return;
    }

    const updateButtonPosition = () => {
      const currentSelection = getCurrentSelection();
      if (currentSelection) {
        if (textSelection && currentSelection.text === textSelection.text) {
          setAskButtonPosition(currentSelection.position);
        } else if (!textSelection) {
          setAskButtonPosition(currentSelection.position);
        }
      } else {
        setShowAskButton(false);
        setTextSelection(null);
      }
    };

    let timeoutId: NodeJS.Timeout | null = null;
    const throttledUpdate = () => {
      if (timeoutId) {
        clearTimeout(timeoutId);
      }
      timeoutId = setTimeout(updateButtonPosition, 16);
    };

    window.addEventListener('scroll', throttledUpdate, true);
    window.addEventListener('resize', throttledUpdate);
    document.addEventListener('scroll', throttledUpdate, true);
    if (document.body) {
      document.body.addEventListener('scroll', throttledUpdate, true);
    }

    return () => {
      if (timeoutId) {
        clearTimeout(timeoutId);
      }
      window.removeEventListener('scroll', throttledUpdate, true);
      window.removeEventListener('resize', throttledUpdate);
      document.removeEventListener('scroll', throttledUpdate, true);
      if (document.body) {
        document.body.removeEventListener('scroll', throttledUpdate, true);
      }
    };
  }, [showAskButton, textSelection]);

  // Handle "Ask with AI" button click
  const handleAskWithAI = useCallback((e: React.MouseEvent) => {
    e.stopPropagation();
    e.preventDefault();
    
    if (textSelection) {
      const selectedTextContent = textSelection.text.trim();
      
      if (!selectedTextContent) {
        return;
      }
      
      pendingSelectedTextRef.current = selectedTextContent;
      setSelectedText(selectedTextContent);
      setShowAskButton(false);
      setIsOpen(true);
      
      window.getSelection()?.removeAllRanges();
    }
  }, [textSelection]);

  // Send message with Chatkit
  const handleSendMessage = async () => {
    const query = inputValue.trim();
    if (!query || isLoading || !chatkitInstance) {
      return;
    }

    // Format message with selected text if available
    let messageContent = query;
    if (selectedText) {
      messageContent = `"${selectedText}"\n\n${query}`;
    }

    // Add user message to UI immediately
    const userMessage: Message = {
      role: 'user',
      content: query, // Show original query, not the formatted one
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setError(null);
    setIsLoading(true);

    // Clear selected text after sending
    const textToSend = selectedText;
    if (selectedText) {
      setSelectedText(null);
    }

    // Create placeholder assistant message
    const placeholderMessage: Message = {
      role: 'assistant',
      content: '',
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, placeholderMessage]);

    try {
      // Send message through Chatkit
      if (chatkitInstance.sendMessage) {
        await chatkitInstance.sendMessage({
          content: messageContent,
          threadId: threadId || undefined,
        });
      } else {
        // Fallback: use fetch API directly
        await sendMessageViaAPI(messageContent);
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);
      setMessages((prev) => prev.slice(0, -2)); // Remove user and placeholder messages
    } finally {
      setIsLoading(false);
    }
  };

  // Fallback: Send message via API if Chatkit methods not available
  const sendMessageViaAPI = async (content: string) => {
    const backendUrl = getBackendUrl();
    const response = await fetch(`${backendUrl}/api/chatkit`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        type: 'thread.message.create',
        thread_id: threadId,
        content: [{ type: 'text', text: content }],
      }),
    });

    if (!response.ok) {
      throw new Error('Failed to send message');
    }

    // Handle streaming response
    const reader = response.body?.getReader();
    if (!reader) {
      throw new Error('No response body');
    }

    const decoder = new TextDecoder();
    let buffer = '';
    let accumulatedText = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const data = JSON.parse(line.substring(6));
            if (data.type === 'thread.message.delta' && data.delta?.content) {
              const textDelta = data.delta.content.find((c: any) => c.type === 'text');
              if (textDelta?.text) {
                accumulatedText += textDelta.text;
                setMessages((prev) => {
                  const updated = [...prev];
                  const lastIndex = updated.length - 1;
                  if (lastIndex >= 0 && updated[lastIndex].role === 'assistant') {
                    updated[lastIndex] = {
                      ...updated[lastIndex],
                      content: accumulatedText,
                    };
                  }
                  return updated;
                });
              }
            } else if (data.type === 'thread.message.complete') {
              // Message complete, extract sources if available
              const sources = data.message?.metadata?.sources || [];
              setMessages((prev) => {
                const updated = [...prev];
                const lastIndex = updated.length - 1;
                if (lastIndex >= 0 && updated[lastIndex].role === 'assistant') {
                  updated[lastIndex] = {
                    ...updated[lastIndex],
                    content: data.message?.content?.[0]?.text || accumulatedText,
                    sources,
                  };
                }
                return updated;
              });
            }
          } catch (e) {
            console.error('Failed to parse SSE data:', e);
          }
        }
      }
    }
  };

  // Handle Enter key
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Clear conversation
  const handleClearConversation = () => {
    setShowClearConfirm(true);
  };

  const confirmClearConversation = async () => {
    setMessages([]);
    setThreadId(null);
    setSelectedText(null);
    setShowClearConfirm(false);
    
    // Clear from localStorage
    localStorage.removeItem('chatkit_thread_id');
    
    // Create new thread
    if (chatkitInstance && chatkitInstance.createThread) {
      try {
        const thread = await chatkitInstance.createThread();
        if (thread?.id) {
          setThreadId(thread.id);
          localStorage.setItem('chatkit_thread_id', thread.id);
        }
      } catch (err) {
        console.error('Failed to create new thread:', err);
      }
    }
  };

  const cancelClearConversation = () => {
    setShowClearConfirm(false);
  };

  // Copy message to clipboard
  const handleCopyMessage = (content: string, messageIndex: number) => {
    navigator.clipboard.writeText(content).then(() => {
      setCopiedMessageId(messageIndex);
      setToastMessage('Copied to clipboard!');
      setShowToast(true);
      setTimeout(() => {
        setShowToast(false);
        setTimeout(() => setCopiedMessageId(null), 300);
      }, 2000);
    }).catch(() => {
      setToastMessage('Failed to copy');
      setShowToast(true);
      setTimeout(() => setShowToast(false), 2000);
    });
  };

  return (
    <>
      {/* Hidden Chatkit container */}
      <div ref={chatkitContainerRef} style={{ display: 'none' }} />

      {/* Toast notification */}
      {showToast && (
        <div className={styles.toast}>
          <FiCheck className={styles.toastIcon} />
          <span>{toastMessage}</span>
        </div>
      )}

      {/* Clear confirmation modal */}
      {showClearConfirm && (
        <div className={styles.modalOverlay} onClick={cancelClearConversation}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h3 className={styles.modalTitle}>Clear Conversation</h3>
              <button
                className={styles.modalCloseButton}
                onClick={cancelClearConversation}
                aria-label="Close"
              >
                <FiX />
              </button>
            </div>
            <div className={styles.modalContent}>
              <p>Are you sure you want to clear the conversation? This action cannot be undone.</p>
            </div>
            <div className={styles.modalActions}>
              <button
                className={styles.modalCancelButton}
                onClick={cancelClearConversation}
              >
                Cancel
              </button>
              <button
                className={styles.modalConfirmButton}
                onClick={confirmClearConversation}
              >
                <FiTrash2 />
                <span>Clear</span>
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Floating "Ask with AI" button for text selection */}
      {showAskButton && textSelection && (
        <div
          ref={askButtonRef}
          className={styles.askButton}
          style={{
            left: `${askButtonPosition.x}px`,
            top: `${askButtonPosition.y}px`,
          }}
          onMouseDown={handleAskWithAI}
        >
          <FiMessageCircle className={styles.askButtonIcon} />
          <span>Ask with AI</span>
        </div>
      )}

      {/* Main chat widget */}
      <div className={`${styles.chatWidget} ${className || ''}`}>
        {isOpen ? (
          <div className={styles.chatWindow}>
            <div className={styles.chatHeader}>
              <div className={styles.chatHeaderLeft}>
                <h3 className={styles.chatTitle}>Teaching Assistant</h3>
                {selectedText && (
                  <div className={styles.selectedTextBadge}>
                    <span>Context: Selected text</span>
                    <button
                      className={styles.clearContextButton}
                      onClick={() => setSelectedText(null)}
                      aria-label="Clear selected text context"
                      title="Clear context"
                    >
                      <FiX />
                    </button>
                  </div>
                )}
              </div>
              <div className={styles.chatHeaderRight}>
                <button
                  className={styles.clearButton}
                  onClick={handleClearConversation}
                  aria-label="Clear conversation"
                  title="Clear conversation"
                >
                  <FiTrash2 />
                </button>
                <button
                  className={styles.closeButton}
                  onClick={() => setIsOpen(false)}
                  aria-label="Close chat"
                  title="Close chat"
                >
                  <FiX />
                </button>
              </div>
            </div>

            <div className={styles.chatContent}>
              <div className={styles.messageList}>
                {messages.length === 0 && (
                  <div className={styles.welcomeMessage}>
                    <p>Welcome! I'm here to help you learn Physical AI & Robotics.</p>
                    <p>Ask me anything about the textbook content, code examples, or concepts you'd like to explore.</p>
                    {selectedText && (
                      <div className={styles.contextInfo}>
                        <strong>Context:</strong> You've selected text that will be included in your questions.
                      </div>
                    )}
                  </div>
                )}

                {messages.map((message, index) => (
                  <div
                    key={index}
                    className={`${styles.message} ${
                      message.role === 'user' ? styles.userMessage : styles.assistantMessage
                    }`}
                  >
                    <div className={styles.messageContent}>
                      {message.content}
                    </div>
                    {message.role === 'assistant' && message.sources && message.sources.length > 0 && (
                      <div className={styles.sources}>
                        <div className={styles.sourcesTitle}>Sources:</div>
                        {message.sources.map((source, sourceIndex) => (
                          <a
                            key={sourceIndex}
                            href={source.url}
                            target="_blank"
                            rel="noopener noreferrer"
                            className={styles.sourceLink}
                          >
                            <span>{source.chapter_name} - {source.lesson_title}</span>
                            {source.section_heading !== 'N/A' && ` (${source.section_heading})`}
                            <FiExternalLink className={styles.externalLinkIcon} />
                          </a>
                        ))}
                      </div>
                    )}
                    {message.role === 'assistant' && (
                      <button
                        className={`${styles.copyButton} ${copiedMessageId === index ? styles.copied : ''}`}
                        onClick={() => handleCopyMessage(message.content, index)}
                        aria-label="Copy message"
                        title="Copy message"
                      >
                        {copiedMessageId === index ? (
                          <>
                            <FiCheck />
                            <span>Copied</span>
                          </>
                        ) : (
                          <>
                            <FiCopy />
                            <span>Copy</span>
                          </>
                        )}
                      </button>
                    )}
                  </div>
                ))}

                {isLoading && (
                  <div className={`${styles.message} ${styles.assistantMessage} ${styles.loadingMessage}`}>
                    <div className={styles.messageContent}>
                      <div className={styles.typingIndicator}>
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}

                {error && (
                  <div className={styles.errorMessage}>
                    <p>{error}</p>
                    <button
                      className={styles.retryButton}
                      onClick={() => {
                        setError(null);
                        if (messages.length > 0) {
                          const lastUserMessage = [...messages].reverse().find(m => m.role === 'user');
                          if (lastUserMessage) {
                            setInputValue(lastUserMessage.content);
                            handleSendMessage();
                          }
                        }
                      }}
                    >
                      <FiRefreshCw />
                      <span>Retry</span>
                    </button>
                  </div>
                )}

                <div ref={messagesEndRef} />
              </div>

              <div className={styles.chatInput}>
                {selectedText && (
                  <div className={styles.selectedTextIndicator}>
                    <span>Using selected text as context</span>
                    <button
                      className={styles.clearContextButton}
                      onClick={() => setSelectedText(null)}
                      aria-label="Clear context"
                      title="Clear context"
                    >
                      <FiX />
                    </button>
                  </div>
                )}
                <div className={styles.inputContainer}>
                  <textarea
                    ref={inputRef}
                    className={styles.input}
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    onKeyDown={handleKeyDown}
                    placeholder={
                      selectedText
                        ? 'Ask about the selected text...'
                        : 'Ask anything about Physical AI & Robotics...'
                    }
                    rows={1}
                    disabled={isLoading}
                  />
                  <button
                    className={styles.sendButton}
                    onClick={handleSendMessage}
                    disabled={!inputValue.trim() || isLoading}
                    aria-label="Send message"
                    title="Send message (Enter)"
                  >
                    <FiSend />
                  </button>
                </div>
              </div>
            </div>
          </div>
        ) : (
          <button
            className={styles.chatButton}
            onClick={() => {
              setIsOpen(true);
            }}
            aria-label="Open chat"
            title="Open AI Agent"
          >
            <FiMessageCircle />
          </button>
        )}
      </div>
    </>
  );
}
