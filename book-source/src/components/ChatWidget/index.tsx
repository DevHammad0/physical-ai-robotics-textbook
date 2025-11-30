/**
 * ChatWidget - Custom chat widget for Docusaurus site
 * Integrates with RAG agent backend and supports text selection
 */
import React, { useState, useEffect, useRef, useCallback } from 'react';
import styles from './styles.module.css';
import { sendMessageStream } from './utils/api';
import { createTextSelectionHandler } from './utils/textSelection';
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

interface ChatWidgetProps {
  className?: string;
}

export default function ChatWidget({ className }: ChatWidgetProps): React.JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string>('');
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [textSelection, setTextSelection] = useState<TextSelection | null>(null);
  const [showAskButton, setShowAskButton] = useState(false);
  const [askButtonPosition, setAskButtonPosition] = useState({ x: 0, y: 0 });
  const [showToast, setShowToast] = useState(false);
  const [toastMessage, setToastMessage] = useState('');
  const [copiedMessageId, setCopiedMessageId] = useState<number | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const askButtonRef = useRef<HTMLDivElement>(null);
  const pendingSelectedTextRef = useRef<string | null>(null);
  const hasPastedTextRef = useRef<boolean>(false);

  // Initialize session ID from localStorage
  useEffect(() => {
    try {
      let storedSessionId = localStorage.getItem('chat_session_id');
      if (!storedSessionId) {
        storedSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem('chat_session_id', storedSessionId);
      }
      setSessionId(storedSessionId);
    } catch (e) {
      console.warn('Failed to access localStorage:', e);
      setSessionId(`session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`);
    }
  }, []);

  // Load conversation history from localStorage
  useEffect(() => {
    if (sessionId) {
      try {
        const stored = localStorage.getItem(`chat_messages_${sessionId}`);
        if (stored) {
          const parsed = JSON.parse(stored);
          setMessages(parsed.messages || []);
          setConversationId(parsed.conversationId || null);
        }
      } catch (e) {
        console.warn('Failed to load conversation history:', e);
      }
    }
  }, [sessionId]);

  // Save conversation history to localStorage
  useEffect(() => {
    if (sessionId && messages.length > 0) {
      try {
        localStorage.setItem(
          `chat_messages_${sessionId}`,
          JSON.stringify({
            messages,
            conversationId,
          })
        );
      } catch (e) {
        console.warn('Failed to save conversation history:', e);
      }
    }
  }, [messages, conversationId, sessionId]);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // Focus input when chat opens (but not if we're setting text from selection)
  useEffect(() => {
    if (isOpen && inputRef.current && !selectedText) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [isOpen, selectedText]);
  
  // Auto-paste selected text into input when chat opens
  useEffect(() => {
    console.log('[Auto-Paste Effect] Running...', {
      isOpen,
      selectedText,
      pendingText: pendingSelectedTextRef.current,
      hasInputRef: !!inputRef.current,
      currentInputValue: inputValue,
      hasPasted: hasPastedTextRef.current
    });
    
    // Check both ref and state to ensure we have the text
    const textToPaste = pendingSelectedTextRef.current || selectedText;
    
    // Only paste if chat is open, we have text to paste, input is empty, and we haven't already pasted
    if (isOpen && textToPaste && !inputValue.trim() && !hasPastedTextRef.current) {
      // Wait for input to be mounted
      const checkAndPaste = () => {
        if (inputRef.current) {
          console.log('[Auto-Paste Effect] Pasting text:', textToPaste);
          
          // Format the selected text with quotes
          const formattedText = `"${textToPaste}"\n\n`;
          setInputValue(formattedText);
          console.log('[Auto-Paste Effect] Input value set to:', formattedText);
          
          // Mark as pasted
          hasPastedTextRef.current = true;
          
          // Clear ref after using (but keep selectedText state for context)
          if (pendingSelectedTextRef.current) {
            pendingSelectedTextRef.current = null;
          }
          
          // Focus and set cursor position after the text
          setTimeout(() => {
            if (inputRef.current) {
              inputRef.current.focus();
              const length = inputRef.current.value.length;
              inputRef.current.setSelectionRange(length, length);
              console.log('[Auto-Paste Effect] Focused input, cursor at position:', length);
            }
          }, 100);
        } else {
          // Input not ready yet, try again
          console.log('[Auto-Paste Effect] Input not ready, retrying in 50ms...');
          setTimeout(checkAndPaste, 50);
        }
      };
      
      // Start checking after a small delay to let React render
      setTimeout(checkAndPaste, 100);
    }
    
    // Reset paste flag when chat closes or selected text changes
    if (!isOpen || !selectedText) {
      hasPastedTextRef.current = false;
    }
  }, [isOpen, selectedText]);

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

    // Handle clicks outside to hide button
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

  // Handle "Ask with AI" button click
  const handleAskWithAI = useCallback((e: React.MouseEvent) => {
    console.log('🎯 ========================================');
    console.log('🎯 [Ask with AI] Button clicked!');
    console.log('🎯 ========================================');
    
    // Prevent the click from bubbling up to document click handler
    e.stopPropagation();
    e.preventDefault();
    
    if (textSelection) {
      const selectedTextContent = textSelection.text.trim();
      console.log('[Ask with AI] Selected text:', selectedTextContent);
      
      if (!selectedTextContent) {
        console.log('[Ask with AI] No text selected, aborting');
        return;
      }
      
      // Store in ref so useEffect can access it when chat opens
      pendingSelectedTextRef.current = selectedTextContent;
      console.log('[Ask with AI] Stored in ref:', pendingSelectedTextRef.current);
      
      // Set as context for the agent (will be sent with the message)
      setSelectedText(selectedTextContent);
      setShowAskButton(false);
      setIsOpen(true);
      console.log('[Ask with AI] Opening chat...');
      
      // Clear selection
      window.getSelection()?.removeAllRanges();
      
      // Reset paste flag so text can be pasted when chat opens
      hasPastedTextRef.current = false;
    } else {
      console.log('[Ask with AI] No textSelection available');
    }
  }, [textSelection]);

  // Send message with streaming
  const handleSendMessage = async () => {
    const query = inputValue.trim();
    if (!query || isLoading || !sessionId) {
      return;
    }

    // Add user message to UI immediately
    const userMessage: Message = {
      role: 'user',
      content: query,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setError(null);
    setIsLoading(true);

    // Clear selected text after sending (it's now in the message)
    const textToSend = selectedText;
    if (selectedText) {
      setSelectedText(null);
    }

    // Create a placeholder assistant message that we'll update as we stream
    let accumulatedText = '';

    try {
      // Add placeholder assistant message
      const placeholderMessage: Message = {
        role: 'assistant',
        content: '',
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, placeholderMessage]);

      // Stream responses
      for await (const event of sendMessageStream(
        query,
        sessionId,
        conversationId,
        textToSend
      )) {
        if (event.type === 'text') {
          // Accumulate text deltas
          accumulatedText += event.data.delta;
          
          // Update the last message (which should be the assistant message) in real-time
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
        } else if (event.type === 'done') {
          // Finalize the message with sources and conversation ID
          setMessages((prev) => {
            const updated = [...prev];
            const lastIndex = updated.length - 1;
            if (lastIndex >= 0 && updated[lastIndex].role === 'assistant') {
              updated[lastIndex] = {
                ...updated[lastIndex],
                content: event.data.message,
                sources: event.data.sources,
              };
            }
            return updated;
          });
          setConversationId(event.data.conversation_id);
        } else if (event.type === 'error') {
          throw new Error(event.data.error);
        }
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);
      // Remove both user and placeholder assistant messages on error
      setMessages((prev) => prev.slice(0, -2));
    } finally {
      setIsLoading(false);
    }
  };

  // Handle Enter key (with Shift for new line)
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Clear conversation
  const handleClearConversation = () => {
    if (confirm('Are you sure you want to clear the conversation?')) {
      setMessages([]);
      setConversationId(null);
      setSelectedText(null);
      if (sessionId) {
        try {
          localStorage.removeItem(`chat_messages_${sessionId}`);
        } catch (e) {
          console.warn('Failed to clear conversation history:', e);
        }
      }
    }
  };

  // Copy message to clipboard with toast feedback
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
      {/* Toast notification */}
      {showToast && (
        <div className={styles.toast}>
          <FiCheck className={styles.toastIcon} />
          <span>{toastMessage}</span>
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
                <h3 className={styles.chatTitle}>AI Assistant</h3>
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
                    <p>Welcome! I'm your Physical AI & Robotics teaching assistant.</p>
                    <p>How can I help you today?</p>
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
              console.log('💬 Regular chat button clicked (no selected text)');
              setIsOpen(true);
            }}
            aria-label="Open chat"
            title="Open AI Assistant"
          >
            <FiMessageCircle />
          </button>
        )}
      </div>
    </>
  );
}
