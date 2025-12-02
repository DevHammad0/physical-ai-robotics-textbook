/**
 * TextSelectionHandler - Detects text selection and shows "Ask with AI" button
 */
import React, { useEffect, useState, useRef, useCallback } from 'react';
import { FiMessageCircle } from 'react-icons/fi';
import { createTextSelectionHandler, getCurrentSelection } from './utils/textSelection';
import type { TextSelection } from './types';
import { useChatKitControl } from '../ChatKitWidget/ChatKitContext';
import styles from './styles.module.css';

const MAX_SELECTION_LENGTH = 500;

export default function TextSelectionHandler(): React.JSX.Element | null {
  const { openWithText } = useChatKitControl();
  const [textSelection, setTextSelection] = useState<TextSelection | null>(null);
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ x: 0, y: 0 });
  const buttonRef = useRef<HTMLDivElement>(null);

  // Text selection listener
  useEffect(() => {
    const handler = createTextSelectionHandler();

    const unsubscribe = handler.onSelectionChange((selection) => {
      if (selection) {
        // Enforce max length
        if (selection.text.length > MAX_SELECTION_LENGTH) {
          return;
        }

        setTextSelection(selection);
        setButtonPosition(selection.position);
        setShowButton(true);
      } else {
        setShowButton(false);
        setTextSelection(null);
      }
    });

    return unsubscribe;
  }, []);

  // Update position on scroll/resize
  useEffect(() => {
    if (!showButton) return;

    const updatePosition = () => {
      const currentSelection = getCurrentSelection();
      if (currentSelection && textSelection?.text === currentSelection.text) {
        setButtonPosition(currentSelection.position);
      } else if (!currentSelection) {
        setShowButton(false);
        setTextSelection(null);
      }
    };

    let timeoutId: NodeJS.Timeout | null = null;
    const throttledUpdate = () => {
      if (timeoutId) clearTimeout(timeoutId);
      timeoutId = setTimeout(updatePosition, 16); // ~60fps
    };

    window.addEventListener('scroll', throttledUpdate, true);
    window.addEventListener('resize', throttledUpdate);

    return () => {
      if (timeoutId) clearTimeout(timeoutId);
      window.removeEventListener('scroll', throttledUpdate, true);
      window.removeEventListener('resize', throttledUpdate);
    };
  }, [showButton, textSelection]);

  // Handle button click
  const handleAskWithAI = useCallback((e: React.MouseEvent) => {
    e.stopPropagation();
    e.preventDefault();

    if (textSelection?.text) {
      openWithText(textSelection.text);
      setShowButton(false);
      setTextSelection(null);
      window.getSelection()?.removeAllRanges();
    }
  }, [textSelection, openWithText]);

  // Hide on click outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (buttonRef.current && !buttonRef.current.contains(e.target as Node)) {
        if (!window.getSelection()?.toString().trim()) {
          setShowButton(false);
          setTextSelection(null);
        }
      }
    };

    if (showButton) {
      document.addEventListener('click', handleClickOutside);
      return () => document.removeEventListener('click', handleClickOutside);
    }
  }, [showButton]);

  // ESC to dismiss
  useEffect(() => {
    const handleEscKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && showButton) {
        setShowButton(false);
        setTextSelection(null);
        window.getSelection()?.removeAllRanges();
      }
    };

    if (showButton) {
      document.addEventListener('keydown', handleEscKey);
      return () => document.removeEventListener('keydown', handleEscKey);
    }
  }, [showButton]);

  if (!showButton || !textSelection) return null;

  return (
    <div
      ref={buttonRef}
      className={styles.askButton}
      style={{
        left: `${buttonPosition.x}px`,
        top: `${buttonPosition.y}px`,
      }}
      onMouseDown={handleAskWithAI}
      role="button"
      aria-label="Ask AI about selected text"
      tabIndex={0}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          handleAskWithAI(e as any);
        }
      }}
    >
      <FiMessageCircle className={styles.icon} />
      <span>Ask with AI</span>
    </div>
  );
}
