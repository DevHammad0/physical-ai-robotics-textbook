/**
 * Text selection handler for "Ask with AI" functionality
 */

import type { TextSelection } from '../types';

export interface TextSelectionHandler {
  getSelection: () => TextSelection | null;
  clearSelection: () => void;
  onSelectionChange: (callback: (selection: TextSelection | null) => void) => () => void;
}

/**
 * Get current text selection from the page
 */
export function getCurrentSelection(): TextSelection | null {
  if (typeof window === 'undefined') {
    return null;
  }

  const selection = window.getSelection();
  if (!selection || selection.rangeCount === 0) {
    return null;
  }

  const selectedText = selection.toString().trim();
  
  // Ignore very short selections (likely accidental)
  if (selectedText.length < 3) {
    return null;
  }

  // Ignore selections in input fields, textareas, or code blocks
  const range = selection.getRangeAt(0);
  const container = range.commonAncestorContainer;
  
  // Check if selection is in an input/textarea
  let element = container.nodeType === Node.TEXT_NODE 
    ? container.parentElement 
    : container as Element;
  
  while (element) {
    if (
      element.tagName === 'INPUT' ||
      element.tagName === 'TEXTAREA' ||
      element.closest('input, textarea, code, pre, .code-block')
    ) {
      return null;
    }
    element = element.parentElement;
  }

  // Get position for button placement (relative to viewport for fixed positioning)
  const rect = range.getBoundingClientRect();
  const viewportHeight = window.innerHeight;
  const viewportWidth = window.innerWidth;
  
  // Account for header/navbar height (typically 60-80px for Docusaurus)
  const headerHeight = 80; // Safe margin to account for navbar and spacing
  const buttonHeight = 40; // Approximate button height including padding
  const minTopMargin = headerHeight + 10; // Header height + small spacing
  
  // Calculate button position based on visible portion of selection
  let x = rect.left + rect.width / 2;
  let y: number;
  
  // Calculate visible portion of selection in viewport
  const visibleTop = Math.max(0, rect.top);
  const visibleBottom = Math.min(viewportHeight, rect.bottom);
  const hasVisibleContent = visibleBottom > visibleTop;
  
  if (rect.top < 0) {
    // Selection starts above viewport - position button below header
    // This ensures button is visible when user scrolls down with large selection
    y = minTopMargin;
  } else if (rect.top > viewportHeight) {
    // Selection is below viewport - position button at bottom of viewport
    y = viewportHeight - buttonHeight - 10; // Leave space for button and margin
  } else if (hasVisibleContent) {
    // Selection is visible in viewport
    // Position button above the visible top, but ensure it's below header
    const preferredY = rect.top - 10; // 10px above selection
    y = Math.max(minTopMargin, preferredY);
    
    // If selection is very large and spans most of viewport, position at top
    const selectionHeight = rect.bottom - rect.top;
    if (selectionHeight > viewportHeight * 0.7) {
      y = minTopMargin;
    }
  } else {
    // Fallback: position below header
    y = minTopMargin;
  }
  
  // Ensure button doesn't go off-screen horizontally
  const buttonWidth = 140; // Approximate button width (slightly larger for safety)
  if (x < buttonWidth / 2) {
    x = buttonWidth / 2;
  } else if (x > viewportWidth - buttonWidth / 2) {
    x = viewportWidth - buttonWidth / 2;
  }
  
  // Ensure button doesn't go off-screen vertically
  if (y < minTopMargin) {
    y = minTopMargin;
  } else if (y > viewportHeight - buttonHeight - 10) {
    y = viewportHeight - buttonHeight - 10;
  }
  
  const position = {
    x,
    y,
  };

  return {
    text: selectedText,
    position,
  };
}

/**
 * Create a text selection handler
 */
export function createTextSelectionHandler(): TextSelectionHandler {
  let currentSelection: TextSelection | null = null;
  let callbacks: Set<(selection: TextSelection | null) => void> = new Set();

  const updateSelection = () => {
    const newSelection = getCurrentSelection();
    
    // Only update if selection actually changed
    if (
      newSelection?.text !== currentSelection?.text ||
      (newSelection === null && currentSelection !== null)
    ) {
      currentSelection = newSelection;
      callbacks.forEach(callback => callback(currentSelection));
    }
  };

  // Listen for selection changes
  if (typeof window !== 'undefined') {
    document.addEventListener('mouseup', updateSelection);
    document.addEventListener('selectionchange', updateSelection);
    // Also listen for clicks to clear selection when clicking elsewhere
    document.addEventListener('mousedown', (e) => {
      // Small delay to allow selection to update first
      setTimeout(updateSelection, 10);
    });
  }

  return {
    getSelection: () => currentSelection,
    clearSelection: () => {
      currentSelection = null;
      callbacks.forEach(callback => callback(null));
      if (typeof window !== 'undefined') {
        window.getSelection()?.removeAllRanges();
      }
    },
    onSelectionChange: (callback: (selection: TextSelection | null) => void) => {
      callbacks.add(callback);
      // Immediately call with current selection
      callback(currentSelection);
      
      // Return cleanup function
      return () => {
        callbacks.delete(callback);
      };
    },
  };
}

