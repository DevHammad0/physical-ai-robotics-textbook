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
  const position = {
    x: rect.left + rect.width / 2,
    y: rect.top - 10, // Position above the selection
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

