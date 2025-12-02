/**
 * ChatKit Context - Communication channel between TextSelectionHandler and ChatKitWidget
 */
import React, { createContext, useContext } from 'react';

interface ChatKitContextValue {
  openWithText: (text: string) => void;
  isOpen: boolean;
}

export const ChatKitContext = createContext<ChatKitContextValue | null>(null);

export function useChatKitControl() {
  const context = useContext(ChatKitContext);
  if (!context) {
    throw new Error('useChatKitControl must be used within ChatKitProvider');
  }
  return context;
}
