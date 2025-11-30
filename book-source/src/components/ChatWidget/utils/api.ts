/**
 * API client for chat endpoint
 */

import type { ChatRequest, ChatResponse, StreamingEvent } from '../types';

// Backend API URL - default to local development server
// In production, this can be set via window.__BACKEND_URL__
const BACKEND_URL = (typeof window !== 'undefined' && (window as any).__BACKEND_URL__)
  || 'http://localhost:8000';

export async function sendMessage(
  query: string,
  sessionId: string,
  conversationId: string | null,
  selectedText: string | null
): Promise<ChatResponse> {
  const requestBody: ChatRequest = {
    query,
    session_id: sessionId,
    conversation_id: conversationId || undefined,
    selected_text: selectedText || undefined,
  };

  const response = await fetch(`${BACKEND_URL}/api/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(requestBody),
  });

  if (!response.ok) {
    const errorText = await response.text();
    let errorMessage = 'Failed to send message';
    
    try {
      const errorJson = JSON.parse(errorText);
      errorMessage = errorJson.detail || errorJson.message || errorMessage;
    } catch {
      errorMessage = errorText || errorMessage;
    }

    throw new Error(errorMessage);
  }

  const data: ChatResponse = await response.json();
  return data;
}

/**
 * Send message with streaming support
 * Yields streaming events as they arrive
 */
export async function* sendMessageStream(
  query: string,
  sessionId: string,
  conversationId: string | null,
  selectedText: string | null
): AsyncGenerator<StreamingEvent, void, unknown> {
  const requestBody: ChatRequest = {
    query,
    session_id: sessionId,
    conversation_id: conversationId || undefined,
    selected_text: selectedText || undefined,
  };

  const response = await fetch(`${BACKEND_URL}/api/chat/stream`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(requestBody),
  });

  if (!response.ok) {
    const errorText = await response.text();
    let errorMessage = 'Failed to send message';
    
    try {
      const errorJson = JSON.parse(errorText);
      errorMessage = errorJson.detail || errorJson.message || errorMessage;
    } catch {
      errorMessage = errorText || errorMessage;
    }

    yield {
      type: 'error',
      data: { error: errorMessage },
    };
    return;
  }

  // Parse Server-Sent Events
  const reader = response.body?.getReader();
  if (!reader) {
    yield {
      type: 'error',
      data: { error: 'No response body' },
    };
    return;
  }

  const decoder = new TextDecoder();
  let buffer = '';

  try {
    while (true) {
      const { done, value } = await reader.read();
      
      if (done) {
        break;
      }

      buffer += decoder.decode(value, { stream: true });
      
      // Process complete SSE messages
      const lines = buffer.split('\n');
      buffer = lines.pop() || ''; // Keep incomplete line in buffer

      let currentEvent: { type?: string; data?: string } = {};

      for (const line of lines) {
        if (line === '') {
          // Empty line indicates end of event
          if (currentEvent.type && currentEvent.data) {
            try {
              const data = JSON.parse(currentEvent.data);
              yield {
                type: currentEvent.type as StreamingEvent['type'],
                data,
              } as StreamingEvent;
            } catch (e) {
              console.error('Failed to parse SSE data:', e);
            }
          }
          currentEvent = {};
        } else if (line.startsWith('event: ')) {
          currentEvent.type = line.substring(7).trim();
        } else if (line.startsWith('data: ')) {
          currentEvent.data = line.substring(6).trim();
        }
      }
    }

    // Process any remaining buffer
    if (buffer.trim()) {
      const lines = buffer.split('\n');
      let currentEvent: { type?: string; data?: string } = {};

      for (const line of lines) {
        if (line === '') {
          if (currentEvent.type && currentEvent.data) {
            try {
              const data = JSON.parse(currentEvent.data);
              yield {
                type: currentEvent.type as StreamingEvent['type'],
                data,
              } as StreamingEvent;
            } catch (e) {
              console.error('Failed to parse SSE data:', e);
            }
          }
          currentEvent = {};
        } else if (line.startsWith('event: ')) {
          currentEvent.type = line.substring(7).trim();
        } else if (line.startsWith('data: ')) {
          currentEvent.data = line.substring(6).trim();
        }
      }
    }
  } catch (error) {
    yield {
      type: 'error',
      data: { error: error instanceof Error ? error.message : 'Streaming error' },
    };
  } finally {
    reader.releaseLock();
  }
}

