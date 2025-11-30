/**
 * TypeScript types for ChatWidget component
 */

export interface Source {
  chapter_name: string;
  lesson_title: string;
  section_heading: string;
  url: string;
  relevance_score: number;
}

export interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp?: Date;
}

export interface ChatRequest {
  query: string;
  session_id: string;
  conversation_id?: string | null;
  selected_text?: string | null;
}

export interface ChatResponse {
  message: string;
  sources: Source[];
  conversation_id: string;
  response_time_ms: number;
}

export interface TextSelection {
  text: string;
  position: {
    x: number;
    y: number;
  };
}

export interface StreamingTextEvent {
  type: 'text';
  data: {
    delta: string;
  };
}

export interface StreamingDoneEvent {
  type: 'done';
  data: {
    message: string;
    sources: Source[];
    conversation_id: string;
  };
}

export interface StreamingErrorEvent {
  type: 'error';
  data: {
    error: string;
  };
}

export type StreamingEvent = StreamingTextEvent | StreamingDoneEvent | StreamingErrorEvent;

