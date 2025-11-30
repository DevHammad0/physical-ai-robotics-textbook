/**
 * API configuration for backend endpoints
 * Centralized configuration for all API requests to the FastAPI backend
 */

// Backend API URL - default to local development server
// In production, this can be set via window.__BACKEND_URL__ or environment variable
const getBackendURL = (): string => {
  if (typeof window !== 'undefined') {
    // Check for window global (can be set in HTML)
    if ((window as any).__BACKEND_URL__) {
      return (window as any).__BACKEND_URL__;
    }
    
    // For local development, use localhost:8000
    const origin = window.location.origin;
    if (origin.includes("localhost") || origin.includes("127.0.0.1")) {
      return "http://localhost:8000";
    }
    
    // For production (GitHub Pages), use Vercel backend URL
    return "https://backendhackathon.vercel.app";
  }
  
  // Server-side: use environment variable or default
  return process.env.NEXT_PUBLIC_BACKEND_URL || "http://localhost:8000";
};

export const BACKEND_URL = getBackendURL();

/**
 * Get the full API endpoint URL
 * @param endpoint - API endpoint path (e.g., "/api/personalize/lesson")
 * @returns Full URL to the endpoint
 */
export function getApiUrl(endpoint: string): string {
  // Remove trailing slash from BACKEND_URL to prevent double slashes
  const baseUrl = BACKEND_URL.replace(/\/+$/, '');
  // Remove leading slashes from endpoint and ensure it starts with /
  const cleanEndpoint = '/' + endpoint.replace(/^\/+/, '');
  return baseUrl + cleanEndpoint;
}

