/// <reference path="../types/better-auth.d.ts" />
import { createAuthClient } from "better-auth/react";
import { BACKEND_URL } from "./api-config";

// Get base URL from environment or use default
const getBaseURL = () => {
  if (typeof window !== "undefined") {
    // Check for window global (can be set in HTML) - same pattern as api-config.ts
    if ((window as any).__BACKEND_URL__) {
      return (window as any).__BACKEND_URL__;
    }
    
    // For local development, use localhost:8000 (FastAPI proxies to auth server on 8001)
    const origin = window.location.origin;
    if (origin.includes("localhost") || origin.includes("127.0.0.1")) {
      return "http://localhost:8000";
    }
    
    // For production, use the same backend URL as the API
    // This will use BACKEND_URL which points to Vercel backend
    return BACKEND_URL;
  }
  // Server-side: use environment variable or default
  return process.env.NEXT_PUBLIC_AUTH_URL || "http://localhost:8000";
};

// Use ReturnType to explicitly type the auth client and avoid TS2742 errors
type AuthClient = ReturnType<typeof createAuthClient>;

const _authClient: AuthClient = createAuthClient({
  baseURL: getBaseURL(),
});

export type AuthClientType = AuthClient;

export const authClient: AuthClientType = _authClient;

// Export functions with explicit types to avoid TS2742 errors
export const signIn: AuthClientType["signIn"] = authClient.signIn;
export const signUp: AuthClientType["signUp"] = authClient.signUp;
export const signOut: AuthClientType["signOut"] = authClient.signOut;
export const useSession: AuthClientType["useSession"] = authClient.useSession;

