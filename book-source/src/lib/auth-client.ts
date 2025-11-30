/// <reference path="../types/better-auth.d.ts" />
import { createAuthClient } from "better-auth/react";

// Get base URL from environment or use default
const getBaseURL = () => {
  if (typeof window !== "undefined") {
    // Client-side: use current origin
    const origin = window.location.origin;
    // For GitHub Pages, the API might be on a different domain
    // For local dev, use localhost:8000 (FastAPI proxies to auth server on 8001)
    if (origin.includes("localhost") || origin.includes("127.0.0.1")) {
      return "http://localhost:8000";
    }
    // For production, better-auth should be on the same domain or configured separately
    return origin;
  }
  // Server-side: use environment variable or default
  return process.env.NEXT_PUBLIC_AUTH_URL || "http://localhost:8000";
};

// Create auth client and infer the type to avoid TS2742 errors
const _authClient = createAuthClient({
  baseURL: getBaseURL(),
});

// Use typeof to get the type without complex generic inference
export type AuthClientType = typeof _authClient;

export const authClient: AuthClientType = _authClient;

// Export functions with explicit types to avoid TS2742 errors
export const signIn: AuthClientType["signIn"] = authClient.signIn;
export const signUp: AuthClientType["signUp"] = authClient.signUp;
export const signOut: AuthClientType["signOut"] = authClient.signOut;
export const useSession: AuthClientType["useSession"] = authClient.useSession;

