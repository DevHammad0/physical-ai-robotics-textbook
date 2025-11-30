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

export const authClient = createAuthClient({
  baseURL: getBaseURL(),
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
} = authClient;

