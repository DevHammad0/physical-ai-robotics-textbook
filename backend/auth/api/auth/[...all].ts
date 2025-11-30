import { auth } from "../../lib/auth.js";
import type { VercelRequest, VercelResponse } from "@vercel/node";

// Allowed origins for CORS
const getAllowedOrigins = (): string[] => {
  const origins = [
    "http://localhost:3000",
    "http://localhost:8000",
    "https://devhammad0.github.io",
    "https://devhammado.github.io", // Alternative spelling
  ];
  
  // Add from environment variables
  if (process.env.CORS_ORIGIN) {
    origins.push(process.env.CORS_ORIGIN);
  }
  if (process.env.CORS_ORIGINS) {
    origins.push(...process.env.CORS_ORIGINS.split(",").map((o: string) => o.trim()));
  }
  
  return origins;
};

// Convert Vercel request to Web API Request
async function toWebRequest(req: VercelRequest): Promise<Request> {
  // Get the full URL - Vercel provides these headers
  const protocol = req.headers["x-forwarded-proto"] || "https";
  const host = req.headers.host || req.headers["x-forwarded-host"] || "";
  const url = `${protocol}://${host}${req.url || ""}`;
  
  // Build headers, excluding host as it's in the URL
  const headers = new Headers();
  Object.entries(req.headers).forEach(([key, value]) => {
    // Skip host header as it's already in the URL
    if (key.toLowerCase() === "host") return;
    
    if (value) {
      if (Array.isArray(value)) {
        value.forEach(v => headers.append(key, v));
      } else {
        headers.set(key, value);
      }
    }
  });
  
  const init: RequestInit = {
    method: req.method || "GET",
    headers,
  };
  
  // Handle request body
  // Vercel may have already parsed JSON, so we need to stringify it back
  if (req.body && (req.method === "POST" || req.method === "PUT" || req.method === "PATCH")) {
    if (typeof req.body === "string") {
      init.body = req.body;
    } else if (req.body instanceof Buffer) {
      init.body = req.body;
    } else {
      // Vercel parsed JSON, stringify it back
      init.body = JSON.stringify(req.body);
      if (!headers.has("Content-Type")) {
        headers.set("Content-Type", "application/json");
      }
    }
  }
  
  return new Request(url, init);
}

// Convert Web API Response to Vercel response
async function sendResponse(res: VercelResponse, response: Response, origin?: string, isAllowedOrigin?: boolean) {
  // Set CORS headers
  if (isAllowedOrigin && origin) {
    res.setHeader("Access-Control-Allow-Origin", origin);
    res.setHeader("Access-Control-Allow-Credentials", "true");
  }
  
  // Copy response headers
  response.headers.forEach((value, key) => {
    // Don't override CORS headers we set
    if (key.toLowerCase() !== "access-control-allow-origin" && 
        key.toLowerCase() !== "access-control-allow-credentials") {
      res.setHeader(key, value);
    }
  });
  
  // Set status
  res.status(response.status);
  
  // Get response body
  const body = await response.text();
  res.send(body);
}

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  try {
    // Get the origin from the request
    const origin = req.headers.origin || req.headers.referer?.split("/").slice(0, 3).join("/");
    const allowedOrigins = getAllowedOrigins();
    
    // Check if origin is allowed
    const isAllowedOrigin = origin && allowedOrigins.some(allowed => 
      origin === allowed || origin.startsWith(allowed)
    );
    
    // Set CORS headers for allowed origins
    if (isAllowedOrigin && origin) {
      res.setHeader("Access-Control-Allow-Origin", origin);
      res.setHeader("Access-Control-Allow-Credentials", "true");
      res.setHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS, PATCH");
      res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, Cookie, X-Requested-With");
      res.setHeader("Access-Control-Max-Age", "86400"); // 24 hours
    }
    
    // Handle preflight requests
    if (req.method === "OPTIONS") {
      res.status(200).end();
      return;
    }
    
    // Convert Vercel request to Web API Request
    const webRequest = await toWebRequest(req);
    
    // Call auth handler
    const response = await auth.handler(webRequest);
    
    // Convert Web API Response to Vercel response
    await sendResponse(res, response, origin, isAllowedOrigin);
  } catch (error) {
    // Enhanced error logging for debugging
    console.error("=== Auth Handler Error ===");
    console.error("Error:", error);
    console.error("Error type:", error instanceof Error ? error.constructor.name : typeof error);
    console.error("Error message:", error instanceof Error ? error.message : String(error));
    if (error instanceof Error && error.stack) {
      console.error("Stack trace:", error.stack);
    }
    console.error("Request method:", req.method);
    console.error("Request URL:", req.url);
    console.error("Request headers:", JSON.stringify(req.headers, null, 2));
    console.error("=========================");
    
    // Set CORS headers even on error
    const origin = req.headers.origin;
    const allowedOrigins = getAllowedOrigins();
    const isAllowedOrigin = origin && allowedOrigins.some(allowed => 
      origin === allowed || origin.startsWith(allowed)
    );
    
    if (isAllowedOrigin && origin) {
      res.setHeader("Access-Control-Allow-Origin", origin);
      res.setHeader("Access-Control-Allow-Credentials", "true");
    }
    
    res.status(500).json({
      error: "Internal server error",
      message: error instanceof Error ? error.message : "Unknown error",
      ...(process.env.NODE_ENV !== "production" && error instanceof Error && error.stack
        ? { stack: error.stack.split("\n").slice(0, 10) }
        : {})
    });
  }
}

