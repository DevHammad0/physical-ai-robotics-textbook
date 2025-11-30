import { auth } from "../../lib/auth";
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

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  // Get the origin from the request
  const origin = req.headers.origin || req.headers.referer?.split("/").slice(0, 3).join("/");
  const allowedOrigins = getAllowedOrigins();
  
  // Check if origin is allowed
  const isAllowedOrigin = origin && allowedOrigins.some(allowed => 
    origin === allowed || origin.startsWith(allowed)
  );
  
  // Set CORS headers
  if (isAllowedOrigin) {
    res.setHeader("Access-Control-Allow-Origin", origin);
    res.setHeader("Access-Control-Allow-Credentials", "true");
    res.setHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, Cookie");
  }
  
  // Handle preflight requests
  if (req.method === "OPTIONS") {
    res.status(200).end();
    return;
  }
  
  return auth.handler(req, res);
}

