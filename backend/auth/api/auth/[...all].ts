import { auth } from "../../lib/auth";
import type { VercelRequest, VercelResponse } from "@vercel/node";

export default async function handler(
  req: VercelRequest,
  res: VercelResponse
) {
  return auth.handler(req, res);
}

