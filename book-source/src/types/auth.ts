/**
 * Extended User type with custom fields
 * This type extends the base better-auth User type to include physical_ai_experience
 */
export interface ExtendedUser {
  id: string;
  createdAt: Date;
  updatedAt: Date;
  email: string;
  emailVerified: boolean;
  name: string;
  image?: string | null;
  physical_ai_experience?: number | null;
}

/**
 * Extended Session type with custom user fields
 */
export interface ExtendedSession {
  user: ExtendedUser;
  session: {
    id: string;
    userId: string;
    expiresAt: Date;
    token: string;
    ipAddress?: string | null;
    userAgent?: string | null;
    createdAt: Date;
    updatedAt: Date;
  };
}

