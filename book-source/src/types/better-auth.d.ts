/**
 * Type declarations for better-auth to include custom user fields
 * This extends the User type to include the physical_ai_experience field
 * that is configured in the backend auth setup.
 */

declare module "better-auth/react" {
  export interface User {
    physical_ai_experience?: number | null;
  }
}

declare module "better-auth/client" {
  export interface User {
    physical_ai_experience?: number | null;
  }
}

// Ensure this file is treated as a module
export {};

