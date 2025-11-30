/**
 * Type declarations for better-auth to include custom user fields
 * This extends the User type to include the physical_ai_experience field
 * that is configured in the backend auth setup.
 * 
 * The backend config defines physical_ai_experience as:
 * - type: "number"
 * - required: true
 * - input: true (allows user to provide during signup)
 */

// Extend better-auth/react module
declare module "better-auth/react" {
  export interface User {
    physical_ai_experience?: number | null;
  }
}

// Extend better-auth/client module  
declare module "better-auth/client" {
  export interface User {
    physical_ai_experience?: number | null;
  }
}

// Ensure this file is treated as a module
export {};
