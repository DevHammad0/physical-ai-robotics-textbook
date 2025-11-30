import React, { useState } from "react";
import { signIn } from "@site/src/lib/auth-client";
import styles from "./Auth.module.css";

interface SignInFormProps {
  onSuccess?: () => void;
  onCancel?: () => void;
  onSwitchToSignUp?: () => void;
}

export default function SignInForm({ 
  onSuccess, 
  onCancel,
  onSwitchToSignUp 
}: SignInFormProps) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      const result = await signIn.email({
        email,
        password,
      });

      if (result.error) {
        setError(result.error.message || "Sign in failed");
      } else {
        onSuccess?.();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : "An error occurred");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authForm}>
      <h2>Sign In</h2>
      <p className={styles.subtitle}>
        Sign in to access personalized content
      </p>

      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            placeholder="your@email.com"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            placeholder="Your password"
          />
        </div>

        {error && <div className={styles.error}>{error}</div>}

        <div className={styles.formActions}>
          {onCancel && (
            <button
              type="button"
              onClick={onCancel}
              className={styles.cancelButton}
              disabled={isLoading}
            >
              Cancel
            </button>
          )}
          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? "Signing in..." : "Sign In"}
          </button>
        </div>

        {onSwitchToSignUp && (
          <div className={styles.switchForm}>
            <p>
              Don't have an account?{" "}
              <button
                type="button"
                onClick={onSwitchToSignUp}
                className={styles.linkButton}
              >
                Sign up
              </button>
            </p>
          </div>
        )}
      </form>
    </div>
  );
}

