import React, { useState } from "react";
import { signUp } from "@site/src/lib/auth-client";
import styles from "./Auth.module.css";

interface SignUpFormProps {
  onSuccess?: () => void;
  onCancel?: () => void;
}

export default function SignUpForm({ onSuccess, onCancel }: SignUpFormProps) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [experienceLevel, setExperienceLevel] = useState(5);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      const result = await signUp.email({
        email,
        password,
        name,
        physical_ai_experience: experienceLevel,
      });

      if (result.error) {
        setError(result.error.message || "Sign up failed");
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
      <h2>Sign Up</h2>
      <p className={styles.subtitle}>
        Create an account to personalize your learning experience
      </p>

      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label htmlFor="name">Name</label>
          <input
            id="name"
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            placeholder="Your name"
          />
        </div>

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
            minLength={8}
            placeholder="At least 8 characters"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="experience">
            Physical AI & Robotics Experience Level: {experienceLevel}/10
          </label>
          <input
            id="experience"
            type="range"
            min="0"
            max="10"
            value={experienceLevel}
            onChange={(e) => setExperienceLevel(parseInt(e.target.value))}
            className={styles.slider}
          />
          <div className={styles.sliderLabels}>
            <span>Beginner (0)</span>
            <span>Expert (10)</span>
          </div>
          <p className={styles.helpText}>
            This helps us personalize the content to match your experience level
          </p>
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
            {isLoading ? "Signing up..." : "Sign Up"}
          </button>
        </div>
      </form>
    </div>
  );
}

