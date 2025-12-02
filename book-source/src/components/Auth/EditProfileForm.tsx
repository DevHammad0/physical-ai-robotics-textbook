import React, { useState } from "react";
import styles from "./Auth.module.css";

interface EditProfileFormProps {
  currentName: string;
  currentExperience: number;
  onSuccess: () => void;
  onCancel: () => void;
}

export default function EditProfileForm({
  currentName,
  currentExperience,
  onSuccess,
  onCancel,
}: EditProfileFormProps) {
  const [name, setName] = useState(currentName);
  const [experience, setExperience] = useState(currentExperience);
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const getApiUrl = (endpoint: string) => {
    if (typeof window === "undefined") return endpoint;
    const isDev = window.location.hostname === "localhost";
    const baseUrl = isDev ? "http://localhost:8000" : process.env.NEXT_PUBLIC_API_URL || "";
    return `${baseUrl}${endpoint}`;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setIsLoading(true);

    try {
      const apiUrl = getApiUrl("/api/auth/update-profile");
      const response = await fetch(apiUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include",
        body: JSON.stringify({
          name,
          physical_ai_experience: experience,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.error || "Failed to update profile");
      }

      // Reload the page to refresh the session
      window.location.reload();
      onSuccess();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to update profile");
    } finally {
      setIsLoading(false);
    }
  };

  const getExperienceLabel = (level: number) => {
    if (level <= 3) return 'Beginner';
    if (level <= 6) return 'Intermediate';
    return 'Advanced';
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2>Edit Profile</h2>
      <p className={styles.subtitle}>Update your profile information</p>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.formGroup}>
        <label htmlFor="name">Full Name</label>
        <input
          id="name"
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          required
          placeholder="Enter your full name"
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="experience">
          Physical AI Experience Level: {experience} ({getExperienceLabel(experience)})
        </label>
        <div className={styles.sliderLabels}>
          <span>Beginner (0-3)</span>
          <span>Intermediate (4-6)</span>
          <span>Advanced (7-10)</span>
        </div>
        <input
          id="experience"
          type="range"
          min="0"
          max="10"
          value={experience}
          onChange={(e) => setExperience(Number(e.target.value))}
          className={styles.slider}
        />
        <p className={styles.helpText}>
          This helps us personalize content based on your experience level
        </p>
      </div>

      <div className={styles.formActions}>
        <button
          type="button"
          onClick={onCancel}
          className={styles.cancelButton}
          disabled={isLoading}
        >
          Cancel
        </button>
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? "Saving..." : "Save Changes"}
        </button>
      </div>
    </form>
  );
}
