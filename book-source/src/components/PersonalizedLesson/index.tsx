import React, { useState, useEffect } from "react";
import { useSession } from "@site/src/lib/auth-client";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import SignInForm from "@site/src/components/Auth/SignInForm";
import SignUpForm from "@site/src/components/Auth/SignUpForm";
import { getApiUrl } from "@site/src/lib/api-config";
import styles from "./PersonalizedLesson.module.css";

interface PersonalizedLessonProps {
  originalContent?: string;
  lessonPath: string;
  children?: React.ReactNode;
}

export default function PersonalizedLesson({
  originalContent: originalContentProp,
  lessonPath,
  children,
}: PersonalizedLessonProps) {
  const { data: session, isPending } = useSession();
  const [activeTab, setActiveTab] = useState<"original" | "personalized">("original");
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [authMode, setAuthMode] = useState<"signin" | "signup">("signin");

  // Determine if we should render children directly (MDX) or use ReactMarkdown (string)
  // If children are provided, they're already rendered React components with MDX styling
  // If originalContent prop is provided, it's a markdown string that needs ReactMarkdown
  const hasChildren = children && React.Children.count(children) > 0;
  const hasOriginalContentProp = !!originalContentProp;

  // Load cached personalized content from localStorage
  useEffect(() => {
    if (session?.user && activeTab === "personalized") {
      const cacheKey = `personalized_${session.user.id}_${lessonPath}`;
      const cached = localStorage.getItem(cacheKey);
      if (cached) {
        try {
          const parsed = JSON.parse(cached);
          setPersonalizedContent(parsed.content);
        } catch (e) {
          console.warn("Failed to parse cached content:", e);
        }
      }
    }
  }, [session, activeTab, lessonPath]);

  const handlePersonalize = async () => {
    if (!session?.user) {
      setShowAuthModal(true);
      setAuthMode("signin");
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Call personalization API (cookies are sent automatically with credentials: "include")
      const apiUrl = getApiUrl("/api/personalize/lesson");
      const personalizeResponse = await fetch(apiUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include", // Important: sends cookies for better-auth session
        body: JSON.stringify({
          lesson_path: lessonPath,
        }),
      });

      // Check if response is OK
      if (!personalizeResponse.ok) {
        // Try to parse as JSON, but handle HTML error pages
        const contentType = personalizeResponse.headers.get("content-type");
        if (contentType && contentType.includes("application/json")) {
          const errorData = await personalizeResponse.json();
          throw new Error(errorData.detail || `Failed to personalize lesson (${personalizeResponse.status})`);
        } else {
          // HTML response (likely 404 or 500 error page)
          const errorText = await personalizeResponse.text();
          throw new Error(
            `API request failed (${personalizeResponse.status}). ` +
            `The backend server may not be running or the endpoint is not available. ` +
            `Expected JSON but received: ${errorText.substring(0, 100)}...`
          );
        }
      }

      // Parse JSON response
      const contentType = personalizeResponse.headers.get("content-type");
      if (!contentType || !contentType.includes("application/json")) {
        const responseText = await personalizeResponse.text();
        throw new Error(
          `Invalid response format. Expected JSON but received: ${responseText.substring(0, 100)}...`
        );
      }

      const data = await personalizeResponse.json();
      setPersonalizedContent(data.personalized_content);

      // Cache in localStorage
      if (session.user.id) {
        const cacheKey = `personalized_${session.user.id}_${lessonPath}`;
        localStorage.setItem(
          cacheKey,
          JSON.stringify({
            content: data.personalized_content,
            timestamp: Date.now(),
          })
        );
      }
    } catch (err) {
      // Enhanced error handling
      if (err instanceof Error) {
        // Check if it's a network error
        if (err.message.includes("Failed to fetch") || err.message.includes("NetworkError")) {
          setError(
            "Unable to connect to the backend server. " +
            "Please ensure the FastAPI backend is running on port 8000."
          );
        } else {
          setError(err.message);
        }
      } else {
        setError("An unexpected error occurred while personalizing the lesson.");
      }
      console.error("Personalization error:", err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleAuthSuccess = () => {
    setShowAuthModal(false);
    // After successful auth, try to personalize
    if (activeTab === "personalized") {
      handlePersonalize();
    }
  };

  return (
    <div className={styles.container}>
      {/* Tabs */}
      <div className={styles.tabs}>
        <button
          className={`${styles.tab} ${activeTab === "original" ? styles.active : ""}`}
          onClick={() => setActiveTab("original")}
        >
          Original
        </button>
        <button
          className={`${styles.tab} ${activeTab === "personalized" ? styles.active : ""}`}
          onClick={() => {
            setActiveTab("personalized");
            if (!session?.user) {
              setShowAuthModal(true);
              setAuthMode("signin");
            } else if (!personalizedContent && !isLoading) {
              handlePersonalize();
            }
          }}
        >
          Personalized
        </button>
      </div>

      {/* Content */}
      <div className={styles.content}>
        {activeTab === "original" ? (
          <div className={styles.markdownContent}>
            {hasChildren ? (
              // Render children directly (MDX content with preserved styling)
              children
            ) : hasOriginalContentProp ? (
              // Render markdown string with ReactMarkdown
              <ReactMarkdown remarkPlugins={[remarkGfm]}>
                {originalContentProp}
              </ReactMarkdown>
            ) : (
              // Fallback: no content available
              <p>No content available</p>
            )}
          </div>
        ) : (
          <div className={styles.personalizedContent}>
            {!session?.user ? (
              <div className={styles.signInPrompt}>
                <h3>Sign In to Personalize</h3>
                <p>
                  Sign in to get a personalized version of this lesson based on your
                  experience level with Physical AI & Robotics.
                </p>
                <div className={styles.authActions}>
                  <button
                    className={styles.primaryButton}
                    onClick={() => {
                      setShowAuthModal(true);
                      setAuthMode("signin");
                    }}
                  >
                    Sign In
                  </button>
                  <button
                    className={styles.secondaryButton}
                    onClick={() => {
                      setShowAuthModal(true);
                      setAuthMode("signup");
                    }}
                  >
                    Sign Up
                  </button>
                </div>
              </div>
            ) : isLoading ? (
              <div className={styles.loading}>
                <div className={styles.spinner}></div>
                <p>Personalizing lesson content...</p>
              </div>
            ) : error ? (
              <div className={styles.error}>
                <p>{error}</p>
                <button
                  className={styles.retryButton}
                  onClick={handlePersonalize}
                >
                  Retry
                </button>
              </div>
            ) : personalizedContent ? (
              <div className={styles.markdownContent}>
                <ReactMarkdown remarkPlugins={[remarkGfm]}>
                  {personalizedContent}
                </ReactMarkdown>
              </div>
            ) : (
              <div className={styles.emptyState}>
                <p>Click the button below to generate personalized content.</p>
                <button
                  className={styles.primaryButton}
                  onClick={handlePersonalize}
                >
                  Generate Personalized Content
                </button>
              </div>
            )}
          </div>
        )}
      </div>

      {/* Auth Modal */}
      {showAuthModal && (
        <div
          className={styles.modalOverlay}
          onClick={() => setShowAuthModal(false)}
        >
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <button
              className={styles.closeButton}
              onClick={() => setShowAuthModal(false)}
            >
              ×
            </button>
            {authMode === "signin" ? (
              <SignInForm
                onSuccess={handleAuthSuccess}
                onCancel={() => setShowAuthModal(false)}
                onSwitchToSignUp={() => setAuthMode("signup")}
              />
            ) : (
              <SignUpForm
                onSuccess={handleAuthSuccess}
                onCancel={() => setShowAuthModal(false)}
              />
            )}
          </div>
        </div>
      )}
    </div>
  );
}

