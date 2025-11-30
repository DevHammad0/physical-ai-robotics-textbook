/// <reference path="../../types/better-auth.d.ts" />
import React, { useState } from "react";
import { useSession, signOut } from "@site/src/lib/auth-client";
import type { ExtendedSession } from "@site/src/types/auth";
import SignInForm from "./SignInForm";
import SignUpForm from "./SignUpForm";
import styles from "./Auth.module.css";

export default function AuthButton() {
  const { data: session, isPending } = useSession();
  const [showModal, setShowModal] = useState(false);
  const [authMode, setAuthMode] = useState<"signin" | "signup">("signin");

  const handleSignOut = async () => {
    await signOut();
    setShowModal(false);
  };

  const handleAuthSuccess = () => {
    setShowModal(false);
  };

  if (isPending) {
    return (
      <button className={styles.authButton} disabled>
        Loading...
      </button>
    );
  }

  if (session?.user) {
    // Type assertion to include our custom field
    const extendedSession = session as ExtendedSession;
    const user = extendedSession.user;
    
    return (
      <div className={styles.userMenu}>
        <button
          className={styles.authButton}
          onClick={() => setShowModal(true)}
          aria-label="Account menu"
        >
          {user.name || user.email}
        </button>
        {showModal && (
          <div className={styles.modalOverlay} onClick={() => setShowModal(false)}>
            <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
              <button
                className={styles.closeButton}
                onClick={() => setShowModal(false)}
                aria-label="Close"
              >
                ×
              </button>
              <div className={styles.modalContent}>
                <div className={styles.userInfo}>
                  <h3>Account</h3>
                  <p><strong>Email:</strong> {user.email}</p>
                  {user.name && (
                    <p><strong>Name:</strong> {user.name}</p>
                  )}
                  {user.physical_ai_experience !== null && user.physical_ai_experience !== undefined && (
                    <p>
                      <strong>Experience Level:</strong> {user.physical_ai_experience}/10
                    </p>
                  )}
                </div>
                <div style={{ padding: '0 1.5rem 1.5rem 1.5rem' }}>
                  <button
                    className={styles.signOutButton}
                    onClick={handleSignOut}
                  >
                    Sign Out
                  </button>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <button
        className={styles.authButton}
        onClick={() => {
          setAuthMode("signin");
          setShowModal(true);
        }}
        aria-label="Sign in or Sign up"
      >
        Sign In
      </button>
      {showModal && (
        <div className={styles.modalOverlay} onClick={() => setShowModal(false)}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <button
              className={styles.closeButton}
              onClick={() => setShowModal(false)}
              aria-label="Close"
            >
              ×
            </button>
            <div className={styles.modalContent}>
              {authMode === "signin" ? (
                <SignInForm
                  onSuccess={handleAuthSuccess}
                  onCancel={() => setShowModal(false)}
                  onSwitchToSignUp={() => setAuthMode("signup")}
                />
              ) : (
                <SignUpForm
                  onSuccess={handleAuthSuccess}
                  onCancel={() => setShowModal(false)}
                />
              )}
            </div>
          </div>
        </div>
      )}
    </>
  );
}

