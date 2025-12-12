/// <reference path="../../types/better-auth.d.ts" />
import React, { useState } from "react";
import { createPortal } from "react-dom";
import { useSession, signOut } from "@site/src/lib/auth-client";
import type { ExtendedSession } from "@site/src/types/auth";
import SignInForm from "./SignInForm";
import SignUpForm from "./SignUpForm";
import EditProfileForm from "./EditProfileForm";
import styles from "./Auth.module.css";

export default function AuthButton() {
  const { data: session, isPending } = useSession();
  const [showModal, setShowModal] = useState(false);
  const [authMode, setAuthMode] = useState<"signin" | "signup">("signin");
  const [showDropdown, setShowDropdown] = useState(false);
  const [isEditingProfile, setIsEditingProfile] = useState(false);

  const handleSignOut = async () => {
    await signOut();
    setShowModal(false);
    setShowDropdown(false);
  };

  const handleAuthSuccess = () => {
    setShowModal(false);
  };

  // Close dropdown when clicking outside
  React.useEffect(() => {
    if (!showDropdown) return;

    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      // Check if click is outside the dropdown and not on the avatar button
      if (!target.closest('.navbar__item') || target.closest('[data-dropdown-backdrop]')) {
        setShowDropdown(false);
      }
    };

    // Add event listener after a small delay to prevent immediate closing
    const timeoutId = setTimeout(() => {
      document.addEventListener('click', handleClickOutside);
    }, 100);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('click', handleClickOutside);
    };
  }, [showDropdown]);

  // Listen for custom event to open auth modal (from ChatKitWidget)
  React.useEffect(() => {
    const handleOpenAuthModal = () => {
      setAuthMode("signin");
      setShowModal(true);
    };

    window.addEventListener('open-auth-modal', handleOpenAuthModal);
    return () => {
      window.removeEventListener('open-auth-modal', handleOpenAuthModal);
    };
  }, []);

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

    // Get user initials for avatar
    const getInitials = (name: string | undefined, email: string) => {
      if (name) {
        const names = name.split(' ');
        if (names.length >= 2) {
          return (names[0][0] + names[names.length - 1][0]).toUpperCase();
        }
        return name.substring(0, 2).toUpperCase();
      }
      return email.substring(0, 2).toUpperCase();
    };

    const initials = getInitials(user.name, user.email);

    // Get experience level label
    const getExperienceLabel = (level: number | null | undefined) => {
      if (level === null || level === undefined) return 'Not set';
      if (level <= 3) return 'Beginner';
      if (level <= 6) return 'Intermediate';
      return 'Advanced';
    };

    const editProfileModal = isEditingProfile ? (
      <div className={styles.modalOverlay} onClick={() => setIsEditingProfile(false)}>
        <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
          <button
            className={styles.closeButton}
            onClick={() => setIsEditingProfile(false)}
            aria-label="Close"
          >
            ×
          </button>
          <div className={styles.modalContent}>
            <EditProfileForm
              currentName={user.name || ''}
              currentExperience={user.physical_ai_experience || 0}
              onSuccess={() => setIsEditingProfile(false)}
              onCancel={() => setIsEditingProfile(false)}
            />
          </div>
        </div>
      </div>
    ) : null;

    return (
      <div className={styles.userMenu}>
        <button
          className={styles.userAvatarButton}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label="Account menu"
          title={user.name || user.email}
        >
          <div className={styles.userAvatar}>
            {user.image ? (
              <img src={user.image} alt={user.name || user.email} className={styles.avatarImage} />
            ) : (
              <span className={styles.avatarInitials}>{initials}</span>
            )}
          </div>
          <svg
            className={styles.dropdownArrow}
            width="12"
            height="12"
            viewBox="0 0 12 12"
            fill="none"
            style={{ transform: showDropdown ? 'rotate(180deg)' : 'rotate(0deg)' }}
          >
            <path d="M2 4L6 8L10 4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>

        {showDropdown && (
          <div className={styles.profileDropdown}>
            <div className={styles.profileHeader}>
              <div className={styles.profileAvatarLarge}>
                {user.image ? (
                  <img src={user.image} alt={user.name || user.email} className={styles.avatarImage} />
                ) : (
                  <span className={styles.avatarInitials}>{initials}</span>
                )}
              </div>
              <div className={styles.profileHeaderInfo}>
                <h3 className={styles.profileName}>{user.name || 'User'}</h3>
                <p className={styles.profileEmail}>{user.email}</p>
              </div>
            </div>

            <div className={styles.profileStats}>
              <div className={styles.profileStat}>
                <span className={styles.statLabel}>EXPERIENCE LEVEL:</span>
                <span className={styles.statValue}>{getExperienceLabel(user.physical_ai_experience)}</span>
              </div>
            </div>

            <div className={styles.profileActions}>
              <button
                className={styles.profileActionButton}
                onClick={() => {
                  setShowDropdown(false);
                  setIsEditingProfile(true);
                }}
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M11.334 2.00004C11.5091 1.82494 11.7169 1.68605 11.9457 1.59129C12.1745 1.49653 12.4197 1.44775 12.6673 1.44775C12.9149 1.44775 13.1601 1.49653 13.3889 1.59129C13.6177 1.68605 13.8256 1.82494 14.0007 2.00004C14.1757 2.17513 14.3146 2.383 14.4094 2.61178C14.5042 2.84055 14.5529 3.08575 14.5529 3.33337C14.5529 3.58099 14.5042 3.82619 14.4094 4.05497C14.3146 4.28374 14.1757 4.49161 14.0007 4.66671L5.00065 13.6667L1.33398 14.6667L2.33398 11L11.334 2.00004Z" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
                Edit Profile
              </button>

              <button
                className={styles.profileActionButton}
                onClick={handleSignOut}
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M6 14H3.33333C2.97971 14 2.64057 13.8595 2.39052 13.6095C2.14048 13.3594 2 13.0203 2 12.6667V3.33333C2 2.97971 2.14048 2.64057 2.39052 2.39052C2.64057 2.14048 2.97971 2 3.33333 2H6M10.6667 11.3333L14 8M14 8L10.6667 4.66667M14 8H6" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
                Sign Out
              </button>
            </div>
          </div>
        )}

        {showDropdown && (
          <div
            className={styles.dropdownBackdrop}
            onClick={() => setShowDropdown(false)}
            data-dropdown-backdrop
          />
        )}

        {typeof document !== 'undefined' && editProfileModal && createPortal(editProfileModal, document.body)}
      </div>
    );
  }

  const signInModalContent = showModal ? (
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
  ) : null;

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
      {typeof document !== 'undefined' && signInModalContent && createPortal(signInModalContent, document.body)}
    </>
  );
}

