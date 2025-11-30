/**
 * Client module to inject auth button into navbar
 * This runs on the client side after the page loads
 * Docusaurus automatically loads files from src/client-modules/
 */
import { createRoot } from 'react-dom/client';
import React from 'react';
import AuthButton from '@site/src/components/Auth/AuthButton';

let injected = false;
let rootInstance: any = null;
let retryCount = 0;
const MAX_RETRIES = 50; // Try for up to 5 seconds (50 * 100ms)

function injectAuthButton() {
  let container = document.getElementById('auth-button-container');
  
  // Fallback: If container doesn't exist, try to create it in the navbar
  if (!container) {
    // First, try to find navbar__items--right (most common)
    const navbarRight = document.querySelector('.navbar__items--right');
    if (navbarRight) {
      console.log('🔧 Creating auth-button-container in .navbar__items--right');
      container = document.createElement('div');
      container.id = 'auth-button-container';
      container.className = 'navbar__item navbar__item--right';
      navbarRight.appendChild(container);
    } else {
      // Try to find any navbar items container
      const navbarItems = document.querySelector('.navbar__items');
      if (navbarItems) {
        console.log('🔧 Creating auth-button-container in .navbar__items');
        container = document.createElement('div');
        container.id = 'auth-button-container';
        container.className = 'navbar__item navbar__item--right';
        // Try to insert after the GitHub link or at the end
        const githubLink = Array.from(navbarItems.children).find((el: Element) => {
          const text = el.textContent || '';
          return text.includes('GitHub');
        });
        if (githubLink && githubLink.nextSibling) {
          navbarItems.insertBefore(container, githubLink.nextSibling);
        } else if (githubLink) {
          navbarItems.insertBefore(container, githubLink.nextSibling);
        } else {
          navbarItems.appendChild(container);
        }
      } else {
        // Last resort: find navbar and create container
        const navbar = document.querySelector('.navbar') || document.querySelector('nav');
        if (navbar) {
          console.log('🔧 Creating auth-button-container in navbar (fallback)');
          // Find or create navbar items container
          let itemsContainer = navbar.querySelector('.navbar__items');
          if (!itemsContainer) {
            itemsContainer = document.createElement('div');
            itemsContainer.className = 'navbar__items navbar__items--right';
            navbar.appendChild(itemsContainer);
          }
          container = document.createElement('div');
          container.id = 'auth-button-container';
          container.className = 'navbar__item navbar__item--right';
          itemsContainer.appendChild(container);
        }
      }
    }
  }
  
  if (container) {
    try {
      // Always try to inject if container is empty, regardless of injected flag
      // This handles cases where the container gets cleared during navigation
      if (!container.hasChildNodes() || container.children.length === 0) {
        // Clean up previous instance if it exists
        if (rootInstance) {
          try {
            rootInstance.unmount();
          } catch (e) {
            // Ignore unmount errors
          }
          rootInstance = null;
        }
        
        // Clear the injected flag to allow re-injection
        injected = false;
        
        // Create new root and render
        rootInstance = createRoot(container);
        rootInstance.render(React.createElement(AuthButton));
        injected = true;
        retryCount = 0; // Reset retry count on success
        console.log('✓ Auth button injected successfully into navbar');
        return true;
      } else {
        // Container has content - check if it's our React component
        const hasReactRoot = container.querySelector('[data-reactroot]') || 
                            container.querySelector('button') ||
                            container.firstElementChild;
        if (hasReactRoot && !injected) {
          console.log('✓ Auth button container already has content');
          injected = true;
          retryCount = 0;
          return true;
        }
      }
    } catch (e) {
      console.error('✗ Failed to inject auth button:', e);
      // Reset on error so we can try again
      injected = false;
      rootInstance = null;
      return false;
    }
  } else {
    // Container not found
    retryCount++;
    if (retryCount <= 5 || retryCount % 10 === 0) {
      // Log first 5 attempts, then every 10th attempt
      console.warn(`⚠ Auth button container not found (attempt ${retryCount}/${MAX_RETRIES})`);
    }
    
    // Try to find the navbar to see if it exists
    const navbar = document.querySelector('.navbar__items--right') || 
                   document.querySelector('.navbar__items') ||
                   document.querySelector('.navbar');
    if (!navbar && retryCount > 10) {
      console.error('✗ Navbar not found after multiple attempts. Check if Docusaurus is properly initialized.');
    }
    
    return false;
  }
  
  return false;
}

// Run when DOM is ready
if (typeof window !== 'undefined') {
  console.log('🔧 Auth navbar injection module loaded');
  
  // Function to try injection
  const tryInject = () => {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', injectAuthButton);
    } else {
      injectAuthButton();
    }
  };

  // Try immediately
  tryInject();
  
  // Aggressive retry strategy - try multiple times with increasing delays
  const retryDelays = [50, 100, 200, 300, 500, 750, 1000, 1500, 2000, 3000];
  retryDelays.forEach(delay => {
    setTimeout(() => {
      if (!injected) {
        injectAuthButton();
      }
    }, delay);
  });
  
  // Continuous retry until injected or max retries reached
  const continuousRetry = setInterval(() => {
    if (injected || retryCount >= MAX_RETRIES) {
      clearInterval(continuousRetry);
      if (retryCount >= MAX_RETRIES && !injected) {
        console.error('✗ Failed to inject auth button after maximum retries. Please check the browser console for errors.');
      }
      return;
    }
    injectAuthButton();
  }, 100);
  
  // Listen for Docusaurus navigation (client-side routing)
  window.addEventListener('load', () => {
    console.log('📄 Page loaded, attempting auth button injection');
    injected = false; // Reset to allow injection on page load
    retryCount = 0;
    injectAuthButton();
  });
  
  // Listen for Docusaurus route changes using history API
  let lastPath = window.location.pathname;
  const checkRouteChange = () => {
    if (window.location.pathname !== lastPath) {
      lastPath = window.location.pathname;
      console.log('🔄 Route changed, re-injecting auth button');
      // Reset injection on route change to allow re-injection
      injected = false;
      retryCount = 0;
      if (rootInstance) {
        try {
          rootInstance.unmount();
        } catch (e) {
          // Ignore
        }
        rootInstance = null;
      }
      setTimeout(injectAuthButton, 50);
      setTimeout(injectAuthButton, 200);
      setTimeout(injectAuthButton, 500);
    }
  };
  
  // Check for route changes
  setInterval(checkRouteChange, 100);
  
  // Also listen to popstate for browser back/forward
  window.addEventListener('popstate', () => {
    setTimeout(() => {
      injected = false;
      retryCount = 0;
      injectAuthButton();
    }, 100);
  });
  
  // Use MutationObserver to watch for navbar changes
  const observer = new MutationObserver((mutations) => {
    if (!injected) {
      // Check if any mutation added our container
      for (const mutation of mutations) {
        if (mutation.type === 'childList') {
          for (const node of Array.from(mutation.addedNodes)) {
            if (node.nodeType === 1) { // Element node
              const container = (node as Element).querySelector?.('#auth-button-container') ||
                               ((node as Element).id === 'auth-button-container' ? node as Element : null);
              if (container) {
                console.log('🔍 Found auth-button-container via MutationObserver');
                setTimeout(injectAuthButton, 50);
                break;
              }
            }
          }
        }
      }
      injectAuthButton();
    }
  });
  
  // Start observing once navbar exists
  const startObserving = () => {
    const navbar = document.querySelector('.navbar__items--right') || 
                   document.querySelector('.navbar__items') ||
                   document.querySelector('.navbar') ||
                   document.querySelector('nav');
    if (navbar) {
      console.log('👀 Observing navbar for changes');
      observer.observe(navbar, { 
        childList: true, 
        subtree: true,
        attributes: false
      });
      // Also observe the document body for when navbar is added
      observer.observe(document.body, {
        childList: true,
        subtree: true
      });
      injectAuthButton(); // Try once more when navbar is found
    } else {
      setTimeout(startObserving, 100);
    }
  };
  
  startObserving();
}

