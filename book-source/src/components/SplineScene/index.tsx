import React, { useState, useEffect, useRef } from 'react';
import styles from './SplineScene.module.css';

interface SplineSceneProps {
  splineUrl: string;
  height?: string;
  className?: string;
  fallbackContent?: React.ReactNode;
  showOverlay?: boolean;
  useIframe?: boolean; // New prop to use iframe instead of React component
}

export default function SplineScene({
  splineUrl,
  height = '100vh',
  className = '',
  fallbackContent,
  showOverlay = false,
  useIframe = false,
}: SplineSceneProps): React.JSX.Element {
  const [loading, setLoading] = useState(true);
  const [hasValidDimensions, setHasValidDimensions] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const checkDimensions = () => {
      if (containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        if (rect.width > 0 && rect.height > 0) {
          setHasValidDimensions(true);
        }
      }
    };

    checkDimensions();
    const resizeObserver = new ResizeObserver(checkDimensions);
    if (containerRef.current) {
      resizeObserver.observe(containerRef.current);
    }

    return () => {
      resizeObserver.disconnect();
    };
  }, []);

  const handleIframeLoad = () => {
    setLoading(false);
  };

  return (
    <div 
      ref={containerRef}
      className={`${styles.container} ${className}`} 
      style={{ height, minHeight: '200px' }}
    >
      {loading && (
        <div className={styles.fallback}>
          <div className={styles.loader}></div>
          <p>Loading 3D Scene...</p>
        </div>
      )}

      {useIframe && hasValidDimensions ? (
        <iframe
          src={splineUrl}
          className={styles.splineCanvas}
          style={{
            border: 'none',
            width: '100%',
            height: '100%',
            display: loading ? 'none' : 'block'
          }}
          title="3D Robot Scene"
          onLoad={handleIframeLoad}
        />
      ) : useIframe ? null : (
        <div className={styles.errorFallback}>
          <div className={styles.errorIcon}>🤖</div>
          <h2>Unable to Load 3D Scene</h2>
          <p>The Spline scene URL needs to be exported for production use.</p>
          <div className={styles.errorDetails}>
            <p><strong>Current URL:</strong></p>
            <code className={styles.urlDisplay}>{splineUrl}</code>
            <p><strong>How to fix:</strong></p>
            <ol>
              <li>Open your Spline project in the editor</li>
              <li>Click the <strong>Export</strong> button (top right)</li>
              <li>Select <strong>Code Export</strong> → <strong>React</strong></li>
              <li>Copy the scene URL (ends with <code>.splinecode</code>)</li>
              <li>Replace the URL in <code>src/pages/index.tsx</code></li>
            </ol>
            <p><em>Or use <code>useIframe=true</code> prop for iframe embedding</em></p>
          </div>
        </div>
      )}

      {showOverlay && !loading && (
        <div className={styles.overlay} />
      )}

      {fallbackContent && (
        <div className={styles.fallbackContent}>
          {fallbackContent}
        </div>
      )}
    </div>
  );
}
