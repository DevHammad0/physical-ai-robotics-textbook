/**
 * Root component for Docusaurus theme
 * This component wraps all pages and allows us to add global components
 */
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }): React.ReactElement {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}

