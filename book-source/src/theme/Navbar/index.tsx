import React, {type ReactNode, useEffect, useState} from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type {WrapperProps} from '@docusaurus/types';
import {createPortal} from 'react-dom';
import AuthButton from '@site/src/components/Auth/AuthButton';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): ReactNode {
  const [navbarRightContainer, setNavbarRightContainer] = useState<Element | null>(null);

  useEffect(() => {
    // Find the navbar right items container
    const rightContainer = document.querySelector('.navbar__items.navbar__items--right');
    if (rightContainer) {
      setNavbarRightContainer(rightContainer);
    }
  }, []);

  return (
    <>
      <Navbar {...props} />
      {navbarRightContainer && createPortal(
        <div className="navbar__item">
          <AuthButton />
        </div>,
        navbarRightContainer
      )}
    </>
  );
}
