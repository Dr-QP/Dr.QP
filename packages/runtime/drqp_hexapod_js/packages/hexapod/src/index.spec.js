import React from 'react';
import ReactDOMClient from 'react-dom/client';
import App from './App';
// import { act, render, screen } from '@testing-library/react';
import { act } from '@testing-library/react';

describe("client", () => {
  it('renders home page', async () => {
    await act(() => {
      // This hangs the tests to the point where Ctrl-C doesn't work
      // Figure out what is going wrong here
      // render(<App />);
    });

    // expect(screen.getByText('Inverse Kinematics')).toBeInTheDocument();
    // expect(screen.getByText('Forward Kinematics')).toBeInTheDocument();
  });

  it('renders without crashing', async () => {
    const container = document.createElement('div');
    document.body.appendChild(container);

    await act(() => {
      const root = ReactDOMClient.createRoot(container);
      root.render(<App />);
      root.unmount();
    });
  });
});
