import React from 'react';
import ReactDOMClient from 'react-dom/client';
import App from './App';
import { render, screen } from '@testing-library/react';
import {act} from 'react';

it('renders home page', async () => {
  await act(async => {
    render(<App />);
  });

  expect(screen.getByText('Inverse Kinematics')).toBeInTheDocument();
  expect(screen.getByText('Forward Kinematics')).toBeInTheDocument();
});

it('renders without crashing', async () => {
  const div = document.createElement('div');

  await act(async => {
    const root = ReactDOMClient.createRoot(div);
    root.render(<App />);
    root.unmount();
  });
});
