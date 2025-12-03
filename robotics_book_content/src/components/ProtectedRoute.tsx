// robotics_book_content/src/components/ProtectedRoute.tsx

import React from 'react';
import { useAuth } from './AuthProvider';
import { Navigate, Outlet } from 'react-router-dom';

interface ProtectedRouteProps {
  // Any additional props for the route, e.g., roles
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = () => {
  const { isAuthenticated, loading } = useAuth();

  if (loading) {
    return <p>Loading authentication status...</p>; // Or a loading spinner
  }

  if (!isAuthenticated) {
    // Redirect unauthenticated users to the sign-in page
    return <Navigate to="/signin" replace />;
  }

  // If authenticated, render the child routes/components
  return <Outlet />;
};

export default ProtectedRoute;
