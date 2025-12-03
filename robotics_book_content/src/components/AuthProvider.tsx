// robotics_book_content/src/components/AuthProvider.tsx

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { authClient } from '../auth/client'; // Assuming authClient is exported from here

interface AuthContextType {
  isAuthenticated: boolean;
  user: any | null; // Replace 'any' with your User type if defined
  login: (email: string, password: string) => Promise<boolean>;
  signup: (userData: any) => Promise<boolean>;
  logout: () => Promise<void>;
  updateProfile: (profileData: any) => Promise<boolean>;
  loading: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [user, setUser] = useState<any | null>(null);
  const [loading, setLoading] = useState<boolean>(true); // Initial loading state

  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        // In a real application, you would check for a valid session/token here
        // For now, we'll simulate a check
        const currentUser = await authClient.getProfile(); // This might return null if not logged in
        if (currentUser) {
          setIsAuthenticated(true);
          setUser(currentUser);
        } else {
          setIsAuthenticated(false);
          setUser(null);
        }
      } catch (error) {
        console.error("Error checking auth status:", error);
        setIsAuthenticated(false);
        setUser(null);
      } finally {
        setLoading(false);
      }
    };
    checkAuthStatus();
  }, []);

  const login = async (email: string, password: string): Promise<boolean> => {
    setLoading(true);
    try {
      const result = await authClient.login(email, password);
      if (result.success) {
        setIsAuthenticated(true);
        const currentUser = await authClient.getProfile();
        setUser(currentUser);
        return true;
      }
      return false;
    } catch (error) {
      console.error("Login failed:", error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const signup = async (userData: any): Promise<boolean> => {
    setLoading(true);
    try {
      const result = await authClient.signup(userData);
      if (result.success) {
        // After signup, optionally log the user in
        const currentUser = await authClient.getProfile();
        setUser(currentUser);
        setIsAuthenticated(true);
        return true;
      }
      return false;
    } catch (error) {
      console.error("Signup failed:", error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const logout = async (): Promise<void> => {
    setLoading(true);
    try {
      await authClient.logout();
      setIsAuthenticated(false);
      setUser(null);
    } catch (error) {
      console.error("Logout failed:", error);
    } finally {
      setLoading(false);
    }
  };

  const updateProfile = async (profileData: any): Promise<boolean> => {
    setLoading(true);
    try {
      const result = await authClient.updateProfile(profileData);
      if (result.success) {
        const updatedUser = { ...user, ...profileData }; // Optimistic update
        setUser(updatedUser);
        return true;
      }
      return false;
    } catch (error) {
      console.error("Profile update failed:", error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const value = {
    isAuthenticated,
    user,
    login,
    signup,
    logout,
    updateProfile,
    loading,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
