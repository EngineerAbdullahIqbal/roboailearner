# Quickstart Guide: Better-Auth.com Authentication

This guide provides a quick overview of how to set up and integrate the Better-Auth.com authentication feature.

## 1. Install Dependencies

Ensure you have `better-auth` installed in your project.

```bash
npm install better-auth
```

## 2. Basic Configuration

Create an authentication configuration file (e.g., `auth/config.ts`) based on Better-Auth.com's documentation. This will involve setting up your Better-Auth.com API keys and other environment-specific settings.

Example `auth/config.ts` (conceptual):

```typescript
// auth/config.ts
import { BetterAuthConfig } from 'better-auth';

export const authConfig: BetterAuthConfig = {
  apiKey: process.env.BETTER_AUTH_API_KEY || 'YOUR_API_KEY',
  apiSecret: process.env.BETTER_AUTH_API_SECRET || 'YOUR_API_SECRET',
  // ... other configuration settings
};
```

## 3. Frontend Client

Set up a frontend client (e.g., `auth/client.ts`) to interact with the Better-Auth.com service.

Example `auth/client.ts` (conceptual):

```typescript
// auth/client.ts
import { BetterAuthClient } from 'better-auth';
import { authConfig } from './config';

export const betterAuth = new BetterAuthClient(authConfig);
```

## 4. Authentication Provider

Wrap your application with an `AuthProvider` component to manage authentication state and provide access to authentication utilities throughout your React application.

Example `components/AuthProvider.tsx` (conceptual):

```typescript jsx
// components/AuthProvider.tsx
import React, { createContext, useContext, useState, useEffect } from 'react_query';
import { betterAuth } from '../auth/client'; // Adjust path as needed

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Attempt to load user from session or token on mount
    betterAuth.getCurrentUser().then(currentUser => {
      setUser(currentUser);
      setLoading(false);
    });
  }, []);

  const signIn = async (email, password) => {
    const response = await betterAuth.signIn(email, password);
    setUser(response.user);
    return response;
  };

  const signUp = async (email, password, profileData) => {
    const response = await betterAuth.signUp(email, password, profileData);
    setUser(response.user);
    return response;
  };

  const signOut = async () => {
    await betterAuth.signOut();
    setUser(null);
  };

  return (
    <AuthContext.Provider value={{ user, loading, signIn, signUp, signOut }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => useContext(AuthContext);
```

## 5. Signup and Signin Pages

Implement your signup and signin forms using the `useAuth` hook from your `AuthProvider`.

Example `pages/signup.tsx` (conceptual):

```typescript jsx
// pages/signup.tsx
import React, { useState } from 'react';
import { useAuth } from '../components/AuthProvider'; // Adjust path as needed

const SignUpPage = () => {
  const { signUp } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  // ... other state for hardwareLevel, learningGoals

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      await signUp(email, password, { name, softwareLevel /* ... */ });
      // Redirect to dashboard or show success
    } catch (error) {
      // Handle error
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} placeholder="Email" />
      <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} placeholder="Password" />
      <input type="text" value={name} onChange={(e) => setName(e.target.value)} placeholder="Name (optional)" />
      {/* ... other input fields */}
      <button type="submit">Sign Up</button>
    </form>
  );
};

export default SignUpPage;
```

## 6. Integrate with Neon Postgres (Backend)

On the backend, ensure your application integrates with Neon Postgres to store user data, as per the defined database schema. Better-Auth.com typically handles the password hashing, but your backend would be responsible for storing user profile data (like `software_level`, `hardware_level`, `learning_goals`) after successful authentication/registration events are forwarded from Better-Auth.com or handled directly.

## 7. Protect Routes

Implement route protection in your frontend or backend (or both) to restrict access to authenticated users for routes like `/profile`, chat history, and personalization settings.

Example Frontend Route Protection (conceptual, React Router):

```typescript jsx
// App.tsx or Router config
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import { useAuth } from './components/AuthProvider';
import ProfilePage from './pages/ProfilePage'; // Example protected page

const PrivateRoute = ({ children }) => {
  const { user, loading } = useAuth();
  if (loading) return <div>Loading...</div>; // Or a spinner
  return user ? children : <Navigate to="/signin" />;
};

const App = () => (
  <Router>
    <AuthProvider>
      <Routes>
        <Route path="/signin" element={<SignInPage />} />
        <Route path="/signup" element={<SignUpPage />} />
        <Route path="/profile" element={<PrivateRoute><ProfilePage /></PrivateRoute>} />
        {/* ... other routes */}
      </Routes>
    </AuthProvider>
  </Router>
);
```
