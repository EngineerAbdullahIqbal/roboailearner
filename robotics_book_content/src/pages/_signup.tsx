// robotics_book_content/src/pages/signup.tsx

import React, { useState } from 'react';
import { useAuth } from '../components/AuthProvider';
import { useNavigate } from 'react-router-dom'; // Assuming react-router-dom is used for navigation

const SignupPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState('');
  const [hardwareLevel, setHardwareLevel] = useState('');
  const [learningGoals, setLearningGoals] = useState('');
  const [error, setError] = useState<string | null>(null);
  const { signup, loading } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      const success = await signup({ email, password, softwareLevel, hardwareLevel, learningGoals });
      if (success) {
        navigate('/profile'); // Redirect to profile or a success page
      } else {
        setError('Signup failed. Please try again.');
      }
    } catch (err: any) {
      setError(err.message || 'An unexpected error occurred during signup.');
    }
  };

  return (
    <div style={{ padding: '20px', maxWidth: '500px', margin: 'auto' }}>
      <h1>Sign Up</h1>
      <form onSubmit={handleSubmit}>
        {error && <p style={{ color: 'red' }}>{error}</p>}
        <div>
          <label>Email:</label>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            disabled={loading}
          />
        </div>
        <div>
          <label>Password:</label>
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            disabled={loading}
          />
        </div>
        <div>
          <label>Software Experience:</label>
          <select value={softwareLevel} onChange={(e) => setSoftwareLevel(e.target.value)} disabled={loading}>
            <option value="">Select Level</option>
            <option value="Beginner">Beginner</option>
            <option value="Intermediate">Intermediate</option>
            <option value="Advanced">Advanced</option>
          </select>
        </div>
        <div>
          <label>Hardware Experience:</label>
          <select value={hardwareLevel} onChange={(e) => setHardwareLevel(e.target.value)} disabled={loading}>
            <option value="">Select Level</option>
            <option value="None">None</option>
            <option value="Hobbyist">Hobbyist</option>
            <option value="Professional">Professional</option>
          </select>
        </div>
        <div>
          <label>Learning Goals:</label>
          <textarea
            value={learningGoals}
            onChange={(e) => setLearningGoals(e.target.value)}
            rows={4}
            disabled={loading}
          />
        </div>
        <button type="submit" disabled={loading}>
          {loading ? 'Signing Up...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupPage;
