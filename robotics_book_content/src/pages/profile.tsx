// robotics_book_content/src/pages/profile.tsx

import React, { useState, useEffect } from 'react';
import { useAuth } from '../components/AuthProvider';
import { useNavigate } from 'react-router-dom';

const ProfilePage: React.FC = () => {
  const { isAuthenticated, user, loading, updateProfile, logout } = useAuth();
  const navigate = useNavigate();

  const [name, setName] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState('');
  const [hardwareLevel, setHardwareLevel] = useState('');
  const [learningGoals, setLearningGoals] = useState('');
  const [isEditing, setIsEditing] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  useEffect(() => {
    if (!loading && !isAuthenticated) {
      navigate('/signin'); // Redirect to signin if not authenticated
    } else if (user) {
      setName(user.name || '');
      setSoftwareLevel(user.softwareLevel || '');
      setHardwareLevel(user.hardwareLevel || '');
      setLearningGoals(user.learningGoals || '');
    }
  }, [isAuthenticated, user, loading, navigate]);

  const handleUpdate = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setSuccess(null);
    try {
      const updatedData = { name, softwareLevel, hardwareLevel, learningGoals };
      const result = await updateProfile(updatedData);
      if (result) {
        setSuccess('Profile updated successfully!');
        setIsEditing(false);
      } else {
        setError('Failed to update profile. Please try again.');
      }
    } catch (err: any) {
      setError(err.message || 'An unexpected error occurred during profile update.');
    }
  };

  const handleLogout = async () => {
    await logout();
    navigate('/signin'); // Redirect to signin after logout
  };

  if (loading) {
    return <p>Loading profile...</p>;
  }

  if (!isAuthenticated) {
    return <p>Please sign in to view your profile.</p>;
  }

  return (
    <div style={{ padding: '20px', maxWidth: '600px', margin: 'auto' }}>
      <h1>User Profile</h1>
      {error && <p style={{ color: 'red' }}>{error}</p>}
      {success && <p style={{ color: 'green' }}>{success}</p>}

      {!isEditing ? (
        <div>
          <p><strong>Email:</strong> {user?.email}</p>
          <p><strong>Name:</strong> {name || 'N/A'}</p>
          <p><strong>Software Level:</strong> {softwareLevel || 'N/A'}</p>
          <p><strong>Hardware Level:</strong> {hardwareLevel || 'N/A'}</p>
          <p><strong>Learning Goals:</strong> {learningGoals || 'N/A'}</p>
          <button onClick={() => setIsEditing(true)}>Edit Profile</button>
          <button onClick={handleLogout} style={{ marginLeft: '10px' }}>Logout</button>
        </div>
      ) : (
        <form onSubmit={handleUpdate}>
          <div>
            <label>Name:</label>
            <input type="text" value={name} onChange={(e) => setName(e.target.value)} disabled={loading} />
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
            <textarea value={learningGoals} onChange={(e) => setLearningGoals(e.target.value)} rows={4} disabled={loading} />
          </div>
          <button type="submit" disabled={loading}>
            {loading ? 'Updating...' : 'Save Changes'}
          </button>
          <button type="button" onClick={() => setIsEditing(false)} style={{ marginLeft: '10px' }} disabled={loading}>
            Cancel
          </button>
        </form>
      )}
    </div>
  );
};

export default ProfilePage;
