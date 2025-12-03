// robotics_book_content/src/pages/dashboard.tsx

import React from 'react';
import Layout from '@theme/Layout'; // Assuming Docusaurus Layout

const DashboardPage: React.FC = () => {
  return (
    <Layout title="Dashboard" description="Protected Dashboard Page">
      <div style={{ padding: '20px', maxWidth: '800px', margin: 'auto' }}>
        <h1>Welcome to Your Dashboard!</h1>
        <p>This is a protected page, accessible only to authenticated users.</p>
        <p>You can view your chat history, personalization settings, and other private content here.</p>
      </div>
    </Layout>
  );
};

export default DashboardPage;
