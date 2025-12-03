import React, { useState, useEffect, useRef } from 'react';

const API_BASE_URL = 'http://localhost:8000'; // Adjust to your backend URL

const ChatWidget = ({ isOpen, toggleChat, selectedText, clearSelectedText }) => {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [conversationId, setConversationId] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    if (isOpen) {
      if (!conversationId && messages.length === 0) {
        setMessages([{ sender: 'bot', text: 'Hello! Ask me anything about the Robotics book.', isComplete: true }]);
      }
      if (selectedText) {
        handleExplainSelection(selectedText);
        clearSelectedText();
      }
    }
  }, [isOpen, selectedText, clearSelectedText]);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleStream = async (response) => {
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let botMessage = { sender: 'bot', text: '', isComplete: false };
    
    setMessages((prev) => [...prev, botMessage]);

    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;
        
        const chunk = decoder.decode(value);
        botMessage.text += chunk;
        
        setMessages((prev) => {
          const newMessages = [...prev];
          newMessages[newMessages.length - 1] = { ...botMessage };
          return newMessages;
        });
      }
      
      setMessages((prev) => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = { ...botMessage, isComplete: true };
        return newMessages;
      });
    } catch (error) {
      console.error("Stream reading error:", error);
      setMessages((prev) => [...prev, { sender: 'bot', text: `Error: ${error.message}`, isComplete: true }]);
    } finally {
      setIsLoading(false);
    }
  };

  const sendMessage = async (messageText, selectionContext = null) => {
    if (messageText.trim() === '') return;

    const userMessage = { sender: 'user', text: messageText, isComplete: true };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const payload = {
        message: messageText,
        conversation_id: conversationId,
        selection_context: selectionContext
      };

      const response = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      await handleStream(response);

    } catch (error) {
      console.error("Fetch error:", error);
      setMessages((prev) => [...prev, { sender: 'bot', text: 'Error: Could not connect to chat server.', isComplete: true }]);
      setIsLoading(false);
    }
  };

  const handleExplainSelection = (selection) => {
    const context = {
      text: selection,
      source_url: window.location.pathname // Send current page URL as source
    };
    sendMessage(`Explain this selection: "${selection}"`, context);
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage(input);
  };

  if (!isOpen) return null;

  return (
    <div style={{
      position: 'fixed',
      bottom: '100px',
      right: '20px',
      width: '350px',
      height: '500px',
      backgroundColor: 'var(--ifm-background-color)', /* Use Docusaurus background color */
      border: '1px solid var(--ifm-color-emphasis-200)', /* Use Docusaurus border color */
      borderRadius: '8px',
      boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
      display: 'flex',
      flexDirection: 'column',
      zIndex: 1000,
      fontFamily: 'var(--ifm-font-family-base)'
    }}>
      <div style={{
        padding: '10px',
        backgroundColor: 'var(--ifm-color-primary-light)', /* Use Docusaurus primary light for header */
        borderBottom: '1px solid var(--ifm-color-emphasis-300)',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        borderRadius: '8px 8px 0 0'
      }}>
        <h4 style={{ margin: 0, fontSize: '16px', color: 'var(--ifm-heading-color)' }}>Robotics AI Assistant</h4> {/* Use heading color */}
        <button onClick={toggleChat} style={{
          background: 'none',
          border: 'none',
          fontSize: '20px',
          cursor: 'pointer',
          color: 'var(--ifm-heading-color)' /* Use heading color for close button */
        }}>
          &times;
        </button>
      </div>
      <div style={{ flex: 1, overflowY: 'auto', padding: '10px', backgroundColor: 'var(--ifm-background-color)' }}>
        {messages.map((msg, index) => (
          <div key={index} style={{
            marginBottom: '10px',
            textAlign: msg.sender === 'user' ? 'right' : 'left'
          }}>
            <div style={{
              display: 'inline-block',
              padding: '8px 12px',
              borderRadius: '18px',
              backgroundColor: msg.sender === 'user' ? 'var(--ifm-color-primary)' : 'var(--ifm-background-surface-color)', /* Use Docusaurus colors */
              color: msg.sender === 'user' ? 'white' : 'var(--ifm-text-color)',
              maxWidth: '85%',
              wordWrap: 'break-word',
              whiteSpace: 'pre-wrap'
            }}>
              {msg.text}
            </div>
          </div>
        ))}
        {isLoading && <div style={{ textAlign: 'left', color: 'var(--ifm-color-emphasis-600)', fontSize: '12px', marginLeft: '10px' }}>Typing...</div>}
        <div ref={messagesEndRef} />
      </div>
      <form onSubmit={handleSubmit} style={{
        display: 'flex',
        padding: '10px',
        borderTop: '1px solid var(--ifm-color-emphasis-200)',
        backgroundColor: 'var(--ifm-background-color)',
        borderRadius: '0 0 8px 8px'
      }}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask a question..."
          style={{
            flex: 1,
            padding: '10px',
            border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: '20px',
            marginRight: '10px',
            fontSize: '14px',
            backgroundColor: 'var(--ifm-background-surface-color)',
            color: 'var(--ifm-text-color)'
          }}
          disabled={isLoading}
        />
        <button type="submit" style={{
          backgroundColor: 'var(--ifm-color-primary)',
          color: 'white',
          border: 'none',
          borderRadius: '20px',
          padding: '10px 15px',
          cursor: 'pointer',
          fontSize: '14px',
          opacity: isLoading ? 0.7 : 1
        }} disabled={isLoading}>
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatWidget;
