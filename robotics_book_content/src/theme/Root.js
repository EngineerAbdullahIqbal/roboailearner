import React, { useState, useEffect } from 'react';
import ChatWidget from '../components/ChatWidget'; // Path adjusted

function Root({ children }) {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [fabPosition, setFabPosition] = useState({ top: 0, left: 0 });
  const [showAskFab, setShowAskFab] = useState(false);

  const toggleChat = () => {
    setIsChatOpen((prev) => !prev);
    // Clear selected text when opening/closing chat, unless it's just been selected to open chat
    if (isChatOpen) {
      setSelectedText('');
      setShowAskFab(false);
    }
  };

  const clearSelectedText = () => {
    setSelectedText('');
    setShowAskFab(false);
  };

  const handleMouseUp = (event) => {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text.length > 0) {
      setSelectedText(text);
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      setFabPosition({
        top: rect.top + window.scrollY - 40, // Position above selection
        left: rect.left + window.scrollX + rect.width / 2 - 50, // Center with selection
      });
      setShowAskFab(true);
    } else {
      setSelectedText('');
      setShowAskFab(false);
    }
  };

  const handleAskAboutSelection = () => {
    setIsChatOpen(true); // Open chat and ChatWidget will handle the selectedText
    setShowAskFab(false); // Hide the ask fab immediately
  };

  useEffect(() => {
    window.addEventListener('mouseup', handleMouseUp);
    return () => {
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  return (
    <>
      {children}
      <ChatWidget
        isOpen={isChatOpen}
        toggleChat={toggleChat}
        selectedText={selectedText}
        clearSelectedText={clearSelectedText}
      />

      {/* Main FAB for opening/closing chat */}
      {!isChatOpen && (
        <button
          onClick={toggleChat}
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            backgroundColor: 'var(--ifm-color-primary)', /* Use Docusaurus primary color */
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '60px',
            height: '60px',
            fontSize: '30px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.2)',
            zIndex: 1001,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
          }}
        >
          💬 {/* Chat bubble emoji */}
        </button>
      )}

      {/* FAB for asking about selected text */}
      {showAskFab && selectedText && !isChatOpen && (
        <button
          onClick={handleAskAboutSelection}
          style={{
            position: 'absolute',
            top: `${fabPosition.top}px`,
            left: `${fabPosition.left}px`,
            backgroundColor: 'var(--ifm-color-info)', /* Use Docusaurus info color for accent */
            color: 'white',
            border: 'none',
            borderRadius: '20px',
            padding: '8px 15px',
            fontSize: '14px',
            cursor: 'pointer',
            boxShadow: '0 2px 8px rgba(0, 0, 0, 0.1)',
            zIndex: 1002,
            whiteSpace: 'nowrap',
          }}
        >
          Ask AI about this
        </button>
      )}
    </>
  );
}

export default Root;
