# Data Model: AI-Generated Book Project

## Entities

### Book
- **Description**: Represents the entire book, encompassing its structure and meta-information.
- **Attributes**:
    - `title`: String (e.g., "Physical AI & Humanoid Robotics Course")
    - `target_audience`: String (e.g., "Undergraduate students with programming experience")
    - `total_chapters`: Integer (Range: 10-15)
    - `chapter_length_min`: Integer (Min words per chapter: 1500)
    - `chapter_length_max`: Integer (Max words per chapter: 2500)
    - `learning_progression`: String (e.g., "Beginner to Advanced")

### Chapter
- **Description**: An individual chapter within the book, containing structured educational content.
- **Attributes**:
    - `chapter_id`: String (Unique identifier)
    - `title`: String
    - `order`: Integer (Sequence in the book)
    - `learning_objectives`: List of Strings (3-5 clear objectives)
    - `theoretical_foundation`: String (Markdown content)
    - `practical_examples`: List of Objects (e.g., `{ type: 'code', content: '...', language: 'python', screenshot_url: '...' }`)
    - `real_world_use_cases`: String (Markdown content)
    - `hands_on_exercises`: String (Markdown content)
    - `summary_key_takeaways`: String (Markdown content)
    - `further_reading_resources`: List of Strings (URLs or titles)
    - `transition_to_next_chapter`: String (Brief text for continuity)
    - `content_body`: String (Main chapter content in Markdown, including conceptual explanations, tutorials, diagrams, best practices, pitfalls)

### User
- **Description**: Represents a registered user of the book platform, enabling personalized experiences.
- **Attributes**:
    - `user_id`: String (Unique identifier)
    - `email`: String (Unique, for Better-Auth.com)
    - `software_background`: List of Strings (e.g., "Python", "ROS 2", "JavaScript")
    - `hardware_background`: List of Strings (e.g., "NVIDIA Jetson", "Robotics kits")
    - `personalization_preferences`: JSON Object (Key-value pairs for content tailoring, e.g., `{ 'difficulty': 'advanced', 'focus_area': 'simulation' }`)
    - `translation_preference`: String (e.g., "English", "Urdu")

### DocumentChunk (for RAG System)
- **Description**: A segment of book content used for retrieval in the RAG chatbot.
- **Attributes**:
    - `chunk_id`: String (Unique identifier)
    - `chapter_id`: String (Foreign key to Chapter)
    - `content`: String (Text of the chunk)
    - `embedding`: Vector (High-dimensional vector representation of `content`)
    - `metadata`: JSON Object (e.g., `{ 'chapter_title': '...', 'section': '...', 'page_number': '...' }`)

### ChatMessage (for RAG System)
- **Description**: A message within a user's chat session with the RAG chatbot.
- **Attributes**:
    - `message_id`: String (Unique identifier)
    - `user_id`: String (Foreign key to User)
    - `session_id`: String (Identifier for a continuous chat session)
    - `timestamp`: DateTime
    - `sender`: String (e.g., "user", "chatbot")
    - `content`: String (Text of the message)
    - `retrieved_chunks`: List of Strings (IDs of DocumentChunks used for response generation)
