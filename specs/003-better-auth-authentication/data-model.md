# Data Model for Better-Auth.com Authentication

## Entity: `User`

Represents an authenticated user within the system.

### Fields:

*   **id**:
    *   **Type**: UUID
    *   **Description**: Unique identifier for the user.
    *   **Constraints**: Primary Key, Default: `uuid_generate_v4()`
*   **email**:
    *   **Type**: VARCHAR(255)
    *   **Description**: User's email address. Used for login.
    *   **Constraints**: Unique, Not Null
*   **password_hash**:
    *   **Type**: VARCHAR(255)
    *   **Description**: Hashed password for security.
    *   **Constraints**: Not Null
*   **name**:
    *   **Type**: VARCHAR(255)
    *   **Description**: User's full name.
    *   **Constraints**: Optional
*   **software_level**:
    *   **Type**: VARCHAR(50)
    *   **Description**: User's self-declared software experience level.
    *   **Constraints**: Optional, Expected values: 'Beginner', 'Intermediate', 'Advanced'
*   **hardware_level**:
    *   **Type**: VARCHAR(50)
    *   **Description**: User's self-declared hardware experience level.
    *   **Constraints**: Optional, Expected values: 'None', 'Hobbyist', 'Professional'
*   **learning_goals**:
    *   **Type**: TEXT
    *   **Description**: Free-form text describing the user's learning objectives.
    *   **Constraints**: Optional
*   **created_at**:
    *   **Type**: TIMESTAMP
    *   **Description**: Timestamp of user creation.
    *   **Constraints**: Default: `NOW()`
*   **updated_at**:
    *   **Type**: TIMESTAMP
    *   **Description**: Timestamp of last user update.
    *   **Constraints**: Default: `NOW()`

### Relationships:

*   Currently, the `User` entity is standalone. Future features (e.g., chat history, personalization settings) will likely introduce foreign key relationships to this table.
