# AI Assistant Widget Specification

## 1. Project Overview
A floating AI Assistant Widget designed for a Docusaurus-based documentation site. The widget allows users to interact with an AI bot via a modal interface. It features text inputs, predefined suggestion chips, context-aware input (via page text selection), and markdown-formatted responses.

## 2. Technical Stack
*   **Framework:** React (functional components with Hooks).
*   **Platform:** Docusaurus v2/v3.
*   **Styling:** CSS Modules (`styles.module.css`) with `clsx` for conditional classes.
*   **Animation:** `motion/react` (Framer Motion).
*   **Icons:** `react-icons` (Fa, Bs, Lu, Io, Ri).
*   **Markdown Parsing:** `marked`.
*   **State Management:** React `useState`, `useRef`, and a custom `ChatContext`.

## 3. UI/UX Requirements

### 3.1 Floating Action Button (FAB)
*   **Position:** Fixed at bottom-right.
*   **Behavior:** Toggles the chat modal open/closed.
*   **Animation:** Scale down slightly on tap/click.

### 3.2 Chat Modal
*   **Dimensions:**
    *   Default: Fixed width (approx 35vw) and height (70vh).
    *   Large Mode: Expanded width/height.
    *   Mobile: Full screen (100vw/100vh).
*   **Header:**
    *   Bot Avatar with status dot (Green for active).
    *   Title ("AI Assistant") and Subtitle ("Online" or "Typing...").
    *   Actions: Refresh (Clear Chat), Maximize/Minimize, Close.
*   **Content Area:**
    *   **Empty State:** Show "Welcome" icon, greeting text, and "Suggested Questions" chips.
    *   **Active State:** Scrollable list of message bubbles.
    *   **User Bubble:** Right-aligned, dark background (in light mode), plain text.
    *   **Bot Bubble:** Left-aligned, light background, rendered Markdown HTML.
    *   **Loading:** Animated 3-dot typing indicator when awaiting API response.
*   **Input Area:**
    *   Pill-shaped input field.
    *   Send button (Paper Plane icon) inside the pill.
    *   Disabled state during loading.

### 3.3 Theming
*   Must support Light and Dark modes using CSS variables (`--cb-bg`, `--cb-text`, etc.).
*   Glassmorphism effect (`backdrop-filter: blur`).

## 4. Functional Requirements

### 4.1 Chat Logic
*   **Message History:** Maintain an array of `{ role: string, text: string }`.
*   **Sending:**
    *   POST request to `customFields.BACKEND_URL/chat`.
    *   Payload: JSON object containing the message history.
    *   Handle success (append bot reply) and error (append error message).
*   **Scrolling:** Auto-scroll to the bottom when new messages appear.

### 4.2 Context Integration
*   **Global State:** Consume `isOpen`, `setIsOpen`, `draftText`, and `clearDraftText` from `ChatContext`.
*   **Text Selection:** If `draftText` exists in context (selected from the doc page), automatically:
    1.  Open the modal.
    2.  Populate the input field.
    3.  Focus the input.
    4.  Clear the draft from context.

### 4.3 Utilities
*   **Sanitization:** Run user input through `sanitizeInput()` before processing.