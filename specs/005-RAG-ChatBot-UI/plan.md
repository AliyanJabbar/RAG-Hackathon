# Development Plan: AI Assistant Widget

## Phase 1: Environment & Architecture Setup
**Goal:** Prepare the Docusaurus environment, install necessary libraries, and establish the file structure.
1.  Install NPM packages.
2.  Create directory structure in `src/`.
3.  Set up the global `ChatContext` to handle the modal state and text selection logic across the documentation site.

## Phase 2: UI Construction (Stateless)
**Goal:** Build the visual components using HTML and CSS Modules without complex logic.
1.  Define CSS variables for Light/Dark mode in `styles.module.css`.
2.  Build the **Floating Button**.
3.  Build the **Modal Skeleton** (Header, Message Area, Input Area).
4.  Style the **Message Bubbles** (User vs. Bot) and the **Welcome Screen**.
5.  Implement responsive media queries for mobile devices.

## Phase 3: Logic & State Implementation
**Goal:** Make the component interactive locally (no API yet).
1.  Implement `useState` for input, messages, and UI toggles (maximize/minimize).
2.  Connect the `ChatContext` for opening/closing the modal.
3.  Implement the logic to handle `draftText` (pre-filling input from page selection).
4.  Implement local message handling (typing, hitting enter, adding message to local state).
5.  Implement `scrollToBottom` logic using `useRef`.

## Phase 4: API Integration & Advanced Features
**Goal:** Connect to the backend and polish the UX.
1.  Implement the `fetch` logic to POST data to the backend.
2.  Handle loading states (Typing indicator) and Error states.
3.  Integrate `marked` to render Markdown in bot responses safely.
4.  Add `motion/react` animations for Modal entrance/exit.
5.  Final testing across themes and devices.