title: "AI Assistant Widget Creation"
plan: "./specs/chatbot/plan.md"

tasks:

# ─────────────────────────────────────────────
# Phase 1 — Environment & File Structure
# ─────────────────────────────────────────────
- Create folder: src/components/AIAssistantWidget
- Create folder: src/context
- Create folder: src/utils

- Generate file: src/components/AIAssistantWidget/index.tsx
- Generate file: src/components/AIAssistantWidget/styles.module.css
- Generate file: src/context/chatContext.tsx
- Generate file: src/utils/sanitizeInput.ts

- Task: Install dependencies (motion, react-icons, marked, clsx)

# ─────────────────────────────────────────────
# Phase 2 — Context & Utilities
# ─────────────────────────────────────────────
- Populate src/utils/sanitizeInput.ts with input sanitization regex/logic
- Populate src/context/chatContext.tsx with:
    * React Context definition
    * ChatProvider component
    * State: isOpen, setIsOpen
    * State: draftText, setDraftText, clearDraftText
    * Export useChat hook

# ─────────────────────────────────────────────
# Phase 3 — Styling Implementation
# ─────────────────────────────────────────────
- Populate src/components/AIAssistantWidget/styles.module.css with:
    * CSS Variables for Light/Dark themes (--cb-bg, --cb-text, etc.)
    * Floating Button (.floatingBtn) styling
    * Modal Container (.modal) with glassmorphism & transitions
    * Header Layout (.header) with Avatar and Action buttons
    * Chat Content Area (.messagesArea, .welcome, .suggestions)
    * Message Bubbles (.bubbleUser, .bubbleBot)
    * Input Field & Send Button (.inputContainer, .sendBtn)
    * Loading Animation (.typingIndicator)
    * Mobile Responsiveness (@media queries)

# ─────────────────────────────────────────────
# Phase 4 — Component Logic & UI
# ─────────────────────────────────────────────
- Populate src/components/AIAssistantWidget/index.tsx with:
    * Imports (React, clsx, motion, react-icons, marked, styles, hooks)
    * Interface definition for ChatMessage
    * Main AIAssistantWidget component structure
    * State initialization (messages, input, isLoading, isLarge, showWelcome)
    * Context integration (useChat, useDocusaurusContext)
    * useEffect for handling draftText auto-fill and focus
    * useEffect for auto-scrolling to bottom
    * sendMessage async function (fetch API, error handling, loading state)
    * JSX Render: Floating Button (motion.button)
    * JSX Render: Modal (AnimatePresence + motion.div)
    * JSX Render: Message List (mapping state to bubbles with Markdown parsing)
    * JSX Render: Input Area (Keydown handling, Send button)

# ─────────────────────────────────────────────
# Phase 5 — Docusaurus Integration
# ─────────────────────────────────────────────
- Task: Update docusaurus.config.js to include BACKEND_URL in customFields
- Task: Wrap the Docusaurus application root with ChatProvider
- Task: Insert <AIAssistantWidget /> into the site Layout (e.g., in a Layout wrapper or Footer swizzle)
- Verify: Widget toggles correctly via Floating Button
- Verify: Markdown rendering in bot responses works
- Verify: Dark mode switching updates widget colors dynamically