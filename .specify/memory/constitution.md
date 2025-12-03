<!-- Sync Impact Report -->
<!--
Version change: 1.0.0 -> 1.0.0
Modified principles: None
Added principles: None
Added sections: None
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending (Constitution Check section needs update)
  - .specify/templates/spec-template.md: ⚠ pending (Scope/requirements alignment needs update)
  - .specify/templates/tasks-template.md: ⚠ pending (Task categorization needs update)
  - .specify/templates/commands/*.md: N/A (no command files found, check for outdated references is implicitly resolved by absence)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. AI-first writing workflow
Claude Code + Subagents: Utilize AI agents for content generation and workflow automation.

### II. Spec-driven development
Spec-Kit Plus: Design and implement features using a specification-first approach.

### III. Atomic chapters
Deterministic and structured: Each chapter must be self-contained and follow a consistent structure.

### IV. Reusable components (MDX)
MDX: Leverage MDX for creating reusable content blocks and interactive elements.

### V. Clear sidebar hierarchy
Logical Navigation: Ensure an intuitive and easy-to-navigate content structure.

### VI. Accessibility & multilingual support
Urdu Translation Layer: Implement chapter-level translation for broader accessibility.

### VII. Personalization-aware architecture
Chapter-level personalization: Design the system to support individualized learning paths.

### VIII. Docusaurus 3
Modern Documentation Platform: Utilize Docusaurus 3 for robust and extensible documentation.

## Hackathon Objectives

- Fully structured textbook in Docusaurus.
- RAG chatbot that answers questions from the textbook.
- Personalization layer (chapter-level).
- Urdu translation layer (chapter-level).
- Optional bonus: Claude Subagents + reusable skills.

## Tech Stack & Constraints

Tech Stack:
- Docusaurus 3
- Next.js API Route OR FastAPI backend
- OpenAI Agents SDK / ChatKit
- Neon Serverless Postgres
- Qdrant Cloud Free Tier
- Better-Auth for signup/signin
- Spec-Kit Plus for designing book structure

Constraints:
- Must publish on GitHub Pages or Vercel.
- Demo video < 90 seconds.
- All chapter content must be deterministic and structured.

## Governance
This constitution supersedes all other project practices. Amendments require thorough documentation, stakeholder approval, and a clear migration plan. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-02 | **Last Amended**: 2025-12-02
