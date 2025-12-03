---
id: 9
title: "Update Chapter 1 Physical AI Spec"
stage: "spec"
date_iso: "2025-12-02"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "chapter-1-physical-ai-embodied-intelligence"
branch: "001-robotics-book"
user: "user"
command: "sp.specify"
labels: ["spec"]
links:
  spec: "specs/chapter-1-physical-ai-embodied-intelligence/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml:
  - "specs/chapter-1-physical-ai-embodied-intelligence/spec.md"
tests_yaml: []
---
## Prompt
```text
/sp.specify
title: "Chapter 1 — Physical AI & Embodied Intelligence"
description: "Full academic chapter explaining Physical AI, embodied intelligence, sensors, physics, control loops, and foundational humanoid robotics concepts."

version: "1.0"

## User Story
As an author, I want SpecifyPlus to generate Chapter 1 of the textbook with deep explanations of Physical AI, embodied intelligence, real-world robotics, and humanoid foundations.

## Requirements
### Functional
1. Provide 8–12 detailed sections explaining:
   - What Physical AI is
   - Embodiment
   - Sensorimotor intelligence
   - Physics-aware reasoning
   - Humanoid sensors
   - Robotic control loops
   - Modern humanoid robots (Optimus, Figure 01, Unitree G1, etc.)
2. Include tables, diagrams, callouts, robotics terminology lists.
3. Include ASCII diagrams for perception → planning → actuation loops.
4. Provide weekly learning outcomes for Weeks 1–2.
5. Output must be Docusaurus-ready Markdown.

### Non-Functional
- Clear academic style.
- No placeholders.
- All sections fully written.

### Success Criteria
- spec.md fully describes every section of Chapter 1.
- Ready for /sp.plan and /sp.implement.
```

## Response
The `specs/chapter-1-physical-ai-embodied-intelligence/spec.md` file has been updated with the provided specification for "Chapter 1 — Physical AI & Embodied Intelligence".
The user was using bash instead of powershell. The prerequisite check script failed previously due to this. The spec has been written to the correct path.
