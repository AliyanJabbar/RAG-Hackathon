title: "Chapter 1 — Physical AI & Embodied Intelligence"
plan: "./specs/chapter-1-physical-ai-embodied-intelligence/plan.md"

tasks:

# ─────────────────────────────────────────────
# Phase 1 — Folder Structure
# ─────────────────────────────────────────────
- Create folder: docs/chapter-1-physical-ai
- Create folder: docs/chapter-1-physical-ai/assets

- Generate file: docs/chapter-1-physical-ai/index.md
- Generate file: docs/chapter-1-physical-ai/01-what-is-physical-ai.md
- Generate file: docs/chapter-1-physical-ai/02-embodiment.md
- Generate file: docs/chapter-1-physical-ai/03-sensorimotor-intelligence.md
- Generate file: docs/chapter-1-physical-ai/04-physics-aware-reasoning.md
- Generate file: docs/chapter-1-physical-ai/05-humanoid-sensors.md
- Generate file: docs/chapter-1-physical-ai/06-robotic-control-loops.md
- Generate file: docs/chapter-1-physical-ai/07-modern-humanoids.md
- Generate file: docs/chapter-1-physical-ai/08-perception-planning-action-loop.md
- Generate file: docs/chapter-1-physical-ai/09-terminology.md
- Generate file: docs/chapter-1-physical-ai/10-learning-outcomes.md

- Generate file: docs/chapter-1-physical-ai/assets/diagrams.txt

# ─────────────────────────────────────────────
# Phase 2 — Content Population
# ─────────────────────────────────────────────
- Populate index.md with chapter overview, learning roadmap, and TOC
- Write 8–12 academic sections based on spec.md

- Populate what-is-physical-ai.md with definitions, comparison table (digital AI vs physical AI), and conceptual diagrams
- Populate embodiment.md with embodied cognition theory, biological analogies, robotics examples
- Populate sensorimotor-intelligence.md with explanations, sensor–action loops, diagrams, and examples
- Populate physics-aware-reasoning.md with friction, CoM, balance, inertia, torque diagrams
- Populate humanoid-sensors.md including cameras, IMU, LiDAR, tactile sensors, F/T sensors, comparison tables
- Populate robotic-control-loops.md with PID, state estimation, low-level control, latency diagrams
- Populate modern-humanoids.md with profiles of Optimus, Figure 01, Unitree G1, Agility Digit, comparison table
- Populate perception-planning-action-loop.md with ASCII diagrams for:
    * perception → world model → planning → control → actuation
    * sensor fusion → decision loop
- Populate terminology.md with 40–60 robotics terms
- Populate learning-outcomes.md with Week 1 and Week 2 learning objectives

# ─────────────────────────────────────────────
# Phase 3 — Diagram & Asset Generation
# ─────────────────────────────────────────────
- Generate ASCII diagrams in diagrams.txt:
    * sensorimotor loop
    * feedback control loop
    * perception → planning → action
    * humanoid sensor placement layout
    * physics-model reference diagram (CoM → torque → stability)

# ─────────────────────────────────────────────
# Phase 4 — Docusaurus Integration
# ─────────────────────────────────────────────
- Add proper frontmatter to each .md file
- Add slugs for clean URLs
- Update sidebar configuration to include all Chapter 1 files
- Ensure markdown tables, callouts, and diagrams render pronon-functional requirements from spec.md satisfied
- Confirm chapter is ready for /sp.implement