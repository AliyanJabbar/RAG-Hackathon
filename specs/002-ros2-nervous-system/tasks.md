title: "Chapter 2 — The Robotic Nervous System (ROS 2)"
plan: "./specs/chapter-2-robotic-nervous-system/plan.md"

tasks:

# ─────────────────────────────────────────────
# Phase 1 — Folder Structure
# ─────────────────────────────────────────────

- Create folder: docs/chapter-2-robotic-nervous-system
- Create folder: docs/chapter-2-robotic-nervous-system/assets

- Generate file: docs/chapter-2-robotic-nervous-system/index.md
- Generate file: docs/chapter-2-robotic-nervous-system/01-ros2-middleware-and-architecture.md
- Generate file: docs/chapter-2-robotic-nervous-system/02-communication-topics-services-actions.md
- Generate file: docs/chapter-2-robotic-nervous-system/03-workspaces-packages-and-launching.md
- Generate file: docs/chapter-2-robotic-nervous-system/04-agent-integration-and-control-pipelines.md
- Generate file: docs/chapter-2-robotic-nervous-system/05-robot-description-urdf-xacro.md
- Generate file: docs/chapter-2-robotic-nervous-system/06-sensing-tf2-and-environment-mapping.md
- Generate file: docs/chapter-2-robotic-nervous-system/07-humanoid-control-with-ros2-control.md
- Generate file: docs/chapter-2-robotic-nervous-system/08-debugging-visualization-and-tools.md

- Generate file: docs/chapter-2-robotic-nervous-system/assets/diagrams.txt

# ─────────────────────────────────────────────
# Phase 2 — Content Population
# ─────────────────────────────────────────────

- Populate index.md with chapter overview, ROS 2 learning roadmap, and table of contents
- Write 8–9 academic sections based on spec.md

- Populate ros2-middleware-and-architecture.md with DDS overview, QoS tables, node architecture, executors, callback groups, and ROS graph diagrams
- Populate communication-topics-services-actions.md with detailed explanations, comparison tables, lifecycle nodes, and message structure examples
- Populate workspaces-packages-and-launching.md with colcon workspace structure, package composition, launch system, config files, and YAML parameter examples
- Populate agent-integration-and-control-pipelines.md with AI Agent → ROS control pipeline, rclpy integration, planning–control loops, and full humanoid command example
- Populate robot-description-urdf-xacro.md with URDF links/joints, Xacro macros, FK chain diagrams, and URDF vs SDF comparison table
- Populate sensing-tf2-and-environment-mapping.md with IMU/camera/LiDAR details, TF2 transform trees, sensor data flow, and coordinate frame diagrams
- Populate humanoid-control-with-ros2-control.md with ros2_control architecture, controllers, hardware interfaces, joint trajectory control, and humanoid walking loops
- Populate debugging-visualization-and-tools.md with RViz2, RQt tools, ros2 doctor, ros2 bag, and debugging workflow steps

# ─────────────────────────────────────────────
# Phase 3 — Diagram & Asset Generation
# ─────────────────────────────────────────────

- Generate ASCII diagrams in diagrams.txt:
    * ROS 2 computation graph (nodes → topics → services → actions)
    * DDS communication layers
    * Agent → Planner → ROS → Controller → Actuator pipeline
    * URDF joint-tree diagram
    * TF2 transform tree for humanoid robot
    * Humanoid joint trajectory control loop
    * Sensor fusion pipeline (IMU + camera + LiDAR)
    
# ─────────────────────────────────────────────
# Phase 4 — Docusaurus Integration
# ─────────────────────────────────────────────

- Add proper frontmatter to each .md file
- Add slugs for clean URLs
- Update sidebar configuration to include all Chapter 2 files
- Ensure markdown tables, callouts, and diagrams render properly
- Confirm all functional & non-functional requirements from spec.md are satisfied
- Ensure chapter is ready for /sp.implement
