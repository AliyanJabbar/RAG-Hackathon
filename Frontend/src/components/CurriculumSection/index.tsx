import React from 'react';
import styles from './styles.module.css';

const modules = [
  {
    number: '01',
    title: 'Foundations & Simulation',
    desc: 'Master the math behind rigid body dynamics. Setup your environment with Docker, ROS 2 Humble, and NVIDIA Isaac Sim.',
    tags: ['Python', 'C++', 'Docker', 'Linux']
  },
  {
    number: '02',
    title: 'Robot Modeling (URDF)',
    desc: 'Design your humanoid from scratch. Learn about links, joints, inertia matrices, and exporting valid URDF/Xacro files from CAD.',
    tags: ['CAD', 'URDF', 'Gazebo', 'XML']
  },
  {
    number: '03',
    title: 'Kinematics & Control',
    desc: 'Implement Forward & Inverse Kinematics (IK). Build stability controllers using ZMP (Zero Moment Point) and MPC.',
    tags: ['Inverse Kinematics', 'Control Theory', 'MPC', 'Eigen']
  },
  {
    number: '04',
    title: 'Vision & Autonomy',
    desc: 'Give your robot eyes. Integrate OpenCV for object detection and V-SLAM for mapping unknown environments.',
    tags: ['OpenCV', 'YOLO', 'SLAM', 'Nav2']
  },
];

export default function CurriculumSection() {
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        <div className={styles.layout}>
          
          {/* Left Side */}
          <div className={styles.header}>
            <span className={styles.tagline}>The Roadmap</span>
            <h2 className={styles.title}>From Zero to <br/> Walking Robot</h2>
            <p className={styles.description}>
              A comprehensive curriculum designed to take you from basic scripts to deploying code on physical humanoid hardware.
            </p>
          </div>

          {/* Right Side */}
          <div className={styles.moduleList}>
            {modules.map((mod, idx) => (
              <div key={idx} className={styles.moduleCard}>
                <div className={styles.moduleNumber}>{mod.number}</div>
                <div className={styles.moduleContent}>
                  <h3>{mod.title}</h3>
                  <p>{mod.desc}</p>
                  <div className={styles.tagContainer}>
                    {mod.tags.map(tag => (
                      <span key={tag} className={styles.tag}>{tag}</span>
                    ))}
                  </div>
                </div>
              </div>
            ))}
          </div>

        </div>
      </div>
    </section>
  );
}