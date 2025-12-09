import React from 'react';
import styles from './styles.module.css';

// Simple inline icons for the tech stack
const RosIcon = () => (
  <svg className={styles.icon} viewBox="0 0 24 24" fill="currentColor"><path d="M12 2L2 7l10 5 10-5-10-5zm0 9l2.5-1.25L12 8.5l-2.5 1.25L12 11zm0 2.5l-5-2.5-5 2.5 10 5 10-5-5-2.5-5 2.5z"/></svg>
);
const NvidiaIcon = () => (
  <svg className={styles.icon} viewBox="0 0 24 24" fill="currentColor"><path d="M4 18h16v-2H4v2zm0-5h16v-2H4v2zm0-7v2h16V6H4z"/></svg>
);
const PythonIcon = () => (
  <svg className={styles.icon} viewBox="0 0 24 24" fill="currentColor"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/></svg>
);
const ChipIcon = () => (
  <svg className={styles.icon} viewBox="0 0 24 24" fill="currentColor"><path d="M6 2h12v2H6V2zm0 18h12v2H6v-2zm14-16v16h2V4h-2zM4 4v16H2V4h2zm11 9h-2V9h2v4zm-4 0H9V9h2v4z"/></svg>
);

const techList = [
  { name: 'ROS 2 Humble', Icon: RosIcon },
  { name: 'NVIDIA Isaac Sim', Icon: NvidiaIcon },
  { name: 'Python & C++', Icon: PythonIcon },
  { name: 'Micro-Controllers', Icon: ChipIcon },
];

export default function TechStackSection() {
  return (
    <section className={styles.section}>
      <div className="container">
        <h2 className={styles.title}>Powered by Industry Standard Tools</h2>
        <div className={styles.grid}>
          {techList.map((item, idx) => (
            <div key={idx} className={styles.card}>
              <item.Icon />
              <p className={styles.toolName}>{item.name}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}