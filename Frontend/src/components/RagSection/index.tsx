import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';
import { useChat } from '@site/src/context/chatContext';
import { RiRobot2Line } from 'react-icons/ri';

export default function RagSection() {
  
    const { openWithText } = useChat();
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        <div className={styles.contentGrid}>
          
          {/* Left Side: Text Content */}
          <div className="text-content">
            <span className={styles.label}>Course Feature</span>
            <h2 className={styles.title}>
              Master Robotics with <br />
              <span className={styles.gradientText}>Context-Aware AI</span>
            </h2>
            <p className={styles.description}>
              Don't get stuck on complex math or hardware specs. Our 
              <strong> Humanoid Robotics RAG Course</strong> includes an AI assistant trained on 
              thousands of pages of Kinematics, ROS2 documentation, and URDF files. 
              Get instant answers to build your robot faster.
            </p>
            <div className={styles.buttons}>
              <button
                className="button button--primary button--lg"
                onClick={() => openWithText('')}>
                Talk To AI
              </button>
              <Link
                className="button button--secondary button--lg"
                to="docs/chapter-1-physical-ai">
                View Curriculum
              </Link>
            </div>
          </div>

          {/* Right Side: Visual Representation */}
          <div className={styles.visualContainer}>
            <div className={styles.blob} /> {/* Glowing background effect */}
            
            <div className={styles.chatCard}>
              <div className={styles.chatHeader}>
               <div className={styles.inHeader}>
                 <div className={styles.dot} />
                <span className={styles.headerText}>Robotics Expert AI</span>
               </div>
               <div>
                <RiRobot2Line size={30}/>
               </div>
              </div>

              {/* User asks a specific Robotics Question */}
              <div className={clsx(styles.message, styles.userMessage)}>
                <p style={{margin: 0}}>How do I ensure stability during the walking cycle?</p>
              </div>

              {/* AI Answers with RAG content (ZMP concept) */}
              <div className={clsx(styles.message, styles.aiMessage)}>
                <span className={styles.aiBadge}>✨ RAG Retrieved</span>
                <p style={{margin: 0}}>
                  To maintain dynamic stability, you must keep the <strong>Zero Moment Point (ZMP)</strong> within the support polygon.
                  <br/><br/>
                  In your control loop, calculate it as:
                  <br/>
                  <code style={{color: '#e2e8f0', background:'rgba(0,0,0,0.3)', display:'block', marginTop:'8px', padding:'4px'}}>
                    x_zmp = x - (z_c / g) * x_accel
                  </code>
                </p>
              </div>
            </div>
          </div>

        </div>
      </div>
    </section>
  );
}