import React, { useEffect, useState } from "react";
import { createPortal } from "react-dom";
import { motion, AnimatePresence } from "framer-motion"; // Changed to framer-motion standard
import { useChat } from "../../context/chatContext";
import { RiRobot2Line, RiSparkling2Fill } from "react-icons/ri";

const THEME_COLOR = "#25c2a0";

export default function SelectionTooltip() {
  const { openWithText } = useChat();
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);

  useEffect(() => {
    const handleMouseUp = () => {
      // Small delay to ensure selection is fully captured by the browser
      setTimeout(() => {
        const selection = window.getSelection();
        if (!selection || selection.isCollapsed || !selection.toString().trim()) {
          return;
        }

        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Calculate horizontal center and vertical top
        setPosition({
          x: rect.left + rect.width / 2,
          y: rect.top + window.scrollY,
        });
      }, 0);
    };

    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (!selection || selection.isCollapsed) {
        setPosition(null);
      }
    };

    const handleScroll = () => setPosition(null);

    document.addEventListener("mouseup", handleMouseUp);
    document.addEventListener("selectionchange", handleSelectionChange);
    window.addEventListener("scroll", handleScroll);

    return () => {
      document.removeEventListener("mouseup", handleMouseUp);
      document.removeEventListener("selectionchange", handleSelectionChange);
      window.removeEventListener("scroll", handleScroll);
    };
  }, []);

  const handleAsk = (e: React.MouseEvent) => {
    e.stopPropagation();
    e.preventDefault();
    const selection = window.getSelection()?.toString();
    if (selection) {
      openWithText(selection);
      window.getSelection()?.removeAllRanges();
      setPosition(null);
    }
  };

  if (typeof document === "undefined") return null;

  return createPortal(
    <AnimatePresence>
      {position && (
        <motion.div
          initial={{ opacity: 0, scale: 0.85, y: 10, x: "-50%" }}
          animate={{ opacity: 1, scale: 1, y: -55, x: "-50%" }}
          exit={{ opacity: 0, scale: 0.5, y: 0, transition: { duration: 0.15 } }}
          transition={{
            type: "spring",
            stiffness: 400,
            damping: 25,
          }}
          style={{
            position: "absolute",
            left: position.x,
            top: position.y,
            zIndex: 9999,
            pointerEvents: "auto",
          }}
        >
          {/* Tooltip Body */}
          <motion.button
            whileHover={{ scale: 1.05, backgroundColor: "#1fa88a" }}
            whileTap={{ scale: 0.95 }}
            onMouseDown={(e) => e.preventDefault()}
            onClick={handleAsk}
            style={{
              background: `linear-gradient(135deg, ${THEME_COLOR} 0%, #1e9e83 100%)`,
              color: "#fff",
              padding: "8px 16px",
              borderRadius: "12px",
              border: "1px solid rgba(255, 255, 255, 0.2)",
              display: "flex",
              alignItems: "center",
              gap: "8px",
              cursor: "pointer",
              boxShadow: `0 10px 25px -5px rgba(37, 194, 160, 0.4), 0 8px 10px -6px rgba(0, 0, 0, 0.1)`,
              fontSize: "14px",
              fontWeight: 600,
              whiteSpace: "nowrap",
              position: "relative",
            }}
          >
            <motion.div
              animate={{ rotate: [0, 10, -10, 0] }}
              transition={{ repeat: Infinity, duration: 2, ease: "easeInOut" }}
            >
              <RiRobot2Line size={18} />
            </motion.div>
            
            <span>Ask AI</span>
            
            <RiSparkling2Fill 
              size={12} 
              style={{ opacity: 0.8, marginLeft: -2 }} 
            />

            {/* Pointer Arrow */}
            <div
              style={{
                position: "absolute",
                bottom: "-6px",
                left: "50%",
                transform: "translateX(-50%) rotate(45deg)",
                width: "12px",
                height: "12px",
                background: "#1e9e83",
                borderRight: "1px solid rgba(255, 255, 255, 0.1)",
                borderBottom: "1px solid rgba(255, 255, 255, 0.1)",
                zIndex: -1,
              }}
            />
          </motion.button>
        </motion.div>
      )},
    </AnimatePresence>,
    document.body
  );
}