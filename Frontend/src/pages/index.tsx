import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";
import Heading from "@theme/Heading";

import styles from "./index.module.css";
import RagSection from "../components/RagSection";
import CurriculumSection from "../components/CurriculumSection";
import TechStackSection from "../components/TechStackSection";

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Humanoid Robotics with Physical AI"
    >
      <main>
        <RagSection />
        <HomepageFeatures />
        <CurriculumSection />
        <TechStackSection />
      </main>
    </Layout>
  );
}
