import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import {
  FiBook,
  FiCode,
  FiMessageCircle,
  FiTarget,
  FiZap,
  FiArrowRight,
} from 'react-icons/fi';
import styles from './index.module.css';

const chapters = [
  {
    title: 'Chapter 1: ROS 2 Fundamentals',
    description: 'Robot Operating System 2 core concepts, publisher-subscriber messaging, services and actions, ROS tools and debugging',
    link: '/docs/chapter-1-ros2-fundamentals/intro',
    icon: <FiCode />,
  },
  {
    title: 'Chapter 2: Gazebo & Digital Twins',
    description: 'Gazebo simulation environment, physics simulation and dynamics, creating digital twins, sensor simulation',
    link: '/docs/chapter-2-gazebo-modeling/intro',
    icon: <FiZap />,
  },
  {
    title: 'Chapter 3: Autonomous Navigation & SLAM',
    description: 'Simultaneous Localization and Mapping (SLAM), path planning, Nav2 stack, multi-sensor fusion',
    link: '/docs/chapter-3-autonomous-navigation/intro',
    icon: <FiTarget />,
  },
  {
    title: 'Chapter 4: Vision-Language-Action Models',
    description: 'Voice input with Whisper, vision systems (YOLO-v8), LLM-driven planning, manipulation and grasping',
    link: '/docs/chapter-4-ai-integration/intro',
    icon: <FiMessageCircle />,
  },
];

const features = [
  {
    title: 'Interactive RAG Chatbot',
    description: 'Ask questions about any content in the textbook and get instant, context-aware answers powered by AI',
    icon: <FiMessageCircle />,
  },
  {
    title: 'Personalized Lessons',
    description: 'Adaptive learning paths that adjust to your progress and learning style for optimal understanding',
    icon: <FiBook />,
  },
  {
    title: 'Text Selection',
    description: 'Highlight any text and ask the AI assistant for clarification, explanations, or deeper insights',
    icon: <FiTarget />,
  },
  {
    title: 'Hands-On Examples',
    description: 'Practical code examples and exercises in each lesson to reinforce learning through practice',
    icon: <FiCode />,
  },
];

const technologies = [
  'ROS 2 Humble/Iron',
  'Gazebo',
  'NVIDIA Isaac Sim',
  'Python/C++',
  'Vision-Language Models',
];

export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description={siteConfig.tagline}
    >
      <main className={styles.homePage}>
        {/* Hero Section */}
        <section className={styles.hero}>
          <div className={styles.heroContent}>
            <h1 className={styles.heroTitle}>
              Physical AI & Humanoid Robotics Textbook
            </h1>
            <p className={styles.heroTagline}>
              {siteConfig.tagline}
            </p>
            <div className={styles.heroButtons}>
              <Link
                className={styles.buttonPrimary}
                to="/docs/chapter-1-ros2-fundamentals/intro"
              >
                Start Reading
                <FiArrowRight className={styles.buttonIcon} />
              </Link>
              <Link
                className={styles.buttonSecondary}
                to="/docs/intro"
              >
                Explore Chapters
                <FiBook className={styles.buttonIcon} />
              </Link>
            </div>
          </div>
        </section>

        {/* Chapters Overview */}
        <section className={styles.chaptersSection}>
          <div className={styles.container}>
            <h2 className={styles.sectionTitle}>What You'll Learn</h2>
            <p className={styles.sectionDescription}>
              A comprehensive journey through embodied intelligence, from ROS 2 fundamentals to advanced vision-language-action models
            </p>
            <div className={styles.chaptersGrid}>
              {chapters.map((chapter, index) => (
                <Link
                  key={index}
                  to={chapter.link}
                  className={styles.chapterCard}
                >
                  <div className={styles.chapterIcon}>{chapter.icon}</div>
                  <h3 className={styles.chapterTitle}>{chapter.title}</h3>
                  <p className={styles.chapterDescription}>{chapter.description}</p>
                  <span className={styles.chapterLink}>
                    Learn more <FiArrowRight />
                  </span>
                </Link>
              ))}
            </div>
          </div>
        </section>

        {/* Features Section */}
        <section className={styles.featuresSection}>
          <div className={styles.container}>
            <h2 className={styles.sectionTitle}>Interactive Learning Features</h2>
            <p className={styles.sectionDescription}>
              Enhance your learning experience with AI-powered tools and personalized content
            </p>
            <div className={styles.featuresGrid}>
              {features.map((feature, index) => (
                <div key={index} className={styles.featureCard}>
                  <div className={styles.featureIcon}>{feature.icon}</div>
                  <h3 className={styles.featureTitle}>{feature.title}</h3>
                  <p className={styles.featureDescription}>{feature.description}</p>
                </div>
              ))}
            </div>
          </div>
        </section>

        {/* Technologies Section */}
        <section className={styles.technologiesSection}>
          <div className={styles.container}>
            <h2 className={styles.sectionTitle}>Technologies Used</h2>
            <div className={styles.technologiesList}>
              {technologies.map((tech, index) => (
                <span key={index} className={styles.techBadge}>
                  {tech}
                </span>
              ))}
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className={styles.ctaSection}>
          <div className={styles.container}>
            <h2 className={styles.ctaTitle}>Ready to Start Learning?</h2>
            <p className={styles.ctaDescription}>
              Begin your journey into Physical AI and Humanoid Robotics today
            </p>
            <Link
              className={styles.buttonPrimary}
              to="/docs/chapter-1-ros2-fundamentals/intro"
            >
              Start Reading
              <FiArrowRight className={styles.buttonIcon} />
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}

