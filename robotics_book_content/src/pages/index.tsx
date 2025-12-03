import React from 'react';
import Layout from '@theme/Layout';
import CourseOverview from '../components/CourseOverview/CourseOverview';
import FutureOfRoboticsAI from '../components/FutureOfRoboticsAI/FutureOfRoboticsAI';
import BenefitsSection from '../components/BenefitsSection/BenefitsSection';
import Footer from '../components/Footer/Footer';
import { SplineSceneBasic } from '../../components/ui/demo';

function Home() {
  const currentYear = new Date().getFullYear();

  const courseModules = [
    { 
      title: 'ROS 2', 
      description: 'Robotic Nervous System', 
      longDescription: 'Master the Robot Operating System 2 to build robust and scalable robotic applications. Learn about communication, tools, and best practices for modern robotics development.',
      icon: '/hackathon-book-project/img/icon-ros2.svg' 
    },
    { 
      title: 'Gazebo & Unity', 
      description: 'Digital Twins', 
      longDescription: 'Explore virtual environments for robotic simulation with Gazebo and Unity. Develop and test complex robotic behaviors in a safe and reproducible setting before deploying to hardware.',
      icon: '/hackathon-book-project/img/icon-gazebo.svg' 
    },
    { 
      title: 'NVIDIA Isaac', 
      description: 'AI Perception & Control', 
      longDescription: 'Delve into NVIDIA Isaac Sim for advanced AI-powered perception and control in robotics. Understand how to integrate machine learning models for intelligent decision-making and task execution.',
      icon: '/hackathon-book-project/img/icon-isaac.svg' 
    },
    { 
      title: 'VLA Systems', 
      description: 'Integrated AI Systems', 
      longDescription: 'Understand the architecture and implementation of Vision-Language-Action (VLA) systems. Learn to build robots that can perceive, reason, and act based on visual and linguistic commands.',
      icon: '/hackathon-book-project/img/icon-vla.svg' 
    },
    { 
      title: 'Humanoid Robotics', 
      description: 'Advanced Embodied Systems', 
      longDescription: 'Dive into the complexities of humanoid robotics, covering topics from locomotion and balance to manipulation and human-robot interaction. Build the next generation of intelligent, human-like machines.',
      icon: '/hackathon-book-project/img/icon-humanoid.svg' 
    },
  ];

  const futureOfRoboticsAIData = {
    title: "The Future of Robotics AI",
    description: "Dive deep into the evolving landscape of intelligent machines, autonomous systems, and their profound impact on industries and daily life. Explore breakthroughs in machine learning, perception, and decision-making that power the next generation of humanoid robots, blurring the lines between science fiction and reality.",
  };

  const benefitsData = [
    { title: "Hands-on Mastery", description: "Gain practical expertise with cutting-edge tools and platforms, building real-world robotics solutions." },
    { title: "Bridge the Sim-to-Real Gap", description: "Learn strategies to effectively translate simulated designs to functional physical robots, overcoming real-world complexities." },
    { title: "Develop Robust Control", description: "Engineer resilient and safe control systems for complex humanoid and autonomous robotic applications." },
    { title: "Career Advancement", description: "Equip yourself with the skills demanded by the rapidly expanding physical AI and humanoid robotics industry." },
  ];

  const footerLinks = [
    { label: 'Curriculum', href: '/docs/course-outline' },
    { label: 'Blog', href: '/blog' },
    { label: 'GitHub', href: 'https://github.com/EngineerAbdullahIqbal/hackathon-book-project' },
  ];

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A Complete Digital Textbook for Embodied Intelligence & Humanoid Systems"
      wrapperClassName="homepage-wrapper">

      <SplineSceneBasic
        title="Learn About Physical AI & Robotics"
        subtitle="Explore the cutting-edge fusion of AI and robotics, from fundamental concepts to advanced humanoid systems."
        ctaLabel="Start Learning"
        ctaLink="/docs/course-outline"
      />
      <main>
        <CourseOverview modules={courseModules} />
        <FutureOfRoboticsAI {...futureOfRoboticsAIData} />
        <BenefitsSection benefits={benefitsData} />
      </main>
      <Footer
        branding="Panaversity"
        links={footerLinks}
        authors="Engineer Abdullah Iqbal"
        copyright={`Copyright © ${currentYear} Panaversity. All rights reserved.`}
      />
    </Layout>
  );
}

export default Home;