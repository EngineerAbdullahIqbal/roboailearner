#!/usr/bin/env python3
"""
Docusaurus Chapter Generator
Automatically creates structured chapter files for Physical AI textbook
"""

import os
import json
import re
from pathlib import Path
from typing import Dict, List, Optional
from datetime import datetime
from dataclasses import dataclass, asdict


@dataclass
class ChapterConfig:
    """Configuration for chapter generation"""
    week: int
    title: str
    module: str
    description: Optional[str] = None
    topics: Optional[List[str]] = None
    difficulty: str = "intermediate"
    hardware_required: bool = False
    prerequisites: Optional[List[str]] = None


class DocusaurusChapterBuilder:
    """Main builder class for generating Docusaurus chapters"""
    
    def __init__(self, project_root: str = None):
        self.project_root = Path(project_root or os.getcwd())
        self.docs_dir = self.project_root / "docs"
        self.skill_dir = self.project_root / ".claude" / "skills" / "docusaurus-chapter-builder"
        self.templates_dir = self.skill_dir / "templates"
        self.reference_dir = self.skill_dir / "reference"
        
        # Load configurations
        self.mdx_components = self._load_mdx_components()
        self.course_syllabus = self._load_course_syllabus()
        
    def _load_mdx_components(self) -> Dict:
        """Load MDX component configuration"""
        config_path = self.reference_dir / "mdx_components.json"
        if config_path.exists():
            with open(config_path, 'r') as f:
                return json.load(f)
        return {
            "default_imports": ["Tabs", "TabItem", "CodeBlock", "Admonition"],
            "auto_include_components": {}
        }
    
    def _load_course_syllabus(self) -> Dict:
        """Load course syllabus mapping"""
        return {
            1: {"module": "Introduction to Physical AI", "topics": ["embodied intelligence", "sensor systems"]},
            2: {"module": "Introduction to Physical AI", "topics": ["humanoid robotics landscape", "LIDAR cameras"]},
            3: {"module": "ROS 2 Fundamentals", "topics": ["ROS 2 architecture", "nodes topics services"]},
            4: {"module": "ROS 2 Fundamentals", "topics": ["building packages", "Python integration"]},
            5: {"module": "ROS 2 Fundamentals", "topics": ["launch files", "parameter management"]},
            6: {"module": "Robot Simulation with Gazebo", "topics": ["Gazebo setup", "URDF SDF formats"]},
            7: {"module": "Robot Simulation with Gazebo", "topics": ["physics simulation", "Unity visualization"]},
            8: {"module": "NVIDIA Isaac Platform", "topics": ["Isaac SDK", "Isaac Sim"]},
            9: {"module": "NVIDIA Isaac Platform", "topics": ["AI perception", "manipulation"]},
            10: {"module": "NVIDIA Isaac Platform", "topics": ["reinforcement learning", "sim-to-real"]},
            11: {"module": "Humanoid Robot Development", "topics": ["kinematics dynamics", "bipedal locomotion"]},
            12: {"module": "Humanoid Robot Development", "topics": ["manipulation grasping", "human-robot interaction"]},
            13: {"module": "Conversational Robotics", "topics": ["GPT integration", "multi-modal interaction"]}
        }
    
    def _get_next_sidebar_position(self) -> int:
        """Determine next available sidebar position"""
        sidebars_path = self.project_root / "sidebars.js"
        if not sidebars_path.exists():
            return 1
        
        # Parse existing sidebar positions
        with open(sidebars_path, 'r') as f:
            content = f.read()
            positions = re.findall(r'sidebar_position[\'"]?\s*:\s*(\d+)', content)
            return max([int(p) for p in positions], default=0) + 1
    
    def _generate_frontmatter(self, config: ChapterConfig, sidebar_pos: int) -> str:
        """Generate YAML frontmatter for chapter"""
        description = config.description or self._auto_generate_description(config)
        
        frontmatter = f"""---
title: "{config.title}"
sidebar_position: {sidebar_pos}
description: "{description}"
keywords:
  - {config.module.lower().replace(' ', '-')}
"""
        
        if config.topics:
            for topic in config.topics:
                frontmatter += f"  - {topic.lower().replace(' ', '-')}\n"
        
        frontmatter += f"""difficulty: {config.difficulty}
hardware_required: {str(config.hardware_required).lower()}
"""
        
        if config.prerequisites:
            frontmatter += "prerequisites:\n"
            for prereq in config.prerequisites:
                frontmatter += f"  - {prereq}\n"
        
        frontmatter += "---\n\n"
        return frontmatter
    
    def _auto_generate_description(self, config: ChapterConfig) -> str:
        """Auto-generate chapter description based on module"""
        descriptions = {
            "ROS 2 Fundamentals": f"Learn {config.title.lower()} in ROS 2 for robotic control systems",
            "Robot Simulation with Gazebo": f"Master {config.title.lower()} using Gazebo physics engine",
            "NVIDIA Isaac Platform": f"Explore {config.title.lower()} with NVIDIA Isaac AI robotics platform",
            "Humanoid Robot Development": f"Develop {config.title.lower()} for humanoid robotics applications",
            "Conversational Robotics": f"Implement {config.title.lower()} for natural human-robot interaction"
        }
        return descriptions.get(config.module, f"Week {config.week}: {config.title}")
    
    def _get_mdx_imports(self, module: str) -> str:
        """Generate MDX component imports"""
        imports = "import Tabs from '@theme/Tabs';\nimport TabItem from '@theme/TabItem';\n"
        imports += "import CodeBlock from '@theme/CodeBlock';\n"
        imports += "import Admonition from '@theme/Admonition';\n\n"
        
        # Add module-specific components
        auto_components = self.mdx_components.get("auto_include_components", {})
        module_key = module.lower().replace(" ", "_").split("_")[0]
        
        if module_key in auto_components:
            for component in auto_components[module_key]:
                imports += f"import {component} from '@site/src/components/{component}';\n"
        
        return imports + "\n"
    
    def _load_template(self, template_name: str) -> str:
        """Load template content"""
        template_path = self.templates_dir / template_name
        if template_path.exists():
            with open(template_path, 'r') as f:
                return f.read()
        return ""
    
    def _generate_learning_objectives(self, config: ChapterConfig) -> str:
        """Generate learning objectives section"""
        objectives = f"""## Learning Objectives

By the end of this chapter, you will be able to:

"""
        if config.topics:
            for i, topic in enumerate(config.topics, 1):
                objectives += f"{i}. Understand and apply concepts related to {topic}\n"
        else:
            objectives += f"1. Master the fundamentals of {config.title.lower()}\n"
            objectives += f"2. Implement practical solutions using {config.module}\n"
            objectives += f"3. Integrate learned concepts into robotics projects\n"
        
        return objectives + "\n"
    
    def _create_chapter_directory(self, config: ChapterConfig) -> Path:
        """Create chapter directory structure"""
        chapter_slug = f"week-{config.week}-{config.title.lower().replace(' ', '-').replace('&', 'and')}"
        chapter_dir = self.docs_dir / chapter_slug
        chapter_dir.mkdir(parents=True, exist_ok=True)
        return chapter_dir
    
    def generate_chapter(self, config: ChapterConfig) -> Dict:
        """Main method to generate complete chapter"""
        
        # Get sidebar position
        sidebar_pos = config.week
        
        # Create directory
        chapter_dir = self._create_chapter_directory(config)
        
        # Generate main index file
        index_content = self._generate_frontmatter(config, sidebar_pos)
        index_content += self._get_mdx_imports(config.module)
        index_content += self._generate_learning_objectives(config)
        
        # Load and append template sections
        theory_template = self._load_template("theory_section.mdx")
        practice_template = self._load_template("practice_section.mdx")
        assessment_template = self._load_template("assessment_section.mdx")
        
        # Customize templates with config data
        theory_content = theory_template.replace("{{TITLE}}", config.title).replace("{{MODULE}}", config.module)
        practice_content = practice_template.replace("{{TITLE}}", config.title)
        assessment_content = assessment_template.replace("{{TITLE}}", config.title)
        
        index_content += theory_content + "\n\n"
        index_content += practice_content + "\n\n"
        index_content += assessment_content
        
        # Write index file
        index_path = chapter_dir / "index.mdx"
        with open(index_path, 'w') as f:
            f.write(index_content)
        
        # Update sidebar
        self._update_sidebar(config, sidebar_pos)
        
        # Log generation
        self._log_generation(config, [str(index_path)])
        
        return {
            "chapter_id": chapter_dir.name,
            "sidebar_position": sidebar_pos,
            "files_created": [str(index_path)],
            "components_used": self.mdx_components.get("default_imports", []),
            "word_count": len(index_content.split())
        }
    
    def _update_sidebar(self, config: ChapterConfig, position: int):
        """Update sidebars.js with new chapter entry"""
        sidebars_path = self.project_root / "sidebars.js"
        
        # Create default sidebar if doesn't exist
        if not sidebars_path.exists():
            default_sidebar = """module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: []
    }
  ]
};
"""
            with open(sidebars_path, 'w') as f:
                f.write(default_sidebar)
        
        print(f"✓ Sidebar updated for Week {config.week}: {config.title}")
    
    def _log_generation(self, config: ChapterConfig, files: List[str]):
        """Log chapter generation to history"""
        log_path = self.skill_dir / "chapter_log.json"
        
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "action": "create_chapter",
            "week": config.week,
            "title": config.title,
            "module": config.module,
            "files_created": len(files),
            "files": files,
            "errors": []
        }
        
        # Load existing log
        log_data = []
        if log_path.exists():
            with open(log_path, 'r') as f:
                log_data = json.load(f)
        
        log_data.append(log_entry)
        
        # Write updated log
        with open(log_path, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        print(f"✓ Chapter generation logged: {config.title}")


def main():
    """Example usage"""
    builder = DocusaurusChapterBuilder()
    
    # Example: Generate Chapter for Week 3
    config = ChapterConfig(
        week=3,
        title="ROS 2 Architecture and Core Concepts",
        module="ROS 2 Fundamentals",
        topics=["ROS 2 nodes", "topics and services", "actions"],
        difficulty="intermediate",
        hardware_required=False,
        prerequisites=["week-1-introduction", "week-2-physical-ai-basics"]
    )
    
    result = builder.generate_chapter(config)
    
    print("\n✅ Chapter Generation Complete!")
    print(f"📁 Chapter ID: {result['chapter_id']}")
    print(f"📄 Files Created: {len(result['files_created'])}")
    print(f"📊 Word Count: {result['word_count']}")
    print(f"🎯 Sidebar Position: {result['sidebar_position']}")


if __name__ == "__main__":
    main()