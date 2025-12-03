#!/usr/bin/env python3
import json
import argparse
from pathlib import Path

class DiagramGenerator:
    def __init__(self, root_dir):
        self.root = Path(root_dir)
        self.templates_dir = self.root / ".claude" / "skills" / "diagram-architect" / "templates"
        self.ref_dir = self.root / ".claude" / "skills" / "diagram-architect" / "reference"
        
        # Load Theme
        with open(self.ref_dir / "style_theme.json", 'r') as f:
            self.theme = json.load(f)

    def generate_ros_graph(self, nodes, topics, connections):
        """Generates a ROS 2 Node/Topic graph"""
        
        # Start with the styles from the theme
        chart = "graph LR;\n"
        chart += f"    classDef node {self.theme['ros_node']};\n"
        chart += f"    classDef topic {self.theme['ros_topic']};\n"
        chart += f"    classDef service {self.theme['ros_service']};\n\n"
        
        # Define Nodes
        for n in nodes:
            clean_name = n.replace(" ", "_")
            chart += f"    {clean_name}(({n})):::node\n"
            
        # Define Topics
        for t in topics:
            clean_topic = t.replace("/", "").replace(" ", "_")
            chart += f"    {clean_topic}[{t}]:::topic\n"
            
        chart += "\n    %% Connections\n"
        
        # Simple parsing of connections "Node->Topic"
        for conn in connections:
            if "->" in conn:
                src, dst = conn.split("->")
                src = src.strip().replace(" ", "_").replace("/", "")
                dst = dst.strip().replace(" ", "_").replace("/", "")
                chart += f"    {src} --> {dst}\n"
                
        return self._wrap_docusaurus(chart)

    def generate_hardware_graph(self, devices, connections):
        """Generates a Hardware Wiring graph"""
        
        chart = "flowchart TD;\n"
        chart += f"    classDef device {self.theme['hw_device']};\n"
        chart += f"    classDef sensor {self.theme['hw_sensor']};\n"
        chart += f"    classDef power {self.theme['hw_power']};\n\n"
        
        # Add devices (simple logic to guess type based on name)
        for d in devices:
            d_id = d.replace(" ", "_")
            style = "device"
            if "sense" in d.lower() or "lidar" in d.lower() or "cam" in d.lower():
                style = "sensor"
            elif "batt" in d.lower() or "power" in d.lower():
                style = "power"
                
            chart += f"    {d_id}[{d}]:::{style}\n"
            
        chart += "\n    %% Wiring\n"
        for conn in connections:
            if "->" in conn:
                parts = conn.split("->")
                src = parts[0].strip().replace(" ", "_")
                dst = parts[1].strip().replace(" ", "_")
                label = ""
                if len(parts) > 2:
                    label = f"|{parts[2].strip()}|"
                
                chart += f"    {src} --{label}--> {dst}\n"
                
        return self._wrap_docusaurus(chart)

    def _wrap_docusaurus(self, chart_content):
        """Wraps in the React component format"""
        return f"""<Mermaid chart={
            {chart_content}
            }/>"""

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", required=True, choices=['ros', 'hardware'])
    parser.add_argument("--items", required=True, help="Comma separated nodes or devices")
    parser.add_argument("--connections", default="", help="Format: A->B, B->C")
    
    args = parser.parse_args()
    
    # Mock root dir resolution for script execution
    # In real usage, this might be passed or calculated
    import os
    project_root = os.getcwd() 
    
    gen = DiagramGenerator(project_root)
    
    items_list = [i.strip() for i in args.items.split(",")]
    conn_list = [c.strip() for c in args.connections.split(",") if c.strip()]
    
    # Identify Topics vs Nodes roughly for CLI usage
    # (In a real app, this logic would be smarter or inputs more structured)
    nodes = []
    topics = []
    
    if args.type == 'ros':
        for i in items_list:
            if i.startswith("/"):
                topics.append(i)
            else:
                nodes.append(i)
        print(gen.generate_ros_graph(nodes, topics, conn_list))
        
    elif args.type == 'hardware':
        print(gen.generate_hardware_graph(items_list, conn_list))

if __name__ == "__main__":
    main()