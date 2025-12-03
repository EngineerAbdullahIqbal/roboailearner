# Chapter 7: Inverse Kinematics and Humanoid Motion Planning

### 🎯 Objective
This chapter dives into the geometry of movement. You will learn the difference between Forward Kinematics (FK) and Inverse Kinematics (IK), understand how to calculate joint angles to reach a specific point in space, and explore how motion planning frameworks like MoveIt 2 navigate complex humanoid bodies without collisions.

### 🧠 Theory: The Mathematics of Reaching
To a computer, a robot is a tree of coordinate frames connected by joints. Moving a hand to a coffee cup involves solving a complex geometric puzzle.

#### 1. Forward Kinematics (FK)
*   **Question:** "If I set my shoulder to 45° and elbow to 90°, where is my hand?"
*   **Mechanism:** Chain of transformation matrices.
*   **Difficulty:** Easy. There is always exactly one solution.

#### 2. Inverse Kinematics (IK)
*   **Question:** "I want my hand at (x, y, z) with orientation (roll, pitch, yaw). What joint angles do I need?"
*   **Mechanism:** Solving non-linear equations (analytical or numerical).
*   **Difficulty:** Hard.
    *   **Multiple Solutions:** You can touch your nose with your elbow up or down.
    *   **No Solution:** The target is out of reach.
    *   **Singularities:** Configurations where the robot loses a degree of freedom (e.g., arm fully extended), causing mathematical instability and potentially dangerous infinite velocities.

#### 3. Motion Planning
Finding a path from Pose A to Pose B while avoiding obstacles (including the robot's own body). Algorithms like RRT* (Rapidly-exploring Random Tree) sample the configuration space to find a collision-free trajectory.

### 🛠️ Architecture: MoveIt 2
ROS 2 uses **MoveIt 2** as the standard motion planning framework.

```mermaid
graph TD
    A[User Goal / VLA Agent] -->|MoveGroup Interface| B(MoveIt 2 Planner)
    B -->|Inverse Kinematics| C[IK Solver (KDL/Trac-IK)]
    B -->|Collision Checking| D[Planning Scene]
    B -->|Trajectory| E[Trajectory Execution]
    E -->|/joint_trajectory| F[Robot Controller]
```

### 💻 Implementation: Basic FK/IK with Python
While MoveIt handles heavy lifting, understanding the math is crucial. We'll use a simple library to demonstrate.

```python
import numpy as np
# Conceptual IK solver for a 2-link planar arm
# Link lengths
L1 = 1.0
L2 = 1.0

def inverse_kinematics(x, y):
    """
    Calculate joint angles (theta1, theta2) to reach (x, y).
    Using Law of Cosines.
    """
    dist = np.sqrt(x**2 + y**2)
    
    # Check reachability
    if dist > (L1 + L2):
        print("Target out of reach!")
        return None, None

    # Law of Cosines for elbow angle (theta2)
    # c2 = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2) # Elbow down solution

    # Angle to target
    phi = np.arctan2(y, x)
    # Angle offset for shoulder
    # sin_theta2 / dist = sin_alpha / l2  ?? No, use Law of Cosines again or trig
    # k1 = L1 + L2 * cos_theta2
    # k2 = L2 * sin_theta2
    # theta1 = phi - arctan2(k2, k1)
    
    # Simplified geometric approach for theta1
    beta = np.arccos((L1**2 + dist**2 - L2**2) / (2 * L1 * dist))
    theta1 = phi - beta

    return np.degrees(theta1), np.degrees(theta2)

# Test
target_x, target_y = 1.414, 1.414 # Should be roughly 45 deg reach
t1, t2 = inverse_kinematics(target_x, target_y)
print(f"To reach ({target_x}, {target_y}): Shoulder={t1:.2f}°, Elbow={t2:.2f}°")
```

### ⚠️ Common Pitfalls (Sim vs. Real)

*   **Simulation:**
    *   **Ideal Geometry:** The robot model (URDF) matches the simulation perfectly.
    *   **No Sag:** Joints are perfectly rigid.
    *   **Perfect Execution:** If the planner says "move to X," the robot moves to X.
*   **Reality:**
    *   **Calibration Errors:** Real link lengths vary slightly due to manufacturing. The URDF might not match the physical robot, leading to IK errors (the hand isn't where the code thinks it is).
    *   **Backlash & Sag:** Gearboxes have "play" (backlash). Heavy arms sag under gravity. A "perfect" IK solution might result in the end-effector being 2cm lower than expected.
    *   **Singularities:** Near singularities, motors might try to move at infinite speed. In sim, this might just look glitchy. In reality, this triggers over-current protection or damages hardware.
    *   **Self-Collision:** Cables and hoses aren't usually modeled in the collision scene. A valid plan might shear a cable in reality.

*   **Fix:**
    *   **Kinematic Calibration:** Use tools to refine the URDF based on physical measurements.
    *   **Singularity Avoidance:** Use "Manipulability" metrics in IK solvers to avoid singular configurations.
    *   **Safety Margins:** Pad collision volumes (make the robot look "fatter" to the planner) to account for cables and calibration errors.

### 🧪 Verification
1.  **Visual Check (RViz):** Use the "MotionPlanning" plugin in RViz. Drag the end-effector to a target. If the "ghost" robot follows, IK is working.
2.  **Plan & Execute:** Click "Plan". Watch the visualized path. Ensure it doesn't pass through the robot body.
3.  **Tolerance Check:** On the real robot, measure the actual end-effector position vs. the requested position. The difference is your "Kinematic Error".

### 📝 Chapter Summary
*   **Kinematics** relates joint angles to end-effector position. **Forward (FK)** is simple (angles -> position); **Inverse (IK)** is complex (position -> angles).
*   **Motion Planning** algorithms like RRT* find collision-free paths in high-dimensional spaces.
*   **MoveIt 2** is the standard ROS 2 framework for managing kinematics, planning, and collision checking.
*   **Real-world challenges** include mechanical backlash, calibration discrepancies (URDF vs. Hardware), and unmodeled obstacles like cables.

### 🔚 Conclusion
Mastering Inverse Kinematics and Motion Planning is the first step in giving a humanoid robot agency. Without it, a robot is just a statue. However, the mathematical precision of an IK solver often crashes against the messy reality of mechanical imperfections. As you move to the next chapter on Control, remember that *planning* a motion is only half the battle; *executing* it against gravity and friction is where the physics truly kicks in.