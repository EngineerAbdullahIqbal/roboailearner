# Chapter 9: Navigation with ROS 2 Nav2

### 🎯 Objective
This chapter enables your robot to move autonomously. We will explore the ROS 2 Navigation Stack (Nav2), understanding how it combines SLAM (Simultaneous Localization and Mapping), global planning (GPS-style), and local planning (obstacle avoidance) to safely traverse the physical world.

### 🧠 Theory: The Map and the Territory
Navigation answers three questions: "Where am I?", "Where am I going?", and "How do I get there?".

#### 1. Localization (AMCL / SLAM)
*   **Problem:** Odometry (counting wheel turns) drifts over time. Tires slip.
*   **Solution:** Match lidar scans to a known map (AMCL) or build the map while exploring (SLAM).

#### 2. Global Planning
*   **Problem:** Find a path across the building.
*   **Solution:** A* or Dijkstra's algorithm on a static costmap. "Go down the hallway, turn left."

#### 3. Local Planning (Controller)
*   **Problem:** Follow the global path while avoiding a person who just stepped in front of you.
*   **Solution:** Dynamic Window Approach (DWA) or MPPI (Model Predictive Path Integral). Simulates short trajectories and picks the best one that avoids collisions and progresses along the path.

### 🛠️ Architecture: The Nav2 Stack

```mermaid
graph TD
    A[Behavior Tree (Navigator)] -->|ComputePathToPose| B[Global Planner]
    A -->|FollowPath| C[Controller (Local Planner)]
    B -->|Global Costmap| D[Static Map + Sensors]
    C -->|Local Costmap| E[Real-time Sensors]
    C -->|/cmd_vel| F[Base Controller]
    G[AMCL / SLAM] -->|/tf (map->odom)| H[TF Tree]
```

### 💻 Implementation: Simple Nav2 Action Client
Instead of using RViz buttons, we can send goals programmatically.

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = w # Simplified quaternion

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    client = NavToPoseClient()
    future = client.send_goal(2.0, 0.0, 1.0)
    rclpy.spin_until_future_complete(client, future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ⚠️ Common Pitfalls (Sim vs. Real)

*   **Simulation:**
    *   **Perfect Surfaces:** Wheels never slip. Odometry is 100% accurate.
    *   **Static World:** Obstacles don't move unless programmed to.
    *   **Perfect Lidar:** No glass walls, no black absorptive objects.
*   **Reality:**
    *   **Wheel Slip:** On polished concrete or carpet, wheels slip. The robot thinks it moved 1 meter forward, but it only moved 0.9m. Localization (AMCL) must correct this jumps (TF correction).
    *   **Dynamic Obstacles:** People, pets, and doors opening/closing require a reactive Local Planner (Controller) running at high frequency.
    *   **Invisible Obstacles:** Lidar passes through glass tables. Depth cameras struggle with sunlight. The Costmap must be tuned to "inflate" obstacles safely.
    *   **The "Kidnapped Robot":** If you pick up the robot and move it, its localization breaks. It needs to be re-initialized.

*   **Fix:**
    *   **Sensor Fusion:** Fuse IMU (gyro) with Wheel Encoders (EKF) to handle slip.
    *   **Costmap Tuning:** Adjust inflation radius and obstacle clearing parameters carefully.
    *   **Recovery Behaviors:** Enable Nav2 "Recoveries" (spin, back up, wait) to handle stuck situations.

### 🧪 Verification
1.  **Map Generation:** Run `ros2 launch slam_toolbox online_async_launch.py`. Drive the robot around. Does the map look like the room? Straight walls?
2.  **Localization:** drive the robot fast and spin. Does the laser scan stay aligned with the map? If it drifts, your Odometry/TF is bad.
3.  **Navigation:** Send a goal behind an obstacle. Does the global planner find a path around it?
4.  **Dynamic Avoidance:** Stand in the robot's path. Does it stop or re-route?

### 📝 Chapter Summary
*   **Navigation** is the art of moving from A to B safely.
*   **Localization** (Where am I?) relies on fusing noisy sensors (Odom, IMU, Lidar) to estimate position on a map.
*   **Nav2** uses a layered Costmap (Static + Dynamic obstacles) and separate Global/Local planners.
*   **Physical Reality** introduces wheel slip and sensor blindness (glass), necessitating robust recovery behaviors and sensor fusion.

### 🔚 Conclusion
Navigation transforms a robot from a remote-controlled toy into an autonomous agent. By mastering Nav2, you empower your robot to explore, map, and traverse the physical world. However, the reliability of this navigation is entirely dependent on the quality of the underlying sensor data and control loops we built in previous chapters. A robot that cannot trust its senses or its wheels is a robot that is lost.
