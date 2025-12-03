# Project 3: Office Runner – Autonomous Item Delivery

### 🎯 Objective
The student will implement an autonomous mobile robot using ROS 2 and Nav2 to navigate a predefined map and simulate item delivery to specific locations.

### 🧠 Theory: Autonomous Navigation with Nav2

Autonomous navigation is a cornerstone of robotics, enabling robots to move from one point to another without constant human intervention. In ROS 2, the Nav2 stack provides a comprehensive set of tools for this purpose.

*   **ROS 2 Actions for Long-Running Tasks**: Unlike topics (continuous data streams) or services (single request-response), ROS 2 Actions are designed for goal-oriented, long-duration tasks. For autonomous navigation, the `NavigateToPose` action is crucial. You send a target pose (x, y, yaw) as a goal, receive continuous feedback on the robot's progress, and eventually a result indicating success or failure. This robust communication pattern is essential when a physical robot might be navigating for minutes and needs to report its status periodically.
*   **Nav2 Stack Components**: Nav2 is a modular system, with each component specializing in a part of the navigation challenge:
    *   **`map_server`**: Loads and provides the robot with a static map of the environment.
    *   **`amcl` (Adaptive Monte Carlo Localization)**: This node is responsible for *localization*—determining the robot's current pose within a known map. It uses a particle filter, processing sensor data (like lidar scans) and odometry estimates to track the robot's position. On a real robot, AMCL combats odometry drift, which is inevitable due to wheel slip and sensor noise.
    *   **`bt_navigator` (Behavior Tree Navigator)**: The brain of Nav2, orchestrating high-level behaviors defined in a Behavior Tree. When a `NavigateToPose` action goal is received, the `bt_navigator` executes a pre-defined sequence of actions, such as planning a global path, following it locally, and handling recovery behaviors.
    *   **`planner_server`**: Generates a global path from the robot's current location to the goal.
    *   **`controller_server`**: Executes the global path by generating velocity commands (`/cmd_vel`) for the robot's motors, while avoiding local obstacles.
    *   **`smoother_server`**: Refines paths for smoother execution.
*   **Behavior Trees (BTs)**: Behavior Trees provide a hierarchical, state-machine-like structure to define complex robot behaviors. For "Office Runner," a BT will define the sequence of `NavigateToPose` actions required to deliver items, potentially incorporating `Wait` actions or recovery strategies. They are critical for managing the robot's responses to its environment and mission goals in a structured, fault-tolerant manner.

### 🛠️ Architecture

The "Office Runner" system involves a Python node (`delivery_manager.py`) sending navigation goals to the Nav2 stack, which then controls the robot. The localization and navigation components work in concert to achieve autonomous movement.

import ThreeDiagram from '@site/src/components/ThreeDiagram';

<ThreeDiagram id="P3.1" />

**TF (Transform) Tree**:
*   `map` (world frame, provided by `map_server`)
*   `odom` (odometry frame, provided by robot's odometry source, e.g., wheel encoders)
*   `base_link` (robot's origin, typically center of its base)
*   `laser_frame` (LiDAR sensor frame, attached to `base_link`)

The `amcl` node computes the transform from `map` to `odom`, aligning the robot's local odometry with the global map. The `robot_state_publisher` ensures `odom` to `base_link` and `base_link` to sensor frames are correctly published.

### 💻 Implementation

The implementation focuses on setting up the necessary ROS 2 infrastructure and a Python node to interact with Nav2.

**1. Create ROS 2 Package**

First, create a new ROS 2 Python package called `office_runner`:

```bash
ros2 pkg create --build-type ament_python office_runner
```

**2. Placeholder Map Files**

Inside your `office_runner` package, create a `maps` directory and add placeholder files:

```bash
mkdir -p office_runner/maps
touch office_runner/maps/office_map.yaml
touch office_runner/maps/office_map.pgm
```

You would later replace these with actual map files generated from SLAM.

**3. Launch Files**

Create `launch` directory inside `office_runner` and add the following launch files.

**`/home/abdullahiqbal/Abdullah/hackathon-book-project/office_runner/launch/localization.launch.py`**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the office_runner package
    office_runner_share_dir = get_package_share_directory('office_runner')
    map_file_path = os.path.join(office_runner_share_dir, 'maps', 'office_map.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True, # Set to False for real robot
                'scan_topic': 'scan',
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                # Add AMCL specific parameters here
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}] # Ensure this matches simulation if used
            # Arguments for URDF description should be added here for real robot
        ),
        # You would typically add a static_transform_publisher for map to odom if your AMCL isn't providing it directly,
        # or if you need an initial pose. Nav2 AMCL provides map->odom implicitly.
    ])
```
*   **Context**: This launch file is responsible for loading the map and localizing the robot within it.
*   **Hardware Constraint**: For a Jetson Orin Nano, these nodes (especially AMCL with a high particle count) can be CPU-intensive. The `use_sim_time` parameter is critical: `True` for simulation, `False` for real hardware.

**`/home/abdullahiqbal/Abdullah/hackathon-book-project/office_runner/launch/navigation.launch.py`**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the path to the office_runner package
    office_runner_share_dir = get_package_share_directory('office_runner')
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')

    # Placeholder for Nav2 parameters - you would have a YAML file here
    nav2_params_file = os.path.join(office_runner_share_dir, 'config', 'nav2_params.yaml')

    # Create a dummy nav2_params.yaml if it doesn't exist to prevent errors during initial setup
    config_dir = os.path.join(office_runner_share_dir, 'config')
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
    if not os.path.exists(nav2_params_file):
        with open(nav2_params_file, 'w') as f:
            f.write("use_sim_time: True\n") # Default to sim time for initial setup
            f.write("controller_server:\n")
            f.write("  ros__parameters:\n")
            f.write("    min_x_velocity_threshold: 0.001\n")
            f.write("    min_y_velocity_threshold: 0.001\n")
            f.write("    min_z_angular_velocity_threshold: 0.001\n")
            f.write("planner_server:\n")
            f.write("  ros__parameters:\n")
            f.write("    PlannerServer:\n")
            f.write("      use_sim_time: True\n")
            f.write("recovery_server:\n")
            f.write("  ros__parameters:\n")
            f.write("    recovery_behaviour_trees_config_file: " + os.path.join(nav2_bringup_share_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml') + "\n")
            f.write("bt_navigator:\n")
            f.write("  ros__parameters:\n")
            f.write("    use_sim_time: True\n")
            f.write("    default_nav_to_pose_bt_xml: " + os.path.join(office_runner_share_dir, 'behavior_trees', 'delivery_routine.xml') + "\n")
            f.write("smoother_server:\n")
            f.write("  ros__parameters:\n")
            f.write("    use_sim_time: True\n")


    # Nav2 uses a common parameter file
    configured_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={'use_sim_time': 'true'}, # This will be overridden by the individual nodes' parameters
            convert_types=True)


    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True,
                                            'default_nav_to_pose_bt_xml': os.path.join(office_runner_share_dir, 'behavior_trees', 'delivery_routine.xml')}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager_node',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True,
                         'node_names': ['controller_server',
                                        'planner_server',
                                        'smoother_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        )
    ])
```
*   **Context**: This launch file brings up the core navigation components of Nav2.
*   **Hardware Constraint**: The path planning and control algorithms run continuously, demanding consistent CPU cycles. On a Jetson Orin, optimizing these parameters (e.g., `controller_frequency`, `planner_frequency`) is crucial to meet latency requirements for motor control. Setting `use_sim_time` to `False` for real hardware is critical.

**4. Behavior Tree**

Create a `behavior_trees` directory inside `office_runner` and add a placeholder behavior tree.

**`/home/abdullahiqbal/Abdullah/hackathon-book-project/office_runner/behavior_trees/delivery_routine.xml`**

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="DeliverySequence">
            <!-- Go to the first delivery point -->
            <Action ID="NavigateToPose" name="GoToPointA"/>
            <!-- Wait for a moment to simulate item drop-off -->
            <Action ID="Wait" duration="5.0"/>
            <!-- Go to the second delivery point -->
            <Action ID="NavigateToPose" name="GoToPointB"/>
        </Sequence>
    </BehaviorTree>

    <!-- Node definitions -->
    <TreeNodesModel>
        <Action ID="NavigateToPose">
            <input_port name="pose">Target pose (geometry_msgs/PoseStamped)</input_port>
        </Action>
        <Action ID="Wait">
            <input_port name="duration">Wait duration in seconds</input_port>
        </Action>
    </TreeNodesModel>
</root>
```
*   **Context**: This XML defines a simple sequence for the robot: navigate to point A, wait, then navigate to point B. This is the `delivery_routine` that `bt_navigator` will execute.

**5. Python Node (`delivery_manager.py`)**

Create a `delivery_manager.py` file in the `office_runner/office_runner` directory (inside the Python package).

**`/home/abdullahiqbal/Abdullah/hackathon-book-project/office_runner/office_runner/delivery_manager.py`**

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# To ensure this node is discoverable by ROS 2
# Add the following to setup.py in the 'entry_points' section:
# 'console_scripts': [
#     'delivery_manager = office_runner.delivery_manager:main',
# ],

class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            )
        )
        self.get_logger().info('Delivery Manager node initialized.')

    def send_goal(self, x, y, yaw):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw # Simplified for 2D, assumes yaw in radians
        goal_msg.pose.pose.orientation.w = 1.0 # For simplicity, assumes no pitch/roll

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)
')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4: # GoalStatus.SUCCEEDED
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
        rclpy.shutdown() # Shutdown the node after goal completion or failure

def main(args=None):
    rclpy.init(args=args)
    delivery_manager = DeliveryManager()

    # Example delivery points
    # Point A
    delivery_manager.send_goal(x=1.0, y=1.0, yaw=0.0) # Move 1m forward, 1m left, facing forward

    # After the first goal completes, you could chain another goal here or in a more complex state machine
    # For simplicity, this example will just shutdown after the first goal.
    # In a real delivery routine, you would have a sequence of goals.

    rclpy.spin(delivery_manager)
    # The node will shutdown in get_result_callback
    delivery_manager.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*   **Context**: This Python node creates an `ActionClient` for `NavigateToPose`, sends a goal, and handles the responses.
*   **Hardware Constraint**: Running Python nodes on a Jetson Orin Nano incurs overhead. While `rclpy` is optimized, complex Python logic in callback functions can introduce latency. The `QoSProfile` is explicitly defined: `ReliabilityPolicy.RELIABLE` is chosen for control commands to ensure critical goals are delivered, even if it means retransmissions. `DurabilityPolicy.VOLATILE` is typical for actions as old goals are not usually relevant.
*   **Code is Law**: Uses `rclpy` class inheritance, `ActionClient`, `QoSProfile`. `setup.py` entry point is commented as a reminder.

### ⚠️ Common Pitfalls (Sim vs. Real)

*   **Simulation**:
    *   **Perfect Odometry**: In Gazebo or Isaac Sim, odometry is often perfect, leading to minimal drift and excellent AMCL performance even with sparse sensor data.
    *   **Ideal Sensors**: Sensor data (LiDAR, camera) is noise-free, and lighting conditions are consistent.
    *   **Infinite Power**: No battery constraints, thermal throttling, or voltage sag. The robot performs identically at all times.
    *   **No Unforeseen Obstacles**: Dynamic obstacles (people, dropped items) are usually not present unless explicitly simulated.
*   **Reality**:
    *   **Localization Drift**: On a physical robot, wheel slippage, bumpy surfaces, and sensor noise (e.g., LiDAR glare on reflective surfaces) cause odometry to drift. AMCL's performance becomes highly dependent on map quality, good sensor data, and careful tuning of its parameters. A robot can easily become "lost" if it moves too fast or in an environment with ambiguous features.
    *   **Latency in Control Loops**: The entire Nav2 stack, from global planning to local control, runs on the robot's onboard computer (e.g., Jetson Orin). If any node in the chain experiences high latency due to CPU/GPU load, memory pressure, or Python overhead, the robot's movements can become jerky, it might overshoot goals, or even oscillate, potentially leading to collisions.
    *   **Power and Thermal Throttling**: Battery voltage sag under heavy motor load affects motor torque and speed, leading to deviations from commanded velocities. High CPU/GPU usage can cause thermal throttling, reducing computational power and increasing latency.
    *   **Dynamic Environments**: People walking, doors opening, or unexpected objects are common. Nav2's local planners need to react quickly and safely, which requires robust sensor input and low-latency processing.
    *   **Imperfect Collision Avoidance**: While Nav2 has collision avoidance, it relies on sensor data. If a small object is below the LiDAR plane or a transparent object is invisible, collisions can occur.
*   **Fix**:
    *   **Tuning Nav2 Parameters**: Invest significant time in tuning `amcl`, `controller_server`, and `planner_server` parameters for your specific robot and environment. This includes `update_frequency`, `max_vel_x`, `min_vel_x`, `acc_lim_x`, and local planner parameters.
    *   **Robust Error Handling**: Implement more sophisticated error handling in `delivery_manager.py` to react to `NavigateToPose` failures (e.g., retry, request human intervention, switch to a recovery behavior).
    *   **Sensor Fusion**: Consider fusing multiple sensor types (LiDAR, depth camera, IMU) for more robust localization and obstacle detection.
    *   **Performance Monitoring**: Actively monitor CPU, memory, and topic latencies (`ros2 topic hz`, `ros2 topic bw`) on the edge device to identify bottlenecks.
    *   **Slow Down**: Often, simply reducing the robot's maximum velocities in the controller parameters (`max_vel_x`, `max_rot_vel`) can significantly improve navigation reliability in reality.

:::danger
**Real-World Collision Risk**: Autonomous navigation on a physical robot carries significant risk. Untuned parameters, sensor failures, or unexpected objects can lead to collisions, potentially causing:
*   **Robot Damage**: Breaking sensors, motors, or structural components.
*   **Environmental Damage**: Hitting furniture, walls, or other equipment.
*   **Personal Injury**: Colliding with people, especially at higher speeds.
Always test in a controlled environment, start with very low speeds, and be ready to trigger an emergency stop (`E-stop`) at all times.
:::

### 🧪 Verification

To verify your "Office Runner" setup, follow these steps:

1.  **Build the ROS 2 Package**:
    ```bash
    cd /home/abdullahiqbal/Abdullah/hackathon-book-project
    colcon build --packages-select office_runner
    source install/setup.bash
    ```
2.  **Launch Localization**: Open a terminal and run your localization stack (you'll need a simulated robot running in Gazebo/Isaac Sim, or your real robot's drivers).
    ```bash
    ros2 launch office_runner localization.launch.py
    ```
3.  **Launch Navigation**: Open another terminal.
    ```bash
    ros2 launch office_runner navigation.launch.py
    ```
4.  **Set Initial Pose (if using AMCL)**: If using AMCL with a static map, you'll need to provide an initial pose estimate in `rviz2`. Open `rviz2`, add a `Map` display, a `LaserScan` display, and an `AmclPose` display. Use the "2D Pose Estimate" tool in `rviz2` to tell AMCL where the robot approximately is on the map.
    ```bash
    rviz2
    ```
5.  **Run the Delivery Manager**: Open a third terminal.
    ```bash
    ros2 run office_runner delivery_manager
    ```
    You should see the `delivery_manager` waiting for the `navigate_to_pose` action server, then sending a goal.
6.  **Observe Robot Movement**: The simulated (or real) robot should start moving towards the target pose (1.0, 1.0, 0.0) on the map.
7.  **Monitor Topics**:
    *   Check the localization quality:
        ```bash
        ros2 topic echo /amcl_pose
        ```
    *   Observe the velocity commands being sent to the robot:
        ```bash
        ros2 topic echo /cmd_vel
        ```
    *   Check for navigation status and feedback:
        ```bash
        ros2 topic list | grep navigate_to_pose
        ```
        Then, echo the relevant feedback or status topics.

**Expected Outcomes**:
*   The robot will autonomously navigate from its starting position to the target pose (1.0, 1.0, 0.0) defined in `delivery_manager.py`.
*   You should observe the robot's pose in `rviz2` updating correctly relative to the loaded map.
*   The `delivery_manager` node will report "Goal accepted :)" and then "Goal succeeded!" (or "Goal rejected/failed") once the navigation task is complete.
*   The `ros2 topic echo /cmd_vel` output will show varying linear and angular velocities as Nav2 guides the robot.
*   The `ros2 topic echo /amcl_pose` output will show `map` frame poses, indicating successful localization within the environment.
