# Project 1: Sentient Sentry – Camera-Based Face Tracking

### 🎯 Objective
Students will implement a ROS 2-based system that uses a simulated camera to detect faces and controls a simulated robot (or a pan-tilt unit) to keep the detected face centered in the camera's view. This project introduces core ROS 2 concepts, basic computer vision with OpenCV, and fundamental PID control.

### 🧠 Theory: Vision-Based Feedback Control
The Sentient Sentry project is a practical application of a vision-based feedback control loop. In essence, the robot acts as a closed-loop system:

1.  **Perception (VisionNode)**: The robot's "eyes" (a simulated camera) capture an image. This image is processed to detect faces and calculate their position relative to the image center.
2.  **Cognition (Error Calculation)**: The `VisionNode` then quantifies the "error" – how far off-center the detected face is. This error is the input to the control system.
3.  **Action (ControlNode)**: The robot's "brain" (`ControlNode`) uses this error to generate a control command (angular velocity). A PID (Proportional-Integral-Derivative) controller is employed to calculate this command, aiming to minimize the error and bring the face back to the center.
4.  **Physical Response**: The simulated robot (or pan-tilt unit) executes the angular velocity command, causing it to turn.

This loop repeats continuously, allowing the robot to "track" the face. The **Quality of Service (QoS)** of ROS 2 topics is crucial here: `Best Effort` for camera images prioritizes speed, while `Reliable` for control commands ensures critical instructions are not lost.

### 🛠️ Architecture

The system consists of two primary ROS 2 Python nodes: `vision_node` and `control_node`, communicating via a custom `/sentry/target_error` topic.

```mermaid
graph LR
    A[SimulatedCameraNode] -->|/image_raw (sensor_msgs/Image)| B(VisionNode)
    B -->|/sentry/target_error (std_msgs/Float32)| C(ControlNode)
    C -->|/cmd_vel (geometry_msgs/Twist)| D[RobotBaseController]
    D --> E[SimulatedRobot]

    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#add8e6,stroke:#333,stroke-width:2px
    style C fill:#ccffcc,stroke:#333,stroke-width:2px
    style D fill:#ffcc99,stroke:#333,stroke-width:2px
    style E fill:#ffff99,stroke:#333,stroke-width:2px
```

**ROS 2 Graph Explanation:**
*   **`SimulatedCameraNode`**: This node (typically part of your simulation environment, e.g., a Gazebo camera plugin) publishes raw image data on the `/image_raw` topic.
*   **`VisionNode`**: Subscribes to `/image_raw`, processes images using OpenCV for face detection, calculates the horizontal offset (error) of the detected face from the image center, and publishes this error on `/sentry/target_error`.
*   **`ControlNode`**: Subscribes to `/sentry/target_error`, implements a PID control algorithm, and publishes the calculated angular velocity commands on `/cmd_vel` (a standard topic for robot base control).
*   **`RobotBaseController`**: This is the interface to your simulated robot's motors (e.g., a Gazebo differential drive controller). It receives `/cmd_vel` commands and translates them into motor actions.
*   **`SimulatedRobot`**: The visual and physical representation of your robot within the simulation environment.

### 💻 Implementation

The project is structured as a single ROS 2 Python package named `sentient_sentry`.

**File Structure:**

```bash
/home/abdullahiqbal/Abdullah/hackathon-book-project/src/sentient_sentry
├── sentient_sentry
│   ├── __init__.py
│   ├── vision_node.py
│   └── control_node.py
├── config
│   └── params.yaml
├── launch
│   └── sentient_sentry.launch.py
├── package.xml
└── setup.py
```

#### 1. Create ROS 2 Python Package

To create the ROS 2 Python package, open your terminal and navigate to your ROS 2 workspace `src` directory. Then, run the following command:

```bash
ros2 pkg create sentient_sentry --build-type ament_python --dependencies rclpy cv_bridge sensor_msgs geometry_msgs std_msgs
```

This command creates a new Python package named `sentient_sentry` with the necessary build type and declares its dependencies on `rclpy` (ROS Client Library for Python), `cv_bridge` (for OpenCV-ROS image conversion), `sensor_msgs` (for image messages), `geometry_msgs` (for Twist velocity messages), and `std_msgs` (for basic data types like Float32).

#### 2. Configuration File (`params.yaml`)

Create a `config` directory inside your `sentient_sentry` package and add `params.yaml`. This file will store our PID gains and other node-specific parameters, allowing for easy tuning without recompiling code.

**Context**: `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/sentient_sentry/config/params.yaml`

```yaml
# PID Controller Gains
# These values are critical for robot stability and responsiveness.
# Improper tuning can lead to oscillations or sluggish behavior.
pid:
  kp: 0.5  # Proportional gain: Determines the response to the current error.
  ki: 0.0  # Integral gain: Addresses accumulated error over time (helps eliminate steady-state error).
  kd: 0.0  # Derivative gain: Damps oscillations by considering the rate of change of error.

# Vision Node Parameters
vision:
  camera_topic: "/image_raw" # The topic from which the camera images are received.
  # Path to the Haar Cascade XML file for face detection.
  # You might need to adjust this path based on your OpenCV installation.
  # For simulation, ensure this file is accessible within the ROS environment.
  face_cascade_path: "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
  image_width: 640 # Expected width of the camera image. Used for centering calculations.
  image_height: 480 # Expected height of the camera image.
```

#### 3. Vision Node (`vision_node.py`)

This node subscribes to the camera's raw image topic, uses OpenCV to detect faces, and publishes the horizontal error of the detected face from the image center.

**Context**: `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/sentient_sentry/sentient_sentry/vision_node.py`

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Declare parameters from params.yaml
        self.declare_parameter('vision.camera_topic', '/image_raw')
        self.declare_parameter('vision.face_cascade_path', '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml')
        self.declare_parameter('vision.image_width', 640)
        self.declare_parameter('vision.image_height', 480)

        self.camera_topic = self.get_parameter('vision.camera_topic').get_parameter_value().string_value
        self.face_cascade_path = self.get_parameter('vision.face_cascade_path').get_parameter_value().string_value
        self.image_width = self.get_parameter('vision.image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('vision.image_height').get_parameter_value().integer_value

        # Initialize CvBridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Load the Haar Cascade for face detection
        # This path might need to be adjusted based on your OpenCV installation
        self.face_cascade = cv2.CascadeClassifier(self.face_cascade_path)
        if not self.face_cascade.empty():
            self.get_logger().info(f"Loaded face cascade from: {self.face_cascade_path}")
        else:
            self.get_logger().error(f"Failed to load face cascade from: {self.face_cascade_path}")

        # QoS profile for camera image subscription
        # Best Effort reliability is chosen to prioritize low latency over guaranteed delivery,
        # which is typical for streaming sensor data where older data quickly becomes irrelevant.
        image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Keep only the most recent message
            durability=DurabilityPolicy.VOLATILE # Not relevant for transient data
        )

        # Create a subscription to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            image_qos_profile # Apply the defined QoS profile
        )
        self.get_logger().info(f"Subscribing to {self.camera_topic} with Best Effort QoS")

        # QoS profile for error publisher
        # Reliable reliability ensures that all error messages are delivered to the ControlNode,
        # which is crucial for stable feedback control.
        error_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create a publisher for the target error
        self.publisher = self.create_publisher(
            Float32,
            '/sentry/target_error',
            error_qos_profile # Apply the defined QoS profile
        )
        self.get_logger().info("Publishing to /sentry/target_error with Reliable QoS")

        self.get_logger().info('Vision Node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (bgr8 format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert to grayscale for face detection (improves performance)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # Equalize histogram for better contrast (can help with varying lighting conditions)
            gray = cv2.equalizeHist(gray)

            # Detect faces in the grayscale image
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1, # How much the image size is reduced at each image scale
                minNeighbors=5,  # How many neighbors each candidate rectangle should have to retain it
                minSize=(30, 30) # Minimum possible object size. Objects smaller than that are ignored.
            )

            target_error_msg = Float32()
            target_error_msg.data = 0.0 # Default to no error if no face is found

            if len(faces) > 0:
                # Assuming we want to track the largest face or the first one found
                # For simplicity, we take the first detected face
                x, y, w, h = faces[0]

                # Calculate the center of the detected face
                face_center_x = x + w / 2

                # Calculate the image center
                image_center_x = self.image_width / 2

                # Calculate the horizontal error: difference between face center and image center
                # Positive error means face is to the right of center, negative to the left.
                target_error_msg.data = float(face_center_x - image_center_x)

                # Optional: Draw a rectangle around the detected face for visualization
                # For real-world applications, this might be done on a separate visualization stream
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            else:
                self.get_logger().debug("No face detected.")

            # Publish the calculated error
            self.publisher.publish(target_error_msg)

            # Optional: Display the image with detected faces for debugging
            # This is typically avoided on embedded systems for performance reasons
            # cv2.imshow("Face Tracking", cv_image)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

#### 4. Control Node (`control_node.py`)

This node subscribes to the target error, implements a PID controller to generate angular velocity commands, and publishes these commands to the robot.

**Context**: `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/sentient_sentry/sentient_sentry/control_node.py`

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Declare PID parameters from params.yaml
        self.declare_parameter('pid.kp', 0.5)
        self.declare_parameter('pid.ki', 0.0)
        self.declare_parameter('pid.kd', 0.0)

        self.kp = self.get_parameter('pid.kp').get_parameter_value().double_value
        self.ki = self.get_parameter('pid.ki').get_parameter_value().double_value
        self.kd = self.get_parameter('pid.kd').get_parameter_value().double_value

        # PID control variables
        self.last_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

        # QoS profile for error subscription and cmd_vel publisher
        # Reliable reliability is chosen for both to ensure precise control.
        # It guarantees that no command or error signal is lost, which is critical for
        # maintaining stable robot behavior.
        control_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create a subscription to the target error topic
        self.subscription = self.create_subscription(
            Float32,
            '/sentry/target_error',
            self.error_callback,
            control_qos_profile # Apply the defined QoS profile
        )
        self.get_logger().info("Subscribing to /sentry/target_error with Reliable QoS")

        # Create a publisher for robot velocity commands
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            control_qos_profile # Apply the defined QoS profile
        )
        self.get_logger().info("Publishing to /cmd_vel with Reliable QoS")

        self.get_logger().info('Control Node has been started.')

    def error_callback(self, msg):
        current_error = msg.data
        current_time = time.time()
        dt = current_time - self.last_time

        if dt == 0: # Avoid division by zero
            return

        # Proportional term
        p_term = self.kp * current_error

        # Integral term
        self.integral_error += current_error * dt
        i_term = self.ki * self.integral_error

        # Derivative term
        derivative_error = (current_error - self.last_error) / dt
        d_term = self.kd * derivative_error

        # Calculate the total control output
        # We only control angular.z (yaw) to turn the robot
        angular_velocity = p_term + i_term + d_term

        # Create and publish the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0.0  # No linear movement
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = -angular_velocity # Negative because a positive error (face right) means turning left (negative angular.z)

        self.publisher.publish(twist_msg)

        # Update for next iteration
        self.last_error = current_error
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 5. Launch File (`sentient_sentry.launch.py`)

This launch file will start both the `vision_node` and `control_node`, loading parameters from `params.yaml`.

**Context**: `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/sentient_sentry/launch/sentient_sentry.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the sentient_sentry package
    sentient_sentry_dir = get_package_share_directory('sentient_sentry')
    config_dir = os.path.join(sentient_sentry_dir, 'config')
    params_file = os.path.join(config_dir, 'params.yaml')

    return LaunchDescription([
        Node(
            package='sentient_sentry',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[params_file], # Load parameters from params.yaml
            emulate_tty=True # Required for proper output buffering in Docker/Kubernetes environments
        ),
        Node(
            package='sentient_sentry',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[params_file], # Load parameters from params.yaml
            emulate_tty=True
        ),
    ])
```

#### 6. Update `setup.py`

To make your Python nodes executable, you need to add entry points in your `setup.py` file within the `sentient_sentry` package.

**Context**: `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/sentient_sentry/setup.py`

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sentient_sentry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files in the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yell]'))),
        # Include all config files in the 'config' directory
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # TODO: Replace with your actual name
    maintainer_email='your.email@example.com', # TODO: Replace with your actual email
    description='A ROS 2 package for camera-based face tracking and robot control.',
    license='Apache-2.0', # TODO: Choose an appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Define your executable nodes here
            'vision_node = sentient_sentry.vision_node:main',
            'control_node = sentient_sentry.control_node:main',
        ],
    },
)

```

### ⚠️ Common Pitfalls (Sim vs. Real)

*   **Simulation**:
    *   **What works**: In Isaac Sim or Gazebo, odometry is often perfect, sensor noise is minimal or perfectly modeled, and lighting conditions are stable. Face detection might work flawlessly even with basic Haar cascades on clean textures. PID tuning can be done aggressively with immediate, predictable results. The Haar Cascade XML path is usually fixed.
*   **Reality**:
    *   **What fails**:
        1.  **Sensor Noise and Latency**: Real cameras suffer from motion blur (if the robot turns too fast), varying lighting, glare, and dead pixels. The `/image_raw` topic might have higher latency or dropped frames, especially over Wi-Fi, impacting the `VisionNode`'s performance.
        2.  **Computational Constraints**: Running OpenCV face detection on an Edge Device (e.g., Jetson Orin Nano) with limited CPU/GPU and shared memory (typically 8GB) can introduce significant latency, leading to a delayed feedback loop and unstable control. Python's Global Interpreter Lock (GIL) can also be a bottleneck.
        3.  **Mechanical Backlash and Friction**: The real robot's motors and gears have friction and backlash, meaning small `cmd_vel` commands might not result in immediate motion, leading to accumulated error or oscillations with high PID gains.
        4.  **PID Tuning**: Gains that work in simulation will almost certainly cause instability (oscillations, overshooting, or sluggishness) on a real robot. The `Kd` term is particularly sensitive to sensor noise.
        5.  **Haar Cascade Path**: The `face_cascade_path` might differ on your actual robot's OS, or the file might not be present if OpenCV is installed differently.
*   **Fix**:
    1.  **Sensor Pre-processing & Robustness**: Implement image pre-processing (e.g., dynamic range compression, adaptive histogram equalization) on the `VisionNode`. Consider more robust (but computationally intensive) face detection models like MTCNN or SSD if computational resources allow, or switch to C++ for critical vision processing on embedded systems.
    2.  **Hardware Acceleration**: On Jetson platforms, leverage NVIDIA's DeepStream SDK or TensorRT to accelerate vision pipelines. If Python is a bottleneck, consider rewriting the `VisionNode` in C++ with `rclcpp`.
    3.  **Iterative PID Tuning (Ziegler-Nichols)**: Never use simulation PID gains directly. Start with very low gains and incrementally tune `Kp`, `Ki`, `Kd` on the real robot. Introduce a deadband around the center to avoid constant micro-adjustments due to sensor noise.
    4.  **Verify Cascade Path**: Explicitly check the `face_cascade_path` on the target hardware and ensure it's correct and accessible. You might need to install `opencv-haar-cascades` separately.
    5.  **Emergency Stop**: **:::danger** Always have a physical emergency stop for the robot. Untuned PID controllers or software glitches can cause unpredictable and potentially dangerous movements, leading to hardware damage or injury. Run safety checks before deployment. **:::danger**

### 🧪 Verification

After compiling your ROS 2 workspace (using `colcon build`) and sourcing your `install/setup.bash` (or `setup.zsh`), you can launch your sentient sentry system.

1.  **Launch the Nodes:**
    Open a terminal, source your ROS 2 environment, and launch the system:
    ```bash
    ros2 launch sentient_sentry sentient_sentry.launch.py
    ```
    You should see output indicating that `vision_node` and `control_node` have started.

2.  **Verify Topics:**
    Open a *new* terminal, source your ROS 2 environment, and check the active ROS 2 topics:
    ```bash
    ros2 topic list
    ```
    You should see `/image_raw`, `/sentry/target_error`, and `/cmd_vel` among the listed topics.

3.  **Monitor Target Error:**
    In a new terminal, echo the `/sentry/target_error` topic:
    ```bash
    ros2 topic echo /sentry/target_error
    ```
    Initially, if no face is in view, you should see `data: 0.0`. When a face appears (e.g., via a simulated face texture or a webcam if configured for `/image_raw` publishing), you should see non-zero values indicating the horizontal offset.

4.  **Monitor Robot Commands:**
    In another new terminal, echo the `/cmd_vel` topic:
    ```bash
    ros2 topic echo /cmd_vel
    ```
    If the robot is successfully detecting and tracking a face, you should observe `angular.z` values changing (non-zero) when a face is off-center, and returning to `0.0` when the face is centered. `linear.x` should remain `0.0` as we are only tracking rotation.

This concludes the project guide for "Sentient Sentry".
