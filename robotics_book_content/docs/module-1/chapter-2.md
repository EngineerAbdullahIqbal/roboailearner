---
id: chapter-2
title: "Chapter 2: ROS 2 Fundamentals: Nodes, Topics, and Services"
difficulty_level: Intermediate
hardware_required: "NVIDIA Jetson Orin Nano (optional for advanced labs), Ubuntu 22.04 with ROS 2 Humble"
---
:::translate:::en
# Chapter 2: ROS 2 Fundamentals: Nodes, Topics, and Services

### 🎯 Learning Objectives
Upon completing this chapter, students will be able to:
1.  **Understand ROS 2 Core Concepts**: Define and explain the purpose of ROS 2 nodes, topics, and services as fundamental communication primitives.
2.  **Implement ROS 2 Nodes**: Create, compile, and run basic ROS 2 Python nodes for simple functionalities.
3.  **Master Topic-Based Communication**: Implement publisher and subscriber nodes, demonstrating asynchronous data flow using ROS 2 topics.
4.  **Apply Quality of Service (QoS) Settings**: Select and configure appropriate QoS profiles (Reliability, History, Depth) for different data types (e.g., sensor data vs. control commands) in real-world robotics scenarios.
5.  **Utilize Service-Based Communication**: Develop service server and client nodes to enable synchronous request-response interactions for specific robot tasks.

In this chapter, we delve into the foundational communication mechanisms of ROS 2: Nodes, Topics, and Services. These concepts are the bedrock upon which all complex robotic applications are built, enabling a distributed and modular approach to robot control and perception. You will learn how individual computational units (Nodes) communicate data (Topics) and request actions (Services), forming the intricate "nervous system" of a robot. We will explore practical Python implementations, focusing on how Quality of Service (QoS) settings are crucial for ensuring safe, reliable, and high-performance operation on real physical hardware, bridging the gap between theoretical understanding and practical deployment. By the end, you'll have a solid grasp of how to orchestrate these components to build robust and responsive robotic systems.

### 🧠 Theory: ROS 2 Nodes
:::translate:::
In ROS 2, a **Node** is the fundamental unit of computation. It is a process that performs a specific task. A complex robot like a Unitree G1 isn\'t controlled by one giant script; it\'s controlled by dozens (or hundreds) of small, modular nodes working in parallel.
*   **Modularity**: One node might handle the camera driver, another processes images for obstacles, and a third calculates motor commands. If the camera driver crashes, the motor controller keeps running, ensuring safety.
*   **Discovery**: ROS 2 uses a distributed discovery mechanism (DDS - Data Distribution Service). Nodes automatically find each other on the network without a central master (unlike ROS 1), making the system more robust and flexible for multi-robot fleets.

#### Life Cycle of a Node
A managed node (Lifecycle Node) has specific states: `Unconfigured`, `Inactive`, `Active`, and `Finalized`. This is vital for physical hardware. You don\'t want your motor controller to be \"Active\" and sending current before the safety checks have been configured.

### 💻 Implementation: Creating a Simple Node
We\'ll create a basic Python node.

#### Node Communication and Graph Structure
ROS 2 nodes typically form a graph where nodes communicate with each other through topics, services, and actions. This graph represents the entire computational architecture of the robot. Tools like `rqt_graph` allow developers to visualize this dynamic graph, which is invaluable for debugging and understanding the data flow within a complex robotic system. Each node operates independently, reducing system coupling and increasing robustness. If one sensor driver node fails, other parts of the robot’s control system can often continue operating, perhaps with degraded performance, rather than causing a complete system shutdown. This modularity is a cornerstone of scalable and fault-tolerant robotic software.

Context: This file would typically live at /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/my_first_node.py

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        # Initialize the node with the name \'my_first_node\'
        super().__init__(\'my_first_node\')
        self.get_logger().info(\'Hello from ROS 2!\')

        # Create a timer that calls \'timer_callback\' every 1.0 seconds
        self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f\'Counter: {self.counter}\')
        self.counter += 1

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = MyFirstNode()

    # Spin the node so it can process callbacks (blocking)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```

To run this, you would typically add an entry point to your `setup.py` and use `ros2 run`.

```bash
# First, ensure your ROS 2 environment is sourced
# This step is critical for ROS 2 commands to be found
source /opt/ros/humble/setup.bash
# If you are in a colcon workspace, also source your workspace setup
# source /path/to/your/workspace/install/setup.bash

# Build your package (assuming you have a workspace setup)
colcon build --packages-select my_robot_pkg

# Source your workspace again after building to include the new executable
source install/setup.bash

# Then, run the node in a terminal
ros2 run my_robot_pkg my_first_node
```

In a **separate terminal**, you can list all active ROS 2 nodes:

```bash
ros2 node list
```

You should see `/my_first_node` in the output, confirming it\'s running.

---

### 🧠 Theory: ROS 2 Topics
:::translate:::
**Topics** are the primary mechanism for asynchronous, many-to-many communication in ROS 2. They operate on a publish-subscribe model, where nodes publish data to a named topic, and other nodes subscribe to that topic to receive the data. This decoupled communication allows for flexible and scalable robotic architectures.

Imagine a robot\'s perception system. A `CameraNode` might publish raw image data to a topic `/camera/image_raw`. An `ObjectDetectionNode` subscribes to this topic, processes the images, and publishes bounding box detections to `/perception/objects`. Finally, a `NavigationNode` subscribes to `/perception/objects` to plan its path, avoiding obstacles. Each node focuses on its specific task without needing direct knowledge of other nodes\' internal workings.

#### Quality of Service (QoS)
QoS is not just a software abstraction; it dictates how critical data moves between physical components. For embodied intelligence, QoS settings are paramount to safety and performance. When you move atoms, precision and timing are survival.

The most important QoS settings for robotics are:
*   **Reliability**:
    *   `Reliable`: Guarantees that every message will be delivered, even if it means retransmitting lost packets. **Use for control commands (e.g., `/cmd_vel`) where missing a message could cause unsafe robot behavior.**
    *   `Best Effort`: Attempts to deliver messages, but does not guarantee delivery. Lost messages are not retransmitted. **Use for high-bandwidth sensor data (e.g., `/camera/image_raw`, `/scan`) where getting the latest data quickly is more important than receiving every single frame.**
*   **Durability**:
    *   `Transient Local`: New subscribers will receive the last message published immediately upon connecting. Useful for static configuration data.
    *   `Volatile`: Only receives messages published *after* the subscription is established. Standard for most real-time data.
*   **History**:
    *   `Keep Last (N)`: Keeps the last N messages in the queue.
    *   `Keep All`: Keeps all messages up to the queue depth.
*   **Depth**: The size of the message queue. A deeper queue can buffer more messages but increases potential latency.

#### Real-world QoS implications
Consider a robot navigating a cluttered environment. Its LiDAR sensor might publish scan data at 10 Hz to the `/scan` topic. If the `reliability` is set to `Reliable` for this high-bandwidth data, and network conditions are poor, the system might spend too much time retransmitting lost packets, leading to increased latency and outdated map information. Conversely, setting the `reliability` to `Best Effort` would prioritize the delivery of the *latest* scan, allowing the navigation system to react to the current environment rather than a stale representation. However, for a motor command to `/cmd_vel`, `Reliable` is paramount. A lost stop command could lead to a collision. Similarly, `Durability` can be critical; a new subscriber to a static configuration topic might need to immediately receive the last published configuration (Transient Local) rather than waiting for the next update (Volatile). The `History` and `Depth` settings further refine this by controlling how many past messages are kept in the queue, balancing memory usage, latency, and data integrity.

### 🛠️ Architecture
Two nodes communicating via a topic.

```mermaid
graph LR
    A[CameraNode] -->|/camera/image_raw| B[ObjectDetectionNode]
```

### 💻 Implementation
Here, we create a simple publisher and subscriber pair. The `MinimalPublisher` node publishes `String` messages, and the `MinimalSubscriber` node receives them. Pay close attention to the QoS profile definition, as this is critical for real-world robotics.

Context: These files would typically live at:
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/minimal_publisher.py`
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/minimal_subscriber.py`

#### Minimal Publisher (`minimal_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String # Standard ROS 2 String message type

class MinimalPublisher(Node):
    \"\"\"
    A ROS 2 Node that publishes String messages to the \'topic\' topic.
    Demonstrates QoS settings for a publisher.
    \"\"\"
    def __init__(self):
        super().__init__(\'minimal_publisher\')

        # Define a QoS profile for sensor data (Best Effort, keep last 1)
        # This is suitable for high-frequency data where missing an occasional message is acceptable
        # but getting the latest data quickly is paramount.
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Prioritize speed over guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Only keep the latest message
            durability=DurabilityPolicy.VOLATILE # Only send to active subscribers
        )

        # Create a publisher that will publish String messages to the \'topic\' topic
        # We use the sensor_qos_profile here, assuming this might be like a simple sensor stream
        self.publisher_ = self.create_publisher(String, \'topic\', sensor_qos_profile)

        self.i = 0
        # Create a timer that calls the timer_callback method every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(f\'MinimalPublisher started with QoS: Reliability={sensor_qos_profile.reliability.name}, Depth={sensor_qos_profile.depth}\')

    def timer_callback(self):
        msg = String()
        msg.data = f\'Hello World: {self.i}\'
        self.publisher_.publish(msg)
        self.get_logger().info(f\'Publishing: \"{msg.data}\"\')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        node.get_logger().info(\'Publisher node stopped cleanly.\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# To make this node executable, add to setup.py:
# entry_points={
#     \'console_scripts\': [
#         \'minimal_publisher = my_robot_pkg.minimal_publisher:main\',
#     ],\
# },
```

#### Minimal Subscriber (`minimal_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String # Standard ROS 2 String message type

class MinimalSubscriber(Node):
    \"\"\"
    A ROS 2 Node that subscribes to String messages from the \'topic\' topic.
    Demonstrates QoS settings for a subscriber.
    \"\"\"
    def __init__(self):
        super().__init__(\'minimal_subscriber\')

        # Define a QoS profile for command data (Reliable, keep last 1)
        # This is suitable for control commands where every message MUST be received
        # and latency is still important (hence depth=1 for the latest command).
        command_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Guarantee delivery
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Only care about the latest command
            durability=DurabilityPolicy.VOLATILE # Only receive from active publishers
        )

        # Create a subscriber that will listen for String messages on the \'topic\' topic
        # We use the command_qos_profile here, imagining this subscriber might be a motor controller
        self.subscription = self.create_subscription(
            String,
            \'topic\',
            self.listener_callback,
            command_qos_profile # Use the defined QoS profile
        )
        self.subscription # prevent unused variable warning
        self.get_logger().info(f\'MinimalSubscriber started with QoS: Reliability={command_qos_profile.reliability.name}, Depth={command_qos_profile.depth}\')

    def listener_callback(self, msg):\
        self.get_logger().info(f\'I heard: \"{msg.data}\"\')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        node.get_logger().info(\'Subscriber node stopped cleanly.\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# To make this node executable, add to setup.py:
# entry_points={
#     \'console_scripts\': [
#         \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
#     ],\
# },
```

### ⚠️ Common Pitfalls (Sim vs. Real)
:::translate:::
*   **Simulation**: In simulation, network latency is negligible, and message delivery is almost always perfect. This can lead to a \"happy path\" mentality where QoS settings seem unimportant.
*   **Reality**: On a physical robot, topics are transmitted over real networks (Ethernet, Wi-Fi, serial). This introduces:
    *   **Latency**: High-frequency sensor data (e.g., a 60Hz camera feed) might experience delays if processed by many nodes or transmitted over slow links. For time-critical control loops, even a few milliseconds of latency can cause oscillations or instability in a physical robot.
    *   **Message Loss**: Wireless networks are prone to interference and packet loss. If your control commands are `Best Effort` and a message is lost, the robot might not receive a critical stop command, leading to collision or falls.
    *   **Synchronization Issues**: If a node processes data from multiple topics (e.g., camera and IMU), differing publication rates and network delays can lead to unsynchronized data, causing algorithms like SLAM to drift or fail.
    *   **Hardware Overload**: Publishing very large messages (e.g., uncompressed 4K images) at high rates can saturate the network or the edge device\'s CPU, impacting all other nodes.
*   **Fix**:
    *   **Appropriate QoS**:
        *   **Control Commands (`/cmd_vel`, `/joint_commands`)**: ALWAYS use `Reliable` reliability. Ensure the `depth` is appropriate (often `1` to only process the latest command).
        *   **High-Frequency Sensor Data (`/camera/image_raw`, `/scan`)**: Use `Best Effort` reliability and a small `depth` (e.g., `1` to `5`). Prioritize fresh data.
    *   **Bandwidth Management**: Compress large data types (e.g., `sensor_msgs/CompressedImage` for camera feeds). Reduce publication rates if not strictly necessary.
    *   **Hardware Acceleration**: Utilize hardware acceleration (e.g., NVIDIA\'s `image_transport` for GPU-accelerated image compression/decompression) on Jetson devices to offload CPU.
    *   **Time Synchronization**: Implement robust time synchronization (e.g., NTP for system clocks, `message_filters.ApproximateTimeSynchronizer` for ROS 2 messages) to handle data from multiple sensors.

### 🧪 Verification
:::translate:::
First, ensure your package is built and sourced as described in the \"ROS 2 Nodes\" verification section.

1.  **Run the Publisher Node** (in one terminal):

    ```bash
    ros2 run my_robot_pkg minimal_publisher
    ```
    You should see output indicating messages being published: `[INFO] [minimal_publisher]: Publishing: \"Hello World: 0\"`

2.  **Run the Subscriber Node** (in a separate terminal):

    ```bash
    ros2 run my_robot_pkg minimal_subscriber
    ```
    You should see output indicating messages being received: `[INFO] [minimal_subscriber]: I heard: \"Hello World: 0\"`

3.  **Inspect Topics** (in a third terminal):
    *   List active topics:
        ```bash
        ros2 topic list
        ```
        You should see `/topic` in the list.
    *   Echo messages on the topic:
        ```bash
        ros2 topic echo /topic
        ```
        This will display the `String` messages being published, confirming data flow.
    *   Check topic information (type, publishers, subscribers, QoS):
        ```bash
        ros2 topic info /topic --verbose
        ```
        This command is invaluable for debugging, as it shows the message type, the number of publishers and subscribers, and crucially, the **QoS profile** for each connection. This helps verify that your chosen QoS settings are active.

---

### 🧠 Theory: ROS 2 Services
:::translate:::
While Topics provide asynchronous, one-to-many communication, **Services** offer a synchronous, request-response communication model. This is ideal when a node needs to explicitly request a computation or an action from another node and then wait for a response.

Think of it like a remote procedure call. A `NavigationNode` might need to know the current battery level. Instead of subscribing to a `/battery_status` topic (which might not update frequently or be relevant for every query), it can call a `BatteryMonitorNode`\'s `/get_battery_status` service. The `NavigationNode` sends a request, waits, and receives a response containing the battery data.

This synchronous nature means the calling node is blocked until the service returns a response or a timeout occurs. Services are well-suited for:
*   Configuration changes (e.g., setting a motor PID gain).
*   Triggering specific actions (e.g., `take_picture`, `start_slam`).
*   Querying current state (e.g., `get_map`, `get_joint_state`).

### 🛠️ Architecture
A client node requesting a service from a server node.

```mermaid
graph LR
    A[NavigationNode] -- Request/Response --> B[BatteryMonitorNode]
    A -- Call Service --> B(GetBatteryStatus Service)
```

### 💻 Implementation
This example demonstrates a simple service server and client. The `AddTwoIntsServer` node provides a service that adds two integers, and the `AddTwoIntsClient` node calls this service.

Context: These files would typically live at:
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_server.py`
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_client.py`

First, you\'ll need a service definition file. Create a `srv` directory in your `my_robot_pkg` and add `AddTwoInts.srv`:

Context: This file would typically live at /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/srv/AddTwoInts.srv

```
int64 a
int64 b
---
int64 sum
```
You\'ll also need to modify `setup.py` and `package.xml` to build this custom service message.

#### `package.xml` modification (inside `<my_robot_pkg>/package.xml`)
Add these lines:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

#### `setup.py` modification (inside `<my_robot_pkg>/setup.py`)
Add these imports and configuration:
```python
import os
from glob import glob
from setuptools import setup

package_name = \'my_robot_pkg\'

setup(
    name=package_name,
    version=\'0.0.0\',
    packages=[package_name],\
    data_files=[\
        (\'share/ament_index/resource_index/packages\',
            [\'resource/\' + package_name]),
        (\'share/\' + package_name, [\'package.xml\']),\
        # Include all srv files
        (os.path.join(\'share\', package_name, \'srv\'), glob(\'srv/*.srv\')),\
    ],\
    install_requires=[\'setuptools\'],\
    zip_safe=True,\
    maintainer=\'your_name\',\
    maintainer_email=\'your_email@example.com\',\
    description=\'TODO: Package description\',\
    license=\'TODO: License declaration\',\
    tests_require=[\'pytest\'],\
    entry_points={\
        \'console_scripts\': [\
            \'my_first_node = my_robot_pkg.my_first_node:main\',\
            \'minimal_publisher = my_robot_pkg.minimal_publisher:main\',\
            \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
            \'add_two_ints_server = my_robot_pkg.add_two_ints_server:main\',\
            \'add_two_ints_client = my_robot_pkg.add_two_ints_client:main\',\
        ],\
    },\
)
```

Now for the service server and client code:

#### Add Two Ints Server (`add_two_ints_server.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsServer(Node):
    \"\"\"\
    A ROS 2 Node that provides an \'add_two_ints\' service.\
    \"\"\"\
    def __init__(self):\
        super().__init__(\'add_two_ints_server\')
        # Create a service with the name \'add_two_ints\' and the AddTwoInts service type
        self.srv = self.create_service(AddTwoInts, \'add_two_ints\', self.add_two_ints_callback)
        self.get_logger().info(\'Add Two Ints Service Server started.\')

    def add_two_ints_callback(self, request, response):\
        \"\"\"\
        Callback function for the service. It receives a request and fills a response.\
        \"\"\"\
        response.sum = request.a + request.b
        self.get_logger().info(f\'Incoming request: a={request.a}, b={request.b}\')
        self.get_logger().info(f\'Sending response: sum={response.sum}\')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node) # Keep the node alive, waiting for service calls
    except KeyboardInterrupt:
        node.get_logger().info(\'Service server node stopped cleanly.\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```

#### Add Two Ints Client (`add_two_ints_client.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import your custom service type
import sys # For command line arguments

class AddTwoIntsClient(Node):
    \"\"\"\
    A ROS 2 Node that calls the \'add_two_ints\' service.\
    \"\"\"\
    def __init__(self):\
        super().__init__(\'add_two_ints_client\')
        # Create a service client for the \'add_two_ints\' service
        self.cli = self.create_client(AddTwoInts, \'add_two_ints\')

        # Wait for the service to be available. This is blocking.\
        while not self.cli.wait_for_service(timeout_sec=1.0):\
            if not rclpy.ok():\
                self.get_logger().error(\'Interrupted while waiting for the service. Exiting.\')\
                sys.exit(0)\
            self.get_logger().info(\'Service not available, waiting again...\')

        self.req = AddTwoInts.Request() # Create an empty service request object

    def send_request(self, a, b):\
        self.req.a = a
        self.req.b = b
        self.get_logger().info(f\'Calling service with a={self.req.a}, b={self.req.b}\')
        # Make the asynchronous service call
        self.future = self.cli.call_async(self.req)
        # Spin until the future is complete (response received or error)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    # Check for command line arguments
    if len(sys.argv) != 3:\
        node = rclpy.create_node(\'add_two_ints_client_error\')\
        node.get_logger().info(\'Usage: ros2 run my_robot_pkg add_two_ints_client <int_a> <int_b>\')\
        node.destroy_node()\
        sys.exit(1)\

    client_node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = client_node.send_request(a, b)

    if response is not None:\
        client_node.get_logger().info(f\'Result of add_two_ints: sum = {response.sum}\')
    else:\
        client_node.get_logger().error(\'Service call failed.\')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# To make this node executable, add to setup.py:\
# entry_points={\
#     \'console_scripts\': [\
#         \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
#     ],\
# },
```

### ⚠️ Common Pitfalls (Sim vs. Real)
:::translate:::
*   **Simulation**: Service calls in simulation are typically instantaneous, and service servers are always available. This can lead to design choices that don\'t account for real-world latencies or failures.
*   **Reality**: On a physical robot, service calls are synchronous and blocking. This means:
    *   **Blocking Calls**: If a client node calls a service and the server is busy, slow, or crashed, the client node will be blocked until a response or timeout. This can freeze the robot\'s control loop if not handled asynchronously, leading to jerky movements or a frozen state.
    *   **Timeouts**: Services need to complete within a reasonable time. If a complex computation or a physical action (like moving a joint) takes longer than expected, the client might timeout, assuming failure, even if the server is still working.
    *   **Service Availability**: In a distributed system with many nodes on an edge device, a service server might not be running yet, or it might crash. Clients must gracefully handle unavailable services, or the system will fail to initialize or recover.
*   **Fix**:
    *   **Asynchronous Service Clients**: For critical control paths, consider making service calls asynchronous and handling the response in a separate callback or thread to avoid blocking the main loop. `rclpy`\'s `call_async` method facilitates this.
    *   **Sensible Timeouts**: Always set realistic timeouts for `wait_for_service` and for the service calls themselves.
    *   **Error Handling**: Implement robust error handling in both client and server. What happens if the service request is invalid? What if the server encounters an internal error?
    *   **Service Availability Checks**: Clients should always use `wait_for_service()` before attempting to call a service, especially during startup.
    *   **Minimize Blocking Operations**: If a service server needs to perform a long-running task, it should ideally process the request in a separate thread to avoid blocking other client requests.

### 🧪 Verification
:::translate:::
First, ensure your package (including the `AddTwoInts.srv` file) is built and sourced as described in the \"ROS 2 Nodes\" verification section. You must re-run `colcon build` after adding the `.srv` file and modifying `setup.py` and `package.xml`.

1.  **Run the Service Server Node** (in one terminal):

    ```bash
    ros2 run my_robot_pkg add_two_ints_server
    ```
    You should see `[INFO] [add_two_ints_server]: Add Two Ints Service Server started.`

2.  **Call the Service using `ros2 service call`** (in a separate terminal):

    ```bash
    ros2 service call /add_two_ints my_robot_pkg/srv/AddTwoInts \"{a: 5, b: 3}\"
    ```
    You should see the client terminal outputting `sum: 8`, and the server terminal showing the incoming request and outgoing response.

3.  **Run the Service Client Node** (in a third terminal):

    ```bash
    ros2 run my_robot_pkg add_two_ints_client 10 20
    ```
    The client node will connect, call the service with `a=10` and `b=20`, and then print the result `sum = 30`.

4.  **Inspect Services**:
    *   List active services:
        ```bash
        ros2 service list
        ```
        You should see `/add_two_ints` in the list.
    *   Get information about the service:
        ```bash
        ros2 service info /add_two_ints
        ```
        This will show the service type, server, and client nodes connected to it.

---

### 🧠 Theory: ROS 2 Services
:::translate:::
While Topics provide asynchronous, one-to-many communication, **Services** offer a synchronous, request-response communication model. This is ideal when a node needs to explicitly request a computation or an action from another node and then wait for a response.

Think of it like a remote procedure call. A `NavigationNode` might need to know the current battery level. Instead of subscribing to a `/battery_status` topic (which might not update frequently or be relevant for every query), it can call a `BatteryMonitorNode`\'s `/get_battery_status` service. The `NavigationNode` sends a request, waits, and receives a response containing the battery data.

This synchronous nature means the calling node is blocked until the service returns a response or a timeout occurs. Services are well-suited for:
*   Configuration changes (e.g., setting a motor PID gain).
*   Triggering specific actions (e.g., `take_picture`, `start_slam`).
*   Querying current state (e.g., `get_map`, `get_joint_state`).

### 🛠️ Architecture
A client node requesting a service from a server node.

```mermaid
graph LR
    A[NavigationNode] -- Request/Response --> B[BatteryMonitorNode]
    A -- Call Service --> B(GetBatteryStatus Service)
```

### 💻 Implementation
This example demonstrates a simple service server and client. The `AddTwoIntsServer` node provides a service that adds two integers, and the `AddTwoIntsClient` node calls this service.

Context: These files would typically live at:
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_server.py`
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_client.py`

First, you\'ll need a service definition file. Create a `srv` directory in your `my_robot_pkg` and add `AddTwoInts.srv`:

Context: This file would typically live at /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/srv/AddTwoInts.srv

```
int64 a
int64 b
---
int64 sum
```
You\'ll also need to modify `setup.py` and `package.xml` to build this custom service message.

#### `package.xml` modification (inside `<my_robot_pkg>/package.xml`)
Add these lines:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

#### `setup.py` modification (inside `<my_robot_pkg>/setup.py`)
Add these imports and configuration:
```python
import os
from glob import glob
from setuptools import setup

package_name = \'my_robot_pkg\'

setup(
    name=package_name,
    version=\'0.0.0\',
    packages=[package_name],\
    data_files=[\
        (\'share/ament_index/resource_index/packages\',
            [\'resource/\' + package_name]),
        (\'share/\' + package_name, [\'package.xml\']),\
        # Include all srv files
        (os.path.join(\'share\', package_name, \'srv\'), glob(\'srv/*.srv\')),\
    ],\
    install_requires=[\'setuptools\'],\
    zip_safe=True,\
    maintainer=\'your_name\',\
    maintainer_email=\'your_email@example.com\',\
    description=\'TODO: Package description\',\
    license=\'TODO: License declaration\',\
    tests_require=[\'pytest\'],\
    entry_points={\
        \'console_scripts\': [\
            \'my_first_node = my_robot_pkg.my_first_node:main\',\
            \'minimal_publisher = my_robot_pkg.minimal_publisher:main\',\
            \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
            \'add_two_ints_server = my_robot_pkg.add_two_ints_server:main\',\
            \'add_two_ints_client = my_robot_pkg.add_two_ints_client:main\',\
        ],\
    },\
)
```

Now for the service server and client code:

#### Add Two Ints Server (`add_two_ints_server.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsServer(Node):
    \"\"\"\
    A ROS 2 Node that provides an \'add_two_ints\' service.\
    \"\"\"\
    def __init__(self):\
        super().__init__(\'add_two_ints_server\')
        # Create a service with the name \'add_two_ints\' and the AddTwoInts service type
        self.srv = self.create_service(AddTwoInts, \'add_two_ints\', self.add_two_ints_callback)
        self.get_logger().info(\'Add Two Ints Service Server started.\')

    def add_two_ints_callback(self, request, response):\
        \"\"\"\
        Callback function for the service. It receives a request and fills a response.\
        \"\"\"\
        response.sum = request.a + request.b
        self.get_logger().info(f\'Incoming request: a={request.a}, b={request.b}\')
        self.get_logger().info(f\'Sending response: sum={response.sum}\')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node) # Keep the node alive, waiting for service calls
    except KeyboardInterrupt:
        node.get_logger().info(\'Service server node stopped cleanly.\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```

#### Add Two Ints Client (`add_two_ints_client.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import your custom service type
import sys # For command line arguments

class AddTwoIntsClient(Node):
    \"\"\"\
    A ROS 2 Node that calls the \'add_two_ints\' service.\
    \"\"\"\
    def __init__(self):\
        super().__init__(\'add_two_ints_client\')
        # Create a service client for the \'add_two_ints\' service
        self.cli = self.create_client(AddTwoInts, \'add_two_ints\')

        # Wait for the service to be available. This is blocking.\
        while not self.cli.wait_for_service(timeout_sec=1.0):\
            if not rclpy.ok():\
                self.get_logger().error(\'Interrupted while waiting for the service. Exiting.\')\
                sys.exit(0)\
            self.get_logger().info(\'Service not available, waiting again...\')

        self.req = AddTwoInts.Request() # Create an empty service request object

    def send_request(self, a, b):\
        self.req.a = a
        self.req.b = b
        self.get_logger().info(f\'Calling service with a={self.req.a}, b={self.req.b}\')
        # Make the asynchronous service call
        self.future = self.cli.call_async(self.req)
        # Spin until the future is complete (response received or error)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    # Check for command line arguments
    if len(sys.argv) != 3:\
        node = rclpy.create_node(\'add_two_ints_client_error\')\
        node.get_logger().info(\'Usage: ros2 run my_robot_pkg add_two_ints_client <int_a> <int_b>\')\
        node.destroy_node()\
        sys.exit(1)\

    client_node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = client_node.send_request(a, b)

    if response is not None:\
        client_node.get_logger().info(f\'Result of add_two_ints: sum = {response.sum}\')
    else:\
        client_node.get_logger().error(\'Service call failed.\')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# To make this node executable, add to setup.py:\
# entry_points={\
#     \'console_scripts\': [\
#         \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
#     ],\
# },
```

### ⚠️ Common Pitfalls (Sim vs. Real)
:::translate:::
*   **Simulation**: Service calls in simulation are typically instantaneous, and service servers are always available. This can lead to design choices that don\'t account for real-world latencies or failures.
*   **Reality**: On a physical robot, service calls are synchronous and blocking. This means:
    *   **Blocking Calls**: If a client node calls a service and the server is busy, slow, or crashed, the client node will be blocked until a response or timeout. This can freeze the robot\'s control loop if not handled asynchronously, leading to jerky movements or a frozen state.
    *   **Timeouts**: Services need to complete within a reasonable time. If a complex computation or a physical action (like moving a joint) takes longer than expected, the client might timeout, assuming failure, even if the server is still working.
    *   **Service Availability**: In a distributed system with many nodes on an edge device, a service server might not be running yet, or it might crash. Clients must gracefully handle unavailable services, or the system will fail to initialize or recover.
*   **Fix**:
    *   **Asynchronous Service Clients**: For critical control paths, consider making service calls asynchronous and handling the response in a separate callback or thread to avoid blocking the main loop. `rclpy`\'s `call_async` method facilitates this.
    *   **Sensible Timeouts**: Always set realistic timeouts for `wait_for_service` and for the service calls themselves.
    *   **Error Handling**: Implement robust error handling in both client and server. What happens if the service request is invalid? What if the server encounters an internal error?
    *   **Service Availability Checks**: Clients should always use `wait_for_service()` before attempting to call a service, especially during startup.
    *   **Minimize Blocking Operations**: If a service server needs to perform a long-running task, it should ideally process the request in a separate thread to avoid blocking other client requests.

### 🧪 Verification
:::translate:::
First, ensure your package (including the `AddTwoInts.srv` file) is built and sourced as described in the \"ROS 2 Nodes\" verification section. You must re-run `colcon build` after adding the `.srv` file and modifying `setup.py` and `package.xml`.

1.  **Run the Service Server Node** (in one terminal):

    ```bash
    ros2 run my_robot_pkg add_two_ints_server
    ```
    You should see `[INFO] [add_two_ints_server]: Add Two Ints Service Server started.`

2.  **Call the Service using `ros2 service call`** (in a separate terminal):

    ```bash
    ros2 service call /add_two_ints my_robot_pkg/srv/AddTwoInts \"{a: 5, b: 3}\"
    ```
    You should see the client terminal outputting `sum: 8`, and the server terminal showing the incoming request and outgoing response.

3.  **Run the Service Client Node** (in a third terminal):

    ```bash
    ros2 run my_robot_pkg add_two_ints_client 10 20
    ```
    The client node will connect, call the service with `a=10` and `b=20`, and then print the result `sum = 30`.

4.  **Inspect Services**:
    *   List active services:
        ```bash
        ros2 service list
        ```
        You should see `/add_two_ints` in the list.
    *   Get information about the service:
        ```bash
        ros2 service info /add_two_ints
        ```
        This will show the service type, server, and client nodes connected to it.

---

### 📝 Chapter Summary
:::translate:::
In this chapter, we\'ve dissected the foundational elements of ROS 2: **Nodes**, **Topics**, and **Services**. We learned that Nodes are the modular, single-purpose processes forming the robot\'s distributed \"brain.\" Topics provide an asynchronous, publish-subscribe mechanism crucial for fluid data flow, with **Quality of Service (QoS)** settings acting as critical directives for reliability and latency in physical systems. Services, on the other hand, offer a synchronous request-response interaction, essential for explicit actions and state queries.

Understanding the stark differences between **Simulation and Reality** for each of these concepts is paramount. Latency, resource constraints, and communication reliability are abstract challenges in a simulator but become physical realities that can cause a robot to fail, fall, or even self-damage. By carefully defining QoS, structuring nodes for efficient edge computing, and anticipating real-world communication pitfalls, you are not just writing code—you are orchestrating the safe and precise movement of atoms.

### 🔚 Conclusion
We have now established the nervous system of our robot. Nodes can think, Topics can share senses, and Services can request actions. This infrastructure is powerful, but it is currently blind. In the next module, we will connect this nervous system to eyes and lasers, exploring how to capture and process the rich sensory data that allows a physical AI to perceive its world.
:::

:::translate:::ur
# باب 2: ROS 2 کی بنیادی باتیں: نوڈز، ٹاپکس، اور سروسز

### 🎯 سیکھنے کے مقاصد
اس باب کو مکمل کرنے کے بعد، طلباء اس قابل ہوں گے:
1.  **ROS 2 کے بنیادی تصورات کو سمجھیں**: ROS 2 نوڈز، ٹاپکس، اور سروسز کے مقصد کی تعریف اور وضاحت کریں جیسے بنیادی مواصلاتی اصول۔
2.  **ROS 2 نوڈز کو نافذ کریں**: سادہ فعالیتوں کے لیے بنیادی ROS 2 پائتھن نوڈز بنائیں، کمپائل کریں اور چلائیں۔
3.  **ٹاپک پر مبنی مواصلات میں مہارت حاصل کریں**: پبلشر اور سبسکرائبر نوڈز کو نافذ کریں، ROS 2 ٹاپکس کا استعمال کرتے ہوئے غیر مطابقت پذیر ڈیٹا کے بہاؤ کا مظاہرہ کریں۔
4.  **کوالٹی آف سروس (QoS) کی ترتیبات کا اطلاق کریں**: حقیقی دنیا کے روبوٹکس کے منظرناموں میں مختلف ڈیٹا کی اقسام (مثلاً، سینسر ڈیٹا بمقابلہ کنٹرول کمانڈز) کے لیے مناسب QoS پروفائلز (قابل اعتماد، تاریخ، گہرائی) کا انتخاب اور ترتیب دیں۔
5.  **سروس پر مبنی مواصلات کا استعمال کریں**: مخصوص روبوٹ ٹاسک کے لیے ہم وقت ساز درخواست-جواب کے تعاملات کو فعال کرنے کے لیے سروس سرور اور کلائنٹ نوڈز تیار کریں۔

### 🧠 نظریہ: ROS 2 نوڈز
:::
ROS 2 میں، **نوڈ** کمپیوٹیشن کی بنیادی اکائی ہے۔ یہ ایک ایسا عمل ہے جو ایک مخصوص کام انجام دیتا ہے۔ Unitree G1 جیسا ایک پیچیدہ روبوٹ ایک بڑے اسکرپٹ کے ذریعے کنٹرول نہیں ہوتا؛ یہ درجنوں (یا سینکڑوں) چھوٹے، ماڈیولر نوڈز کے ذریعے متوازی طور پر کام کرتا ہے۔
*   **ماڈیولرٹی**: ایک نوڈ کیمرہ ڈرائیور کو سنبھال سکتا ہے، دوسرا رکاوٹوں کے لیے تصاویر پر کارروائی کرتا ہے، اور تیسرا موٹر کمانڈز کا حساب لگاتا ہے۔ اگر کیمرہ ڈرائیور کریش ہو جاتا ہے، تو موٹر کنٹرولر چلتا رہتا ہے، حفاظت کو یقینی بناتا ہے۔
*   **دریافت**: ROS 2 ایک تقسیم شدہ دریافت میکانزم (DDS - Data Distribution Service) استعمال کرتا ہے۔ نوڈز مرکزی ماسٹر (ROS 1 کے برعکس) کے بغیر نیٹ ورک پر خود بخود ایک دوسرے کو تلاش کر لیتے ہیں، جس سے سسٹم مزید مضبوط اور کثیر روبوٹ بیڑے کے لیے لچکدار بن جاتا ہے۔

#### نوڈ کا لائف سائیکل
ایک منظم نوڈ (لائف سائیکل نوڈ) کی مخصوص حالتیں ہوتی ہیں: `Unconfigured`، `Inactive`، `Active`، اور `Finalized`۔ یہ فزیکل ہارڈویئر کے لیے بہت اہم ہے۔ آپ نہیں چاہتے کہ آپ کا موٹر کنٹرولر "Active" ہو اور حفاظتی جانچ کے ترتیب دیے جانے سے پہلے کرنٹ بھیجے۔

### 💻 نفاذ: ایک سادہ نوڈ بنانا
ہم ایک بنیادی پائتھن نوڈ بنائیں گے۔

Context: یہ فائل عام طور پر /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/my_first_node.py پر موجود ہوگی

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        # Initialize the node with the name \'my_first_node\'
        super().__init__(\'my_first_node\')
        self.get_logger().info(\'Hello from ROS 2!\')

        # Create a timer that calls \'timer_callback\' every 1.0 seconds
        self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f\'Counter: {self.counter}\')
        self.counter += 1

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = MyFirstNode()

    # Spin the node so it can process callbacks (blocking)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```

اس کو چلانے کے لیے، آپ عام طور پر اپنی `setup.py` میں ایک انٹری پوائنٹ شامل کریں گے اور `ros2 run` استعمال کریں گے۔

```bash
# سب سے پہلے، یقینی بنائیں کہ آپ کا ROS 2 ماحول سورس کیا گیا ہے
# یہ قدم ROS 2 کمانڈز کے لیے بہت اہم ہے
source /opt/ros/humble/setup.bash
# اگر آپ کسی colcon ورک اسپیس میں ہیں، تو اپنی ورک اسپیس سیٹ اپ بھی سورس کریں
# source /path/to/your/workspace/install/setup.bash

# اپنا پیکیج بنائیں (فرض کرتے ہوئے کہ آپ کے پاس ورک اسپیس سیٹ اپ ہے)
colcon build --packages-select my_robot_pkg

# نئے ایگزیکیوٹیبل کو شامل کرنے کے لیے بلڈ کے بعد دوبارہ اپنی ورک اسپیس سورس کریں
source install/setup.bash

# پھر، ایک ٹرمینل میں نوڈ کو چلائیں
ros2 run my_robot_pkg my_first_node
```

ایک **علیحدہ ٹرمینل** میں، آپ تمام فعال ROS 2 نوڈز کی فہرست دے سکتے ہیں:

```bash
ros2 node list
```

آپ کو آؤٹ پٹ میں `/my_first_node` دیکھنا چاہیے، جو اس کے چلنے کی تصدیق کرتا ہے۔

---

### 🧠 نظریہ: ROS 2 ٹاپکس
:::
**ٹاپکس** ROS 2 میں غیر مطابقت پذیر، کئی سے کئی مواصلات کا بنیادی میکانزم ہیں۔ وہ ایک پبلش-سبسکرائب ماڈل پر کام کرتے ہیں، جہاں نوڈز ایک نامزد ٹاپک پر ڈیٹا شائع کرتے ہیں، اور دوسرے نوڈز اس ٹاپک کو سبسکرائب کرتے ہیں تاکہ ڈیٹا وصول کر سکیں۔ یہ غیر مربوط مواصلات لچکدار اور قابل توسیع روبوٹک آرکیٹیکچرز کی اجازت دیتا ہے۔

ایک روبوٹ کے پرسیپشن سسٹم کا تصور کریں۔ ایک `CameraNode` خام امیج ڈیٹا کو `/camera/image_raw` ٹاپک پر شائع کر سکتا ہے۔ ایک `ObjectDetectionNode` اس ٹاپک کو سبسکرائب کرتا ہے، تصاویر پر کارروائی کرتا ہے، اور باؤنڈنگ باکس کی شناخت کو `/perception/objects` پر شائع کرتا ہے۔ آخر میں، ایک `NavigationNode` `/perception/objects` کو سبسکرائب کرتا ہے تاکہ اپنے راستے کی منصوبہ بندی کر سکے، رکاوٹوں سے بچ سکے۔ ہر نوڈ دوسرے نوڈز کے اندرونی کاموں کے بارے میں براہ راست علم کے بغیر اپنے مخصوص کام پر توجہ مرکوز کرتا ہے۔

#### کوالٹی آف سروس (QoS)
QoS صرف ایک سافٹ ویئر ایبسٹریکشن نہیں ہے؛ یہ بتاتا ہے کہ اہم ڈیٹا فزیکل اجزاء کے درمیان کیسے حرکت کرتا ہے۔ مجسم ذہانت کے لیے، QoS کی ترتیبات حفاظت اور کارکردگی کے لیے انتہائی اہم ہیں۔ جب آپ ایٹموں کو حرکت دیتے ہیں، تو درستگی اور وقت زندگی اور موت کی جنگ ہے۔

روبوٹکس کے لیے سب سے اہم QoS ترتیبات یہ ہیں:
*   **قابل اعتماد (Reliability)**:
    *   `Reliable`: اس بات کی ضمانت دیتا ہے کہ ہر پیغام ڈیلیور ہو گا، چاہے اس کا مطلب گمشدہ پیکٹوں کو دوبارہ منتقل کرنا ہی کیوں نہ ہو۔ **کنٹرول کمانڈز (مثلاً، `/cmd_vel`) کے لیے استعمال کریں جہاں کسی پیغام کا گم ہونا غیر محفوظ روبوٹ رویے کا سبب بن سکتا ہے۔**
    *   `Best Effort`: پیغامات کو ڈیلیور کرنے کی کوشش کرتا ہے، لیکن ڈیلیوری کی ضمانت نہیں دیتا۔ گمشدہ پیغامات دوبارہ منتقل نہیں ہوتے۔ **اعلی بینڈوتھ والے سینسر ڈیٹا (مثلاً، `/camera/image_raw`, `/scan`) کے لیے استعمال کریں جہاں ہر ایک فریم کو وصول کرنے سے زیادہ اہم یہ ہے کہ تازہ ترین ڈیٹا جلدی حاصل ہو۔**
*   **استحکام (Durability)**:
    *   `Transient Local`: نئے سبسکرائبرز منسلک ہونے کے فوراً بعد شائع کردہ آخری پیغام وصول کریں گے۔ جامد کنفیگریشن ڈیٹا کے لیے مفید ہے۔
    *   `Volatile`: صرف سبسکرپشن قائم ہونے کے *بعد* شائع ہونے والے پیغامات وصول کرتا ہے۔ زیادہ تر حقیقی وقت کے ڈیٹا کے لیے معیاری ہے۔
*   **تاریخ (History)**:
    *   `Keep Last (N)`: قطار میں آخری N پیغامات رکھتا ہے۔
    *   `Keep All`: قطار کی گہرائی تک تمام پیغامات رکھتا ہے۔
*   **گہرائی (Depth)**: پیغام کی قطار کا سائز۔ ایک گہری قطار زیادہ پیغامات کو بفر کر سکتی ہے لیکن ممکنہ تاخیر کو بڑھاتی ہے۔

### 🛠️ فن تعمیر
ایک ٹاپک کے ذریعے بات چیت کرنے والے دو نوڈز۔

```mermaid
graph LR
    A[CameraNode] -->|/camera/image_raw| B[ObjectDetectionNode]
```

### 💻 نفاذ
یہاں، ہم ایک سادہ پبلشر اور سبسکرائبر جوڑا بناتے ہیں۔ `MinimalPublisher` نوڈ \'topic\' ٹاپک پر String پیغامات شائع کرتا ہے، اور `MinimalSubscriber` نوڈ انہیں وصول کرتا ہے۔ QoS پروفائل کی تعریف پر گہری توجہ دیں، کیونکہ یہ حقیقی دنیا کے روبوٹکس کے لیے بہت اہم ہے۔

Context: یہ فائلیں عام طور پر اس پر موجود ہوں گی:
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/minimal_publisher.py`
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/minimal_subscriber.py`

#### کم سے کم پبلشر (`minimal_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String # Standard ROS 2 String message type

class MinimalPublisher(Node):
    \"\"\"
    ایک ROS 2 نوڈ جو \'topic\' ٹاپک پر String پیغامات شائع کرتا ہے۔
    ایک پبلشر کے لیے QoS ترتیبات کا مظاہرہ کرتا ہے۔
    \"\"\"\
    def __init__(self):
        super().__init__(\'minimal_publisher\')

        # سینسر ڈیٹا کے لیے QoS پروفائل کی تعریف کریں (Best Effort, آخری 1 رکھیں)
        # یہ اعلی تعدد ڈیٹا کے لیے موزوں ہے جہاں کبھی کبھار پیغام کا گم ہونا قابل قبول ہے
        # لیکن تازہ ترین ڈیٹا جلدی حاصل کرنا انتہائی اہم ہے۔
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # گارنٹی شدہ ترسیل پر رفتار کو ترجیح دیں
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # صرف تازہ ترین پیغام رکھیں
            durability=DurabilityPolicy.VOLATILE # صرف فعال سبسکرائبرز کو بھیجیں
        )

        # ایک پبلشر بنائیں جو \'topic\' ٹاپک پر String پیغامات شائع کرے گا۔
        # ہم یہاں sensor_qos_profile استعمال کرتے ہیں، یہ فرض کرتے ہوئے کہ یہ ایک سادہ سینسر سٹریم کی طرح ہو سکتا ہے
        self.publisher_ = self.create_publisher(String, \'topic\', sensor_qos_profile)

        self.i = 0
        # ایک ٹائمر بنائیں جو ہر 0.5 سیکنڈ میں timer_callback طریقہ کو کال کرتا ہے۔
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(f\'MinimalPublisher started with QoS: Reliability={sensor_qos_profile.reliability.name}, Depth={sensor_qos_profile.depth}\')

    def timer_callback(self):
        msg = String()
        msg.data = f\'Hello World: {self.i}\'
        self.publisher_.publish(msg)
        self.get_logger().info(f\'Publishing: \"{msg.data}\"\')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node) # نوڈ کو زندہ رکھیں
    except KeyboardInterrupt:
        node.get_logger().info(\'Publisher node stopped cleanly.\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# اس نوڈ کو قابل عمل بنانے کے لیے، setup.py میں شامل کریں:
# entry_points={
#     \'console_scripts\': [
#         \'minimal_publisher = my_robot_pkg.minimal_publisher:main\',\
#     ],\
# },
```

#### کم سے کم سبسکرائبر (`minimal_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String # Standard ROS 2 String message type

class MinimalSubscriber(Node):
    \"\"\"\
    ایک ROS 2 نوڈ جو \'topic\' ٹاپک سے String پیغامات کو سبسکرائب کرتا ہے۔
    ایک سبسکرائبر کے لیے QoS ترتیبات کا مظاہرہ کرتا ہے۔
    \"\"\"\
    def __init__(self):
        super().__init__(\'minimal_subscriber\')

        # کمانڈ ڈیٹا کے لیے QoS پروفائل کی تعریف کریں (Reliable, آخری 1 رکھیں)
        # یہ کنٹرول کمانڈز کے لیے موزوں ہے جہاں ہر پیغام کو وصول ہونا ضروری ہے
        # اور تاخیر ابھی بھی اہم ہے (اس لیے تازہ ترین کمانڈ کے لیے depth=1)۔
        command_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # ترسیل کی ضمانت
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # صرف تازہ ترین کمانڈ کی پرواہ کریں
            durability=DurabilityPolicy.VOLATILE # صرف فعال پبلشرز سے وصول کریں
        )

        # ایک سبسکرائبر بنائیں جو \'topic\' ٹاپک پر String پیغامات کو سنے گا۔
        # ہم یہاں command_qos_profile استعمال کرتے ہیں، یہ تصور کرتے ہوئے کہ یہ سبسکرائبر ایک موٹر کنٹرولر ہو سکتا ہے
        self.subscription = self.create_subscription(
            String,
            \'topic\',
            self.listener_callback,
            command_qos_profile # متعین QoS پروفائل استعمال کریں
        )
        self.subscription # غیر استعمال شدہ متغیر وارننگ سے بچیں
        self.get_logger().info(f\'MinimalSubscriber started with QoS: Reliability={command_qos_profile.reliability.name}, Depth={command_qos_profile.depth}\')

    def listener_callback(self, msg):\
        self.get_logger().info(f\'I heard: \"{msg.data}\"\')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node) # نوڈ کو زندہ رکھیں
    except KeyboardInterrupt:
        node.get_logger().info(\'Subscriber node stopped cleanly.\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# اس نوڈ کو قابل عمل بنانے کے لیے، setup.py میں شامل کریں:
# entry_points={
#     \'console_scripts\': [\
#         \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
#     ],\
# },
```

### ⚠️ عام خرابیاں (نقلی بمقابلہ حقیقی)
:::
*   **نقلی (Simulation)**: نقلی میں، نیٹ ورک کی تاخیر نہ ہونے کے برابر ہوتی ہے، اور پیغام کی ترسیل تقریباً ہمیشہ کامل ہوتی ہے۔ یہ ایک "خوشگوار راستہ" کی ذہنیت کا باعث بن سکتا ہے جہاں QoS کی ترتیبات غیر اہم لگتی ہیں۔
*   **حقیقت (Reality)**: ایک فزیکل روبوٹ پر، ٹاپکس حقیقی نیٹ ورکس (ایتھرنیٹ، وائی فائی، سیریل) پر منتقل ہوتے ہیں۔ یہ متعارف کراتا ہے:
    *   **تاخیر (Latency)**: اعلی تعدد والے سینسر ڈیٹا (مثلاً، 60Hz کیمرہ فیڈ) کو اگر بہت سے نوڈز کے ذریعے پروسیس کیا جائے یا سست روابط پر منتقل کیا جائے تو تاخیر کا سامنا کرنا پڑ سکتا ہے۔ وقت کے لحاظ سے اہم کنٹرول لوپس کے لیے، چند ملی سیکنڈ کی تاخیر بھی ایک فزیکل روبوٹ میں اتار چڑھاؤ یا عدم استحکام کا سبب بن سکتی ہے۔
    *   **پیغام کا گم ہونا (Message Loss)**: وائرلیس نیٹ ورکس مداخلت اور پیکٹ کے گم ہونے کا شکار ہوتے ہیں۔ اگر آپ کے کنٹرول کمانڈز `Best Effort` ہیں اور کوئی پیغام گم ہو جاتا ہے، تو روبوٹ کو ایک اہم اسٹاپ کمانڈ وصول نہیں ہو سکتا، جس سے تصادم یا گرنے کا خطرہ پیدا ہوتا ہے۔
    *   **ہم آہنگی کے مسائل (Synchronization Issues)**: اگر کوئی نوڈ متعدد ٹاپکس (مثلاً، کیمرہ اور IMU) سے ڈیٹا پر کارروائی کرتا ہے، تو مختلف اشاعت کی شرحیں اور نیٹ ورک کی تاخیر غیر ہم آہنگ ڈیٹا کا باعث بن سکتی ہے، جس سے SLAM جیسے الگورتھم کو غلطی یا ناکامی کا سامنا کرنا پڑ سکتا ہے۔
    *   **ہارڈویئر اوورلوڈ (Hardware Overload)**: بہت بڑے ڈیٹا کی اقسام (مثلاً، غیر کمپریسڈ 4K تصاویر) کو تیز رفتاری سے شائع کرنا نیٹ ورک یا ایج ڈیوائس کے CPU کو سیر کر سکتا ہے، جس سے دوسرے تمام نوڈز متاثر ہوتے ہیں۔
*   **حل**:
    *   **مناسب QoS**:
        *   **کنٹرول کمانڈز (`/cmd_vel`, `/joint_commands`)**: ہمیشہ `Reliable` قابل اعتماد استعمال کریں۔ یقینی بنائیں کہ `depth` مناسب ہے (اکثر `1` صرف تازہ ترین کمانڈ پر کارروائی کرنے کے لیے)۔
        *   **اعلی تعدد سینسر ڈیٹا (`/camera/image_raw`, `/scan`)**: `Best Effort` قابل اعتماد اور ایک چھوٹی `depth` (مثلاً، `1` سے `5`) استعمال کریں۔ تازہ ڈیٹا کو ترجیح دیں۔
    *   **بینڈوتھ کا انتظام**: بڑی ڈیٹا کی اقسام کو کمپریس کریں (مثلاً، کیمرہ فیڈز کے لیے `sensor_msgs/CompressedImage`)۔ اگر سختی سے ضروری نہ ہو تو اشاعت کی شرحوں کو کم کریں۔
    *   **ہارڈویئر ایکسیلریشن**: Jetson آلات پر ہارڈویئر ایکسیلریشن (مثلاً، GPU-accelerated امیج کمپریشن/ڈیکمپریشن کے لیے NVIDIA کا `image_transport`) کا استعمال کریں تاکہ CPU کو اوورلوڈ سے بچایا جا سکے۔
    *   **وقت کی ہم آہنگی**: متعدد سینسرز سے ڈیٹا کو ہینڈل کرنے کے لیے مضبوط وقت کی ہم آہنگی (مثلاً، سسٹم کلاک کے لیے NTP، ROS 2 پیغامات کے لیے `message_filters.ApproximateTimeSynchronizer`) کو نافذ کریں۔

### 🧪 تصدیق
:::
سب سے پہلے، یقینی بنائیں کہ آپ کا پیکیج "ROS 2 نوڈز" تصدیقی سیکشن میں بیان کردہ طریقے سے بنایا اور سورس کیا گیا ہے۔

1.  **پبلشر نوڈ چلائیں** (ایک ٹرمینل میں):

    ```bash
    ros2 run my_robot_pkg minimal_publisher
    ```
    آپ کو یہ آؤٹ پٹ دیکھنا چاہیے کہ پیغامات شائع ہو رہے ہیں: `[INFO] [minimal_publisher]: Publishing: \"Hello World: 0\"`

2.  **سبسکرائبر نوڈ چلائیں** (ایک علیحدہ ٹرمینل میں):

    ```bash
    ros2 run my_robot_pkg minimal_subscriber
    ```
    آپ کو یہ آؤٹ پٹ دیکھنا چاہیے کہ پیغامات وصول ہو رہے ہیں: `[INFO] [minimal_subscriber]: I heard: \"Hello World: 0\"`

3.  **ٹاپکس کا معائنہ کریں** (تیسرے ٹرمینل میں):
    *   فعال ٹاپکس کی فہرست:
        ```bash
        ros2 topic list
        ```
        آپ کو فہرست میں `/topic` دیکھنا چاہیے۔
    *   ٹاپک پر پیغامات کو گونج دیں:
        ```bash
        ros2 topic echo /topic
        ```
        یہ شائع ہونے والے `String` پیغامات کو ظاہر کرے گا، جو ڈیٹا کے بہاؤ کی تصدیق کرتا ہے۔
    *   ٹاپک کی معلومات کی جانچ کریں (قسم، پبلشرز، سبسکرائبرز، QoS):
        ```bash
        ros2 topic info /topic --verbose
        ```
        یہ کمانڈ ڈیبگنگ کے لیے انمول ہے، کیونکہ یہ پیغام کی قسم، پبلشرز اور سبسکرائبرز کی تعداد، اور خاص طور پر، ہر کنکشن کے لیے **QoS پروفائل** کو ظاہر کرتا ہے۔ یہ آپ کی منتخب کردہ QoS ترتیبات کے فعال ہونے کی تصدیق میں مدد کرتا ہے۔

---

### 🧠 نظریہ: ROS 2 سروسز
:::
جبکہ ٹاپکس غیر مطابقت پذیر، کئی سے کئی مواصلات فراہم کرتے ہیں، **سروسز** ایک ہم وقت ساز، درخواست-جواب مواصلاتی ماڈل پیش کرتی ہیں۔ یہ اس وقت مثالی ہے جب کسی نوڈ کو کسی دوسرے نوڈ سے ایک کمپیوٹیشن یا ایک عمل کی واضح طور پر درخواست کرنے کی ضرورت ہو اور پھر جواب کا انتظار کرنا پڑے۔

اسے ریموٹ پروسیجر کال کی طرح سمجھیں۔ ایک `NavigationNode` کو موجودہ بیٹری کی سطح جاننے کی ضرورت ہو سکتی ہے۔ `/battery_status` ٹاپک کو سبسکرائب کرنے کے بجائے (جو کثرت سے اپ ڈیٹ نہیں ہو سکتا یا ہر سوال کے لیے متعلقہ نہیں ہو سکتا)، یہ `BatteryMonitorNode` کی `/get_battery_status` سروس کو کال کر سکتا ہے۔ `NavigationNode` ایک درخواست بھیجتا ہے، انتظار کرتا ہے، اور بیٹری ڈیٹا پر مشتمل جواب وصول کرتا ہے۔

یہ ہم وقت ساز نوعیت کا مطلب ہے کہ کال کرنے والا نوڈ اس وقت تک بلاک ہو جاتا ہے جب تک کہ سروس جواب نہ دے یا ٹائم آؤٹ نہ ہو جائے۔ سروسز ان کے لیے موزوں ہیں:
*   کنفیگریشن میں تبدیلیاں (مثلاً، ایک موٹر PID گین سیٹ کرنا)۔
*   مخصوص اعمال کو متحرک کرنا (مثلاً، `take_picture`, `start_slam`)۔
*   موجودہ حالت کی استفسار (مثلاً، `get_map`, `get_joint_state`)۔

### 🛠️ فن تعمیر
ایک کلائنٹ نوڈ ایک سرور نوڈ سے سروس کی درخواست کر رہا ہے۔

```mermaid
graph LR
    A[NavigationNode] -- درخواست/جواب --> B[BatteryMonitorNode]
    A -- سروس کال کریں --> B(GetBatteryStatus سروس)
```

### 💻 نفاذ
یہ مثال ایک سادہ سروس سرور اور کلائنٹ کا مظاہرہ کرتی ہے۔ `AddTwoIntsServer` نوڈ ایک سروس فراہم کرتا ہے جو دو انٹیجرز کو جوڑتا ہے، اور `AddTwoIntsClient` نوڈ اس سروس کو کال کرتا ہے۔

Context: یہ فائلیں عام طور پر اس پر موجود ہوں گی:
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_server.py`
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_client.py`

سب سے پہلے، آپ کو ایک سروس ڈیفینیشن فائل کی ضرورت ہوگی۔ اپنے `my_robot_pkg` میں ایک `srv` ڈائریکٹری بنائیں اور `AddTwoInts.srv` شامل کریں:

Context: یہ فائل عام طور پر /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/srv/AddTwoInts.srv پر موجود ہوگی

```
int64 a
int64 b
---
int64 sum
```
آپ کو اس کسٹم سروس پیغام کو بنانے کے لیے `setup.py` اور `package.xml` میں بھی ترمیم کرنے کی ضرورت ہوگی۔

#### `package.xml` میں ترمیم ( `<my_robot_pkg>/package.xml` کے اندر)
یہ لائنیں شامل کریں:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

#### `setup.py` میں ترمیم ( `<my_robot_pkg>/setup.py` کے اندر)
یہ امپورٹس اور کنفیگریشن شامل کریں:
```python
import os
from glob import glob
from setuptools import setup

package_name = \'my_robot_pkg\'

setup(
    name=package_name,
    version=\'0.0.0\',
    packages=[package_name],\
    data_files=[\
        (\'share/ament_index/resource_index/packages\',
            [\'resource/\' + package_name]),
        (\'share/\' + package_name, [\'package.xml\']),\
        # تمام srv فائلیں شامل کریں
        (os.path.join(\'share\', package_name, \'srv\'), glob(\'srv/*.srv\')),\
    ],\
    install_requires=[\'setuptools\'],\
    zip_safe=True,\
    maintainer=\'آپ کا نام\',\
    maintainer_email=\'آپ کا ای میل@example.com\',\
    description=\'TODO: پیکیج کی تفصیل\',\
    license=\'TODO: لائسنس کا اعلان\',\
    tests_require=[\'pytest\'],\
    entry_points={\
        \'console_scripts\': [\
            \'my_first_node = my_robot_pkg.my_first_node:main\',\
            \'minimal_publisher = my_robot_pkg.minimal_publisher:main\',\
            \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
            \'add_two_ints_server = my_robot_pkg.add_two_ints_server:main\',\
            \'add_two_ints_client = my_robot_pkg.add_two_ints_client:main\',\
        ],\
    },\
)
```

اب سروس سرور اور کلائنٹ کوڈ کے لیے:

#### دو انٹیجرز سرور شامل کریں (`add_two_ints_server.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # اپنی کسٹم سروس ٹائپ امپورٹ کریں

class AddTwoIntsServer(Node):
    \"\"\"\
    ایک ROS 2 نوڈ جو \'add_two_ints\' سروس فراہم کرتا ہے۔
    \"\"\"\
    def __init__(self):\
        super().__init__(\'add_two_ints_server\')
        # \'add_two_ints\' نام اور AddTwoInts سروس ٹائپ کے ساتھ ایک سروس بنائیں
        self.srv = self.create_service(AddTwoInts, \'add_two_ints\', self.add_two_ints_callback)
        self.get_logger().info(\'Add Two Ints Service Server started.\')

    def add_two_ints_callback(self, request, response):\
        \"\"\"\
        سروس کے لیے کال بیک فنکشن۔ یہ ایک درخواست وصول کرتا ہے اور جواب بھرتا ہے۔
        \"\"\"\
        response.sum = request.a + request.b
        self.get_logger().info(f\'آنے والی درخواست: a={request.a}, b={request.b}\')
        self.get_logger().info(f\'جواب بھیج رہا ہے: sum={response.sum}\')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node) # نوڈ کو زندہ رکھیں، سروس کالز کا انتظار کر رہے ہیں
    except KeyboardInterrupt:
        node.get_logger().info(\'سروس سرور نوڈ صاف طور پر بند ہو گیا۔\')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```

#### دو انٹیجرز کلائنٹ شامل کریں (`add_two_ints_client.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # اپنی کسٹم سروس ٹائپ امپورٹ کریں
import sys # کمانڈ لائن دلائل کے لیے

class AddTwoIntsClient(Node):
    \"\"\"\
    ایک ROS 2 نوڈ جو \'add_two_ints\' سروس کو کال کرتا ہے۔
    \"\"\"\
    def __init__(self):\
        super().__init__(\'add_two_ints_client\')
        # \'add_two_ints\' سروس کے لیے ایک سروس کلائنٹ بنائیں
        self.cli = self.create_client(AddTwoInts, \'add_two_ints\')

        # سروس کے دستیاب ہونے کا انتظار کریں۔ یہ بلاکنگ ہے۔
        while not self.cli.wait_for_service(timeout_sec=1.0):\
            if not rclpy.ok():\
                self.get_logger().error(\'سروس کا انتظار کرتے ہوئے مداخلت کی گئی۔ باہر نکل رہا ہے۔\')\
                sys.exit(0)\
            self.get_logger().info(\'سروس دستیاب نہیں، دوبارہ انتظار کر رہا ہوں...\')

        self.req = AddTwoInts.Request() # ایک خالی سروس درخواست آبجیکٹ بنائیں

    def send_request(self, a, b):\
        self.req.a = a
        self.req.b = b
        self.get_logger().info(f\'سروس کو کال کر رہا ہے: a={self.req.a}, b={self.req.b}\')
        # غیر مطابقت پذیر سروس کال کریں
        self.future = self.cli.call_async(self.req)
        # فیوچر مکمل ہونے تک گھمائیں (جواب موصول ہوا یا غلطی)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    # کمانڈ لائن دلائل کی جانچ کریں
    if len(sys.argv) != 3:\
        node = rclpy.create_node(\'add_two_ints_client_error\')\
        node.get_logger().info(\'استعمال: ros2 run my_robot_pkg add_two_ints_client <int_a> <int_b>\')\
        node.destroy_node()\
        sys.exit(1)\

    client_node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = client_node.send_request(a, b)

    if response is not None:\
        client_node.get_logger().info(f\'add_two_ints کا نتیجہ: sum = {response.sum}\')
    else:\
        client_node.get_logger().error(\'سروس کال ناکام ہو گئی۔\')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
# اس نوڈ کو قابل عمل بنانے کے لیے، setup.py میں شامل کریں:
# entry_points={
#     \'console_scripts\': [\
#         \'minimal_subscriber = my_robot_pkg.minimal_subscriber:main\',\
#     ],\
# },
```

### ⚠️ عام خرابیاں (نقلی بمقابلہ حقیقی)
:::
*   **نقلی (Simulation)**: نقلی میں، سروس کالز عام طور پر فوری ہوتی ہیں، اور سروس سرورز ہمیشہ دستیاب ہوتے ہیں۔ یہ ایسے ڈیزائن کے انتخاب کا باعث بن سکتا ہے جو حقیقی دنیا کی تاخیر یا ناکامیوں کا حساب نہیں لیتے۔
*   **حقیقت (Reality)**: ایک فزیکل روبوٹ پر، سروس کالز ہم وقت ساز اور بلاکنگ ہوتی ہیں۔ اس کا مطلب ہے:
    *   **بلاکنگ کالز (Blocking Calls)**: اگر کوئی کلائنٹ نوڈ کسی سروس کو کال کرتا ہے اور سرور مصروف، سست، یا کریش ہو گیا ہے، تو کلائنٹ نوڈ جواب یا ٹائم آؤٹ تک بلاک رہے گا۔ اگر اسے غیر مطابقت پذیر طریقے سے ہینڈل نہ کیا جائے تو یہ روبوٹ کے کنٹرول لوپ کو منجمد کر سکتا ہے، جس سے جھٹکے دار حرکات یا منجمد حالت ہو سکتی ہے۔
    *   **ٹائم آؤٹ (Timeouts)**: سروسز کو معقول وقت کے اندر مکمل ہونا چاہیے۔ اگر ایک پیچیدہ کمپیوٹیشن یا ایک فزیکل عمل (جیسے ایک جوائنٹ کو حرکت دینا) توقع سے زیادہ وقت لیتا ہے، تو کلائنٹ ٹائم آؤٹ ہو سکتا ہے، ناکامی کو فرض کرتے ہوئے، یہاں تک کہ اگر سرور ابھی بھی کام کر رہا ہو۔
    *   **سروس کی دستیابی (Service Availability)**: ایک ایج ڈیوائس پر بہت سے نوڈز کے ساتھ ایک تقسیم شدہ نظام میں، ایک سروس سرور ابھی چل نہیں رہا ہو سکتا، یا یہ کریش ہو سکتا ہے۔ کلائنٹس کو غیر دستیاب سروسز کو خوبصورتی سے ہینڈل کرنا چاہیے، ورنہ سسٹم شروع کرنے یا بحال ہونے میں ناکام ہو جائے گا۔
*   **حل**:
    *   **غیر مطابقت پذیر سروس کلائنٹس (Asynchronous Service Clients)**: اہم کنٹرول راستوں کے لیے، سروس کالز کو غیر مطابقت پذیر بنانے اور جواب کو ایک علیحدہ کال بیک یا تھریڈ میں ہینڈل کرنے پر غور کریں تاکہ مرکزی لوپ کو بلاک کرنے سے بچا جا سکے۔ `rclpy` کا `call_async` طریقہ اس میں سہولت فراہم کرتا ہے۔
    *   **معقول ٹائم آؤٹ (Sensible Timeouts)**: `wait_for_service` اور خود سروس کالز کے لیے ہمیشہ حقیقت پسندانہ ٹائم آؤٹ سیٹ کریں۔
    *   **غلطی کی ہینڈلنگ (Error Handling)**: کلائنٹ اور سرور دونوں میں مضبوط غلطی کی ہینڈلنگ کو نافذ کریں۔ اگر سروس کی درخواست غلط ہے تو کیا ہوتا ہے؟ اگر سرور کو اندرونی غلطی کا سامنا کرنا پڑتا ہے تو کیا ہوتا ہے؟
    *   **سروس کی دستیابی کی جانچ (Service Availability Checks)**: کلائنٹس کو ہمیشہ سروس کو کال کرنے کی کوشش کرنے سے پہلے `wait_for_service()` استعمال کرنا چاہیے، خاص طور پر آغاز کے دوران۔
    *   **بلاکنگ آپریشنز کو کم سے کم کریں (Minimize Blocking Operations)**: اگر کسی سروس سرور کو ایک طویل عرصے تک چلنے والا کام انجام دینے کی ضرورت ہے، تو اسے مثالی طور پر ایک علیحدہ تھریڈ میں درخواست پر کارروائی کرنی چاہیے تاکہ دوسرے کلائنٹ کی درخواستوں کو بلاک کرنے سے بچا جا سکے۔

### 🧪 تصدیق
:::
سب سے پہلے، یقینی بنائیں کہ آپ کا پیکیج (بشمول `AddTwoInts.srv` فائل) "ROS 2 نوڈز" تصدیقی سیکشن میں بیان کردہ طریقے سے بنایا اور سورس کیا گیا ہے۔ آپ کو `.srv` فائل شامل کرنے اور `setup.py` اور `package.xml` میں ترمیم کرنے کے بعد `colcon build` کو دوبارہ چلانا چاہیے۔

1.  **سروس سرور نوڈ چلائیں** (ایک ٹرمینل میں):

    ```bash
    ros2 run my_robot_pkg add_two_ints_server
    ```
    آپ کو یہ دیکھنا چاہیے `[INFO] [add_two_ints_server]: Add Two Ints Service Server started.`

2.  **`ros2 service call` کا استعمال کرتے ہوئے سروس کو کال کریں** (ایک علیحدہ ٹرمینل میں):

    ```bash
    ros2 service call /add_two_ints my_robot_pkg/srv/AddTwoInts \"{a: 5, b: 3}\"
    ```
    آپ کو کلائنٹ ٹرمینل کو `sum: 8` آؤٹ پٹ کرتے ہوئے دیکھنا چاہیے، اور سرور ٹرمینل کو آنے والی درخواست اور جانے والے جواب کو دکھاتے ہوئے۔

3.  **سروس کلائنٹ نوڈ چلائیں** (تیسرے ٹرمینل میں):

    ```bash
    ros2 run my_robot_pkg add_two_ints_client 10 20
    ```
    کلائنٹ نوڈ جڑ جائے گا، `a=10` اور `b=20` کے ساتھ سروس کو کال کرے گا، اور پھر نتیجہ `sum = 30` پرنٹ کرے گا۔

4.  **سروسز کا معائنہ کریں**:
    *   فعال سروسز کی فہرست:
        ```bash
        ros2 service list
        ```
        آپ کو فہرست میں `/add_two_ints` دیکھنا چاہیے۔
    *   سروس کے بارے میں معلومات حاصل کریں:
        ```bash
        ros2 service info /add_two_ints
        ```
        یہ سروس کی قسم، سرور، اور اس سے منسلک کلائنٹ نوڈز کو ظاہر کرے گا۔

---

### 📝 باب کا خلاصہ
:::
اس باب میں، ہم نے ROS 2 کے بنیادی عناصر: **نوڈز**، **ٹاپکس**، اور **سروسز** کا تجزیہ کیا ہے۔ ہم نے سیکھا کہ نوڈز روبوٹ کے تقسیم شدہ "دماغ" کو بنانے والے ماڈیولر، واحد مقصد والے عمل ہیں۔ ٹاپکس ایک غیر مطابقت پذیر، پبلش-سبسکرائب میکانزم فراہم کرتے ہیں جو ڈیٹا کے ہموار بہاؤ کے لیے اہم ہے، جس میں **کوالٹی آف سروس (QoS)** کی ترتیبات فزیکل سسٹمز میں قابل اعتماد اور تاخیر کے لیے اہم ہدایات کے طور پر کام کرتی ہیں۔ دوسری طرف، سروسز ایک ہم وقت ساز درخواست-جواب کا تعامل پیش کرتی ہیں، جو مخصوص روبوٹ کاموں کے لیے واضح اعمال اور حالت کے استفسارات کے لیے ضروری ہے۔

ان تصورات میں سے ہر ایک کے لیے **نقلی اور حقیقت** کے درمیان نمایاں اختلافات کو سمجھنا انتہائی اہم ہے۔ تاخیر، وسائل کی رکاوٹیں، اور مواصلات کی قابل اعتمادی ایک سمیلیٹر میں تجریدی چیلنجز ہیں لیکن فزیکل حقیقت بن جاتے ہیں جو ایک روبوٹ کو ناکام، گرنے، یا یہاں تک کہ خود کو نقصان پہنچانے کا سبب بن سکتے ہیں۔ QoS کی احتیاط سے تعریف کرکے، موثر ایج کمپیوٹنگ کے لیے نوڈز کی ساخت بنا کر، اور حقیقی دنیا کے مواصلاتی خرابیوں کا اندازہ لگا کر، آپ صرف کوڈ نہیں لکھ رہے ہیں - آپ ایٹموں کی محفوظ اور درست حرکت کا انتظام کر رہے ہیں۔

### 🔚 نتیجہ
ہم نے اب اپنے روبوٹ کا اعصابی نظام قائم کر لیا ہے۔ نوڈز سوچ سکتے ہیں، ٹاپکس حواس کو بانٹ سکتے ہیں، اور سروسز اعمال کی درخواست کر سکتی ہیں۔ یہ بنیادی ڈھانچہ طاقتور ہے، لیکن یہ فی الحال نابینا ہے۔ اگلے ماڈیول میں، ہم اس اعصابی نظام کو آنکھوں اور لیزرز سے جوڑیں گے، یہ دریافت کریں گے کہ کس طرح بھرپور حسی ڈیٹا کو حاصل اور پروسیس کیا جائے جو ایک فزیکل اے آئی کو اپنی دنیا کو سمجھنے کی اجازت دیتا ہے۔
:::
