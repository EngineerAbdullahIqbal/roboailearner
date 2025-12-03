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
In ROS 2, a **Node** is the fundamental unit of computation. It is a process that performs a specific task. A complex robot like a Unitree G1 isn't controlled by one giant script; it's controlled by dozens (or hundreds) of small, modular nodes working in parallel.
*   **Modularity**: One node might handle the camera driver, another processes images for obstacles, and a third calculates motor commands. If the camera driver crashes, the motor controller keeps running, ensuring safety.
*   **Discovery**: ROS 2 uses a distributed discovery mechanism (DDS - Data Distribution Service). Nodes automatically find each other on the network without a central master (unlike ROS 1), making the system more robust and flexible for multi-robot fleets.

#### Life Cycle of a Node
A managed node (Lifecycle Node) has specific states: `Unconfigured`, `Inactive`, `Active`, and `Finalized`. This is vital for physical hardware. You don't want your motor controller to be "Active" and sending current before the safety checks have been configured.

### 💻 Implementation: Creating a Simple Node
We'll create a basic Python node.

#### Node Communication and Graph Structure
ROS 2 nodes typically form a graph where nodes communicate with each other through topics, services, and actions. This graph represents the entire computational architecture of the robot. Tools like `rqt_graph` allow developers to visualize this dynamic graph, which is invaluable for debugging and understanding the data flow within a complex robotic system. Each node operates independently, reducing system coupling and increasing robustness. If one sensor driver node fails, other parts of the robot’s control system can often continue operating, perhaps with degraded performance, rather than causing a complete system shutdown. This modularity is a cornerstone of scalable and fault-tolerant robotic software.

Context: This file would typically live at /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/my_first_node.py

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        # Initialize the node with the name 'my_first_node'
        super().__init__('my_first_node')
        self.get_logger().info('Hello from ROS 2!')

        # Create a timer that calls 'timer_callback' every 1.0 seconds
        self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Counter: {self.counter}')
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

if __name__ == '__main__':
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

You should see `/my_first_node` in the output, confirming it's running.

---

### 🧠 Theory: ROS 2 Topics
:::translate:::
**Topics** are the primary mechanism for asynchronous, many-to-many communication in ROS 2. They operate on a publish-subscribe model, where nodes publish data to a named topic, and other nodes subscribe to that topic to receive the data. This decoupled communication allows for flexible and scalable robotic architectures.

Imagine a robot's perception system. A `CameraNode` might publish raw image data to a topic `/camera/image_raw`. An `ObjectDetectionNode` subscribes to this topic, processes the images, and publishes bounding box detections to `/perception/objects`. Finally, a `NavigationNode` subscribes to `/perception/objects` to plan its path, avoiding obstacles. Each node focuses on its specific task without needing direct knowledge of other nodes' internal workings.

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

import ThreeDiagram from '@site/src/components/ThreeDiagram';

<ThreeDiagram id="2.1" />

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
    """
    A ROS 2 Node that publishes String messages to the 'topic' topic.
    Demonstrates QoS settings for a publisher.
    """
    def __init__(self):
        super().__init__('minimal_publisher')

        # Define a QoS profile for sensor data (Best Effort, keep last 1)
        # This is suitable for high-frequency data where missing an occasional message is acceptable
        # but getting the latest data quickly is paramount.
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Prioritize speed over guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Only keep the latest message
            durability=DurabilityPolicy.VOLATILE # Only send to active subscribers
        )

        # Create a publisher that will publish String messages to the 'topic' topic
        # We use the sensor_qos_profile here, assuming this might be like a simple sensor stream
        self.publisher_ = self.create_publisher(String, 'topic', sensor_qos_profile)

        self.i = 0
        # Create a timer that calls 'timer_callback' method every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(f'MinimalPublisher started with QoS: Reliability={sensor_qos_profile.reliability.name}, Depth={sensor_qos_profile.depth}')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        node.get_logger().info('Publisher node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# To make this node executable, add to setup.py:
# entry_points={
#     'console_scripts': [
#         'minimal_publisher = my_robot_pkg.minimal_publisher:main',
#     ],
# },
```

#### Minimal Subscriber (`minimal_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String # Standard ROS 2 String message type

class MinimalSubscriber(Node):
    """
    A ROS 2 Node that subscribes to String messages from the 'topic' topic.
    Demonstrates QoS settings for a subscriber.
    """
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Define a QoS profile for command data (Reliable, keep last 1)
        # This is suitable for control commands where every message MUST be received
        # and latency is still important (hence depth=1 for the latest command).
        command_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Guarantee delivery
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Only care about the latest command
            durability=DurabilityPolicy.VOLATILE # Only receive from active publishers
        )

        # Create a subscriber that will listen for String messages on the 'topic' topic
        # We use the command_qos_profile here, imagining this subscriber might be a motor controller
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            command_qos_profile # Use the defined QoS profile
        )
        self.subscription # prevent unused variable warning
        self.get_logger().info(f'MinimalSubscriber started with QoS: Reliability={command_qos_profile.reliability.name}, Depth={command_qos_profile.depth}')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# To make this node executable, add to setup.py:
# entry_points={
#     'console_scripts': [
#         'minimal_subscriber = my_robot_pkg.minimal_subscriber:main',
#     ],
# },
```

### ⚠️ Common Pitfalls (Sim vs. Real)
:::translate:::
*   **Simulation**: In simulation, network latency is negligible, and message delivery is almost always perfect. This can lead to a "happy path" mentality where QoS settings seem unimportant.
*   **Reality**: On a physical robot, topics are transmitted over real networks (Ethernet, Wi-Fi, serial). This introduces:
    *   **Latency**: High-frequency sensor data (e.g., a 60Hz camera feed) might experience delays if processed by many nodes or transmitted over slow links. For time-critical control loops, even a few milliseconds of latency can cause oscillations or instability in a physical robot.
    *   **Message Loss**: Wireless networks are prone to interference and packet loss. If your control commands are `Best Effort` and a message is lost, the robot might not receive a critical stop command, leading to collision or falls.
    *   **Synchronization Issues**: If a node processes data from multiple topics (e.g., camera and IMU), differing publication rates and network delays can lead to unsynchronized data, causing algorithms like SLAM to drift or fail.
    *   **Hardware Overload**: Publishing very large messages (e.g., uncompressed 4K images) at high rates can saturate the network or the edge device's CPU, impacting all other nodes.
*   **Fix**:
    *   **Appropriate QoS**:
        *   **Control Commands (`/cmd_vel`, `/joint_commands`)**: ALWAYS use `Reliable` reliability. Ensure the `depth` is appropriate (often `1` to only process the latest command).
        *   **High-Frequency Sensor Data (`/camera/image_raw`, `/scan`)**: Use `Best Effort` reliability and a small `depth` (e.g., `1` to `5`). Prioritize fresh data.
    *   **Bandwidth Management**: Compress large data types (e.g., `sensor_msgs/CompressedImage` for camera feeds). Reduce publication rates if not strictly necessary.
    *   **Hardware Acceleration**: Utilize hardware acceleration (e.g., NVIDIA's `image_transport` for GPU-accelerated image compression/decompression) on Jetson devices to offload CPU.
    *   **Time Synchronization**: Implement robust time synchronization (e.g., NTP for system clocks, `message_filters.ApproximateTimeSynchronizer` for ROS 2 messages) to handle data from multiple sensors.

### 🧪 Verification
:::translate:::
First, ensure your package is built and sourced as described in the "ROS 2 Nodes" verification section.

1.  **Run the Publisher Node** (in one terminal):

    ```bash
    ros2 run my_robot_pkg minimal_publisher
    ```
    You should see output indicating messages being published: `[INFO] [minimal_publisher]: Publishing: "Hello World: 0"`

2.  **Run the Subscriber Node** (in a separate terminal):

    ```bash
    ros2 run my_robot_pkg minimal_subscriber
    ```
    You should see output indicating messages being received: `[INFO] [minimal_subscriber]: I heard: "Hello World: 0"`

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

Think of it like a remote procedure call. A `NavigationNode` might need to know the current battery level. Instead of subscribing to a `/battery_status` topic (which might not update frequently or be relevant for every query), it can call a `BatteryMonitorNode`'s `/get_battery_status` service. The `NavigationNode` sends a request, waits, and receives a response containing the battery data.

This synchronous nature means the calling node is blocked until the service returns a response or a timeout occurs. Services are well-suited for:
*   Configuration changes (e.g., setting a motor PID gain).
*   Triggering specific actions (e.g., `take_picture`, `start_slam`).
*   Querying current state (e.g., `get_map`, `get_joint_state`).

### 🛠️ Architecture
A client node requesting a service from a server node.

<ThreeDiagram id="2.2" />

### 💻 Implementation
This example demonstrates a simple service server and client. The `AddTwoIntsServer` node provides a service that adds two integers, and the `AddTwoIntsClient` node calls this service.

Context: These files would typically live at:
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_server.py`
*   `/home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/my_robot_pkg/add_two_ints_client.py`

First, you'll need a service definition file. Create a `srv` directory in your `my_robot_pkg` and add `AddTwoInts.srv`:

Context: This file would typically live at /home/abdullahiqbal/Abdullah/hackathon-book-project/src/my_robot_pkg/srv/AddTwoInts.srv

```
int64 a
int64 b
---
int64 sum
```
You'll also need to modify `setup.py` and `package.xml` to build this custom service message.

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

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all srv files
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_robot_pkg.my_first_node:main',
            'minimal_publisher = my_robot_pkg.minimal_publisher:main',
            'minimal_subscriber = my_robot_pkg.minimal_subscriber:main',
            'add_two_ints_server = my_robot_pkg.add_two_ints_server:main',
            'add_two_ints_client = my_robot_pkg.add_two_ints_client:main',
        ],
    },
)
```

Now for the service server and client code:

#### Add Two Ints Server (`add_two_ints_server.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsServer(Node):
    """
    A ROS 2 Node that provides an 'add_two_ints' service.
    """
    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service with the name 'add_two_ints' and the AddTwoInts service type
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Server started.')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for the service. It receives a request and fills a response.
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node) # Keep the node alive, waiting for service calls
    except KeyboardInterrupt:
        node.get_logger().info('Service server node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Add Two Ints Client (`add_two_ints_client.py`)

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import your custom service type
import sys # For command line arguments

class AddTwoIntsClient(Node):
    """
    A ROS 2 Node that calls the 'add_two_ints' service.
    """
    def __init__(self):
        super().__init__('add_two_ints_client')
        # Create a service client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available. This is blocking.
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request() # Create an empty service request object

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.get_logger().info(f'Calling service with a={self.req.a}, b={self.req.b}')
        # Make the asynchronous service call
        self.future = self.cli.call_async(self.req)
        # Spin until the future is complete (response received or error)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    # Check for command line arguments
    if len(sys.argv) != 3:
        node = rclpy.create_node('add_two_ints_client_error')
        node.get_logger().info('Usage: ros2 run my_robot_pkg add_two_ints_client <int_a> <int_b>')
        node.destroy_node()
        sys.exit(1)

    client_node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = client_node.send_request(a, b)

    if response is not None:
        client_node.get_logger().info(f'Result of add_two_ints: sum = {response.sum}')
    else:
        client_node.get_logger().error('Service call failed.')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# To make this node executable, add to setup.py:
# entry_points={
#     'console_scripts': [
#         'minimal_subscriber = my_robot_pkg.minimal_subscriber:main',
#     ],
# },
```

### ⚠️ Common Pitfalls (Sim vs. Real)
:::translate:::
*   **Simulation**: Service calls in simulation are typically instantaneous, and service servers are always available. This can lead to design choices that don't account for real-world latencies or failures.
*   **Reality**: On a physical robot, service calls are synchronous and blocking. This means:
    *   **Blocking Calls**: If a client node calls a service and the server is busy, slow, or crashed, the client node will be blocked until a response or timeout. This can freeze the robot's control loop if not handled asynchronously, leading to jerky movements or a frozen state.
    *   **Timeouts**: Services need to complete within a reasonable time. If a complex computation or a physical action (like moving a joint) takes longer than expected, the client might timeout, assuming failure, even if the server is still working.
    *   **Service Availability**: In a distributed system with many nodes on an edge device, a service server might not be running yet, or it might crash. Clients must gracefully handle unavailable services, or the system will fail to initialize or recover.
*   **Fix**:
    *   **Asynchronous Service Clients**: For critical control paths, consider making service calls asynchronous and handling the response in a separate callback or thread to avoid blocking the main loop. `rclpy`'s `call_async` method facilitates this.
    *   **Sensible Timeouts**: Always set realistic timeouts for `wait_for_service` and for the service calls themselves.
    *   **Error Handling**: Implement robust error handling in both client and server. What happens if the service request is invalid? What if the server encounters an internal error?
    *   **Service Availability Checks**: Clients should always use `wait_for_service()` before attempting to call a service, especially during startup.
    *   **Minimize Blocking Operations**: If a service server needs to perform a long-running task, it should ideally process the request in a separate thread to avoid blocking other client requests.

### 🧪 Verification
:::translate:::
First, ensure your package (including the `AddTwoInts.srv` file) is built and sourced as described in the "ROS 2 Nodes" verification section. You must re-run `colcon build` after adding the `.srv` file and modifying `setup.py` and `package.xml`.

1.  **Run the Service Server Node** (in one terminal):

    ```bash
    ros2 run my_robot_pkg add_two_ints_server
    ```
    You should see `[INFO] [add_two_ints_server]: Add Two Ints Service Server started.`

2.  **Call the Service using `ros2 service call`** (in a separate terminal):

    ```bash
    ros2 service call /add_two_ints my_robot_pkg/srv/AddTwoInts "{a: 5, b: 3}"
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
In this chapter, we've dissected the foundational elements of ROS 2: **Nodes**, **Topics**, and **Services**. We learned that Nodes are the modular, single-purpose processes forming the robot's distributed "brain." Topics provide an asynchronous, publish-subscribe mechanism crucial for fluid data flow, with **Quality of Service (QoS)** settings acting as critical directives for reliability and latency in physical systems. Services, on the other hand, offer a synchronous request-response interaction, essential for explicit actions and state queries.

Understanding the stark differences between **Simulation and Reality** for each of these concepts is paramount. Latency, resource constraints, and communication reliability are abstract challenges in a simulator but become physical realities that can cause a robot to fail, fall, or even self-damage. By carefully defining QoS, structuring nodes for efficient edge computing, and anticipating real-world communication pitfalls, you are not just writing code—you are orchestrating the safe and precise movement of atoms.

### 🔚 Conclusion
We have now established the nervous system of our robot. Nodes can think, Topics can share senses, and Services can request actions. This infrastructure is powerful, but it is currently blind. In the next module, we will connect this nervous system to eyes and lasers, exploring how to capture and process the rich sensory data that allows a physical AI to perceive its world.
:::