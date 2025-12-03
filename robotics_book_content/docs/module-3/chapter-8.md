# Chapter 8: Robot Control: Actuators, Joints, and PID

### 🎯 Objective
This chapter bridges the gap between a motion plan (trajectory) and the physical voltage applied to motors. We will dissect the PID controller, the workhorse of robotics, and explore the nuances of controlling actuators in the real world.

### 🧠 Theory: The Physics of Control
A motion planner tells the robot *where* to be. The controller ensures it *gets there* and *stays there*.

#### 1. The Control Loop
1.  **Reference (Set Point):** Where we want to be (e.g., Joint angle = 90°).
2.  **Error:** Difference between Reference and Current State.
3.  **Control Effort:** The command sent to the actuator (e.g., Voltage, Current, Torque).

#### 2. PID Control
*   **Proportional (Kp):** Spring-like force. Pulls toward the target. High Kp = stiff, fast, but overshoots.
*   **Integral (Ki):** Memory of past errors. Pushes harder if the error persists (e.g., holding up an arm against gravity). Fixes steady-state error.
*   **Derivative (Kd):** Damper/Friction. Resists change. Slows down the movement to prevent overshoot.

#### 3. Actuators
*   **DC Motors:** Common, easy to control.
*   **BLDC (Brushless):** High efficiency, high torque, used in high-end humanoids (Unitree, Tesla Optimus).
*   **Series Elastic Actuators (SEA):** Include a spring for compliance and shock absorption.

### 🛠️ Architecture: The ros2_control Stack

```mermaid
graph TD
    A[MoveIt / Trajectory Controller] -->|/joint_trajectory| B(JointTrajectoryController)
    B -->|Command (Position/Velocity/Effort)| C[Hardware Interface (SystemInterface)]
    C -->|Driver API (CAN/EtherCAT)| D[Physical Motor Driver]
    D -->|Encoder Feedback| C
    C -->|Joint States| B
    B -->|/joint_states| A
```

### 💻 Implementation: Custom PID Controller Node
While `ros2_control` is the standard, writing a simple PID loop demystifies the magic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SimplePID(Node):
    def __init__(self):
        super().__init__('simple_pid')
        self.target_sub = self.create_subscription(Float64, '/joint/target', self.target_cb, 10)
        self.state_sub = self.create_subscription(Float64, '/joint/state', self.state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64, '/joint/cmd_voltage', 10)
        
        self.kp = 10.0
        self.ki = 0.1
        self.kd = 1.0
        
        self.target = 0.0
        self.current = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.timer = self.create_timer(0.01, self.loop) # 100Hz

    def target_cb(self, msg): self.target = msg.data
    def state_cb(self, msg): self.current = msg.data

    def loop(self):
        error = self.target - self.current
        dt = 0.01
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Clamp output to physical limits (e.g., +/- 12V)
        output = max(-12.0, min(12.0, output))
        
        msg = Float64()
        msg.data = output
        self.cmd_pub.publish(msg)
        
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = SimplePID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ⚠️ Common Pitfalls (Sim vs. Real)

*   **Simulation:**
    *   **Ideal Actuators:** Motors produce exact torque instantly.
    *   **No Friction:** Joints move effortlessly.
    *   **Rigid Body:** No flexing or vibration.
*   **Reality:**
    *   **Stiction (Static Friction):** The joint won't move until a certain voltage threshold is crossed. This causes "stick-slip" motion (jerky movement) at low speeds.
    *   **Gravity:** Holding an arm out requires constant torque. A pure P-controller will sag (Steady-state error). You need an **I-term** or, better yet, **Feedforward Gravity Compensation**.
    *   **Noise:** Derivative terms amplify sensor noise. A noisy encoder can make the motors hum or vibrate violently if Kd is too high.
    *   **Delay:** Communication latency (USB/CAN) limits the max stable Gains.

*   **Fix:**
    *   **Gravity Compensation:** Add a term to the output based on the physics model: `Output = PID + G(theta)`.
    *   **Deadband Compensation:** Add a minimum voltage offset to overcome stiction.
    *   **Low-Pass Filter:** Filter the velocity signal before calculating the D-term to reduce noise.

### 🧪 Verification
1.  **Step Response:** Command the joint to move from 0° to 10° instantly.
    *   **Over-damped:** Moves slowly, no overshoot.
    *   **Under-damped:** Overshoots, oscillates, then settles.
    *   **Critically damped:** Fastest rise time without overshoot.
2.  **Disturbance Rejection:** Push the robot arm. A good controller acts like a spring, pushing back to restore the position.
3.  **Steady-State Error:** Hold the arm horizontally. Does it droop? If so, increase Ki or add gravity compensation.

### 📝 Chapter Summary
*   **Control Theory** ensures physical motors faithfully execute software commands.
*   **PID Controllers** use Error (P), History (I), and Predicted Future (D) to calculate control signals.
*   **ros2_control** is the modular framework for abstracting hardware (drivers) from software (trajectory controllers).
*   **Real-world actuators** suffer from friction, gravity, and noise, requiring techniques like Gravity Compensation and Filtering to achieve smooth motion.

### 🔚 Conclusion
Code moves atoms, but Control Theory dictates *how gracefully* they move. A poorly tuned controller can turn an expensive humanoid robot into a flailing hazard. While simulation provides a safe sandbox for tuning gains, the final validation must always occur on hardware, where friction and gravity are the ultimate arbiters of success. With precise control established, we can now look outward to navigation.
