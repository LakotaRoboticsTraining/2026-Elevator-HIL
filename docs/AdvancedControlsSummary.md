# WPILib Control System Basics & Tuning Guide

---

## 1. Advanced Controls Introduction

Use the WPILIB link below for a deep dive into these subjects.

[Link to WPILIB Detailed Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/index.html)

### Control System Basics
A control system manages, commands, directs, or regulates the behavior of other devices or systems using control loops.

* **Open-Loop Control:** Sends a command to an actuator (e.g., motor voltage) without measuring or reacting to the actual physical outcome.
  * *Example:* Setting a motor to `0.50` (50% output) without checking encoder velocity.
* **Closed-Loop Control:** Uses sensor feedback (encoders, gyros, vision) to continuously calculate error and adjust control output to reach a target setpoint.
  * *Formula:* $\text{Error} = \text{Setpoint} - \text{Process Variable (Measurement)}$

---

### Picking a Control Strategy
Choosing the right combination of Feedforward and Feedback (PID) depends on the physical mechanism being controlled:

| Mechanism Type | Recommended Strategy | Key Considerations |
| :--- | :--- | :--- |
| **Velocity (Flywheels, Shooters)** | Feedforward + Proportional ($k_V + k_A + k_P$) | Main effort comes from Feedforward; $k_P$ corrects small disturbances. |
| **Position (Turrets, Swerve Steering)** | Feedforward + PID ($k_S + k_P + k_D$) | Static friction ($k_S$) overcomes stiction; $k_D$ eliminates overshoot. |
| **Loaded Arms (Single-Jointed)** | Gravity Feedforward + PID ($k_G(\theta) + k_P$) | $k_G$ varies continuously with the cosine of the arm angle. |
| **Elevators (Vertical Lift)** | Gravity Feedforward + Motion Profile + PID | Constant $k_G$ balances gravity; Trapezoid Profiles limit acceleration. |

---

### Introduction to DC Motor Feedforward
Feedforward calculates the necessary voltage to achieve a target motion based on the physical model of the DC motor and mechanism.

* **Static Friction Gain ($k_S$):** The minimum voltage required to overcome static friction and initiate motion.
* **Velocity Gain ($k_V$):** The voltage required to maintain a given steady-state velocity ($V / \text{rad/s}$).
* **Acceleration Gain ($k_A$):** The voltage required to produce a given acceleration ($V / \text{rad/s}^2$).
* **Gravity Gain ($k_G$):** The voltage required to hold a vertical mechanism static against gravity.

$$\text{Voltage}_{\text{FF}} = k_S \cdot \text{sign}(\text{velocity}) + k_V \cdot \text{velocity} + k_A \cdot \text{acceleration} + k_G$$

---

### Introduction to PID

A **PID Controller** calculates a corrective voltage output based on three terms:

1. **Proportional ($k_P$):** Produces an output proportional to the current error.
   $$\text{Output}_P = k_P \times \text{Error}$$
2. **Integral ($k_I$):** Accumulates past error over time to eliminate steady-state offset (use sparingly in FRC due to windup).
   $$\text{Output}_I = k_I \times \int \text{Error} \, dt$$
3. **Derivative ($k_D$):** Responds to the rate of change of error, acting as a dynamic brake to prevent overshoot and dampen oscillations.
   $$\text{Output}_D = k_D \times \frac{d(\text{Error})}{dt}$$

---

### PID Introduction Video by WPI
* **Key Takeaway:** PID feedback should not do all the heavy lifting. Combining a solid **Feedforward** model with a light **PID** feedback loop produces faster response times, minimal overshoot, and lower battery drain.
[PID Intro Video](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/pid-video.html)

---

## 2. Controls Tuning Tutorials

### Tuning a Flywheel Velocity Controller
1. **Determine Feedforward Gains ($k_V$, $k_A$):** Use SysId or approximate manually ($k_V \approx \frac{12\,\text{V}}{\text{Max RPM}}$).
2. **Apply Feedforward:** Verify the flywheel gets close to the setpoint target velocity on feedforward alone.
3. **Add Proportional ($k_P$):** Increase $k_P$ until the velocity quickly recovers when a game piece is fed into the rollers.
4. **Avoid $k_D$ and $k_I$:** High sensor noise on fast flywheels usually makes $k_D$ noisy; $k_V$ handles steady-state accuracy without $k_I$.

---

### Tuning a Turret Position Controller
1. **Determine $k_S$:** Increase voltage slowly until the turret barely begins to rotate.
2. **Increase $k_P$:** Raise $k_P$ until the turret rotates quickly to target position setpoints.
3. **Add $k_D$:** If the turret overshoots or oscillates around the target angle, increase $k_D$ until the motion settles smoothly without ringing.

---

### Tuning a Vertical Arm Position Controller
1. **Determine $k_G$:** Measure the voltage required to hold the arm horizontal ($\theta = 0^\circ$).
2. **Apply Gravity Compensation:** Ensure the arm holds position across its range of motion with $k_G \cdot \cos(\theta)$.
3. **Tune $k_P$ and $k_D$:** Increase $k_P$ for responsiveness; add $k_D$ to smooth out stopping behavior and prevent mechanical shock.

---

### Tuning a Vertical Elevator with Motion Profiling
1. **Calculate Constant $k_G$:** Determine the hold voltage required to prevent the elevator carriage from dropping under its own weight.
2. **Set Trapezoidal Motion Profile Limits:** Define Maximum Velocity ($v_{\max}$) and Maximum Acceleration ($a_{\max}$) based on mechanism tolerances.
3. **Tune PID ($k_P$, $k_D$):** Use $k_P$ to ensure the elevator tracks the profile setpoints accurately throughout the lift path.

---

## 3. Common Control Loop Tuning Issues

* **Oscillation / Instability:** $k_P$ is too high or sensor latency is significant. Reduce $k_P$ or add $k_D$ to increase damping.
* **Mechanical Jitter:** $k_D$ is too high or encoder measurements are noisy. Apply a low-pass filter to velocity measurements or reduce $k_D$.
* **Integral Windup:** $k_I$ accumulates massive error while the mechanism is physically jammed or saturated at max voltage. Cap $k_I$ range using `setIntegratorRange()`.
* **Asymmetrical Behavior (Up vs. Down):** Missing or incorrect $k_G$ (gravity compensation) on vertical lifts or arms.