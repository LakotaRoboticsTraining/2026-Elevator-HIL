# FRC Elevator Simulation & Live Tuning Guide

This guide details where to find public FRC training repositories, along with complete implementation patterns for **NT4 telemetry**, an **Xbox Controller**, and **live Glass sliders/inputs** for student experimentation.

---

## 1. Top Public FRC Training & Simulation Repositories

Several FRC teams maintain open-source training repositories specifically built for teaching students mechanism simulation and live tuning:

| Repository / Organization | Highlights |
| :--- | :--- |
| **[FRC1360/ElevatorSimTraining](https://github.com/FRC1360/ElevatorSimTraining)** *(Team 1360 Orbit Robotics)* | A dedicated elevator simulation training project designed to onboard new software students to `ElevatorSim`, motion profiles, and tuning. |
| **[Iron-Panthers/FRC-2023-Programming-Training-Elevator](https://github.com/Iron-Panthers/FRC-2023-Programming-Training-Elevator)** *(Team 5012 / 5026)* | Curriculum repository teaching elevator programming, soft limits, state control, and unit testing in simulation. |
| **[Mechanical-Advantage/AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit)** *(Team 6328)* | The gold standard for NT4 telemetry, simulation, and live tuning. Check their example projects and `LoggedTunableNumber` pattern for Glass / AdvantageScope live sliders. |
| **[wpilibsuite/allwpilib Examples](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples)** *(Official WPILib)* | Look at `elevatorsimulation` (physics + `Mechanism2d`) and `elevatorprofiledpid` (Command-based `ProfiledPIDSubsystem`). |
| **[Automated-Amphibians/YAMS-Learning](https://github.com/Automated-Amphibians/YAMS-Learning)** | A modular mechanism training repo featuring elevators, arms, and turrets with built-in telemetry and physics simulation. |

---

## 2. How to Create Live Sliders & Inputs in Glass

There are two primary ways to give students live tuning sliders inside Glass:

### Approach A: The Native WPILib `Sendable` Widget (Instant PID Sliders)
If you publish your `ProfiledPIDController` using `SmartDashboard.putData()`, **Glass automatically builds a live tuning card** with editable text inputs and step buttons for $k_P$, $k_I$, $k_D$, and the setpoint goal:

```java
// Inside Elevator.java constructor
SmartDashboard.putData("Elevator PID", m_controller);
```
In Glass: open **NetworkTables $\to$ SmartDashboard $\to$ Elevator PID**. Students can drag or type new PID values live without restarting simulation.

---

### Approach B: The `TunableNumber` Pattern (For $k_S$, $k_G$, $v_{\max}$, $a_{\max}$, and Heights)
Controllers like `ElevatorFeedforward` and `TrapezoidProfile.Constraints` don't implement `Sendable` directly. FRC teams (like Team 6328 Mechanical Advantage and Team 254) use a lightweight **`TunableNumber`** helper that publishes default values to NT4 and updates when students edit them in Glass.

Here is a simple, reusable helper for your students:

```java
package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableNumber {
  private final DoubleEntry m_entry;
  private double m_lastValue;

  public TunableNumber(String tableKey, double defaultValue) {
    m_entry = NetworkTableInstance.getDefault()
        .getTable("Tuning")
        .getDoubleTopic(tableKey)
        .getEntry(defaultValue);
    m_entry.setDefault(defaultValue);
    m_lastValue = defaultValue;
  }

  public double get() {
    return m_entry.get();
  }

  public boolean hasChanged() {
    double current = m_entry.get();
    if (current != m_lastValue) {
      m_lastValue = current;
      return true;
    }
    return false;
  }
}
```

In Glass, students open the **Tuning** table, right-click any value (e.g., `Tuning/kG` or `Tuning/MaxVelocity`), and select **Display As $\to$ Slider** to scrub gains live!

---

## 3. Complete Architecture Blueprint for This Repo

You can upgrade this repository to give students the full experience:
1. **`CommandXboxController`**: A, B, X, Y buttons for preset positions; Left Y-stick for analog manual override.
2. **NT4 Telemetry**: Publishes position, velocity, motor voltage, current draw, position error, and `Mechanism2d`.
3. **Live Tuning Inputs**: Exposes $k_P$, $k_G$, $k_V$, $v_{\max}$, and $a_{\max}$ to Glass.

### A. Updated `Elevator.java` (Telemetry + Live Tuning)

```java
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Elevator implements AutoCloseable {
  private final DCMotor m_gearbox = DCMotor.getVex775Pro(4);

  // Live tunable gains for Glass
  private final TunableNumber m_tuneKp = new TunableNumber("kP", Constants.kElevatorKp);
  private final TunableNumber m_tuneKg = new TunableNumber("kG", Constants.kElevatorkG);
  private final TunableNumber m_tuneKv = new TunableNumber("kV", Constants.kElevatorkV);
  private final TunableNumber m_tuneMaxVel = new TunableNumber("MaxVelocity", 2.45);
  private final TunableNumber m_tuneMaxAcc = new TunableNumber("MaxAcceleration", 2.45);

  private ProfiledPIDController m_controller =
      new ProfiledPIDController(Constants.kElevatorKp, 0, 0, new TrapezoidProfile.Constraints(2.45, 2.45));
  private ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(0, Constants.kElevatorkG, Constants.kElevatorkV, 0);

  private final Encoder m_encoder = new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // Simulation models
  private final ElevatorSim m_sim = new ElevatorSim(
      m_gearbox, Constants.kElevatorGearing, Constants.kCarriageMass,
      Constants.kElevatorDrumRadius, Constants.kMinElevatorHeightMeters,
      Constants.kMaxElevatorHeightMeters, true, 0, 0.01, 0.0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Visualization
  private final Mechanism2d m_mech = new Mechanism2d(20, 50);
  private final MechanismLigament2d m_elevatorLigament =
      m_mech.getRoot("Root", 10, 0).append(new MechanismLigament2d("Elevator", 0, 90));

  public Elevator() {
    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    SmartDashboard.putData("Elevator Sim", m_mech);
    SmartDashboard.putData("Elevator PID Controller", m_controller);
  }

  public void reachGoal(double goalMeters) {
    checkTunableUpdates();

    // Clamp goal within soft limits
    goalMeters = Math.max(Constants.kMinElevatorHeightMeters, 
                          Math.min(goalMeters, Constants.kMaxElevatorHeightMeters));
    m_controller.setGoal(goalMeters);

    double currentPos = m_encoder.getDistance();
    double pidOut = m_controller.calculate(currentPos);
    double ffOut = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOut + ffOut);
  }

  public void setManualVoltage(double volts) {
    // Direct manual override
    m_motor.setVoltage(volts);
  }

  public void stop() {
    m_motor.set(0.0);
  }

  private void checkTunableUpdates() {
    if (m_tuneKp.hasChanged()) {
      m_controller.setP(m_tuneKp.get());
    }
    if (m_tuneKg.hasChanged() || m_tuneKv.hasChanged()) {
      m_feedforward = new ElevatorFeedforward(0, m_tuneKg.get(), m_tuneKv.get(), 0);
    }
    if (m_tuneMaxVel.hasChanged() || m_tuneMaxAcc.hasChanged()) {
      m_controller.setConstraints(new TrapezoidProfile.Constraints(m_tuneMaxVel.get(), m_tuneMaxAcc.get()));
    }
  }

  public void updateTelemetry() {
    double currentPos = m_encoder.getDistance();
    double targetPos = m_controller.getGoal().position;

    // Send all pertinent variables to NT4
    SmartDashboard.putNumber("Elevator/PositionMeters", currentPos);
    SmartDashboard.putNumber("Elevator/TargetMeters", targetPos);
    SmartDashboard.putNumber("Elevator/PositionErrorMeters", targetPos - currentPos);
    SmartDashboard.putNumber("Elevator/VelocitySetpoint", m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("Elevator/AppliedVolts", m_motor.get() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Elevator/CurrentDrawAmps", m_sim.getCurrentDrawAmps());
    SmartDashboard.putBoolean("Elevator/AtSetpoint", Math.abs(targetPos - currentPos) < 0.02);

    m_elevatorLigament.setLength(currentPos);
  }

  public void simulationPeriodic() {
    m_sim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
    m_sim.update(0.020);
    m_encoderSim.setDistance(m_sim.getPositionMeters());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech.close();
  }
}
```

---

### B. Updated `Robot.java` (CommandXboxController Setup)

```java
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
  // Use CommandXboxController for button events and axes
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final Elevator m_elevator = new Elevator();

  // Preset setpoints in meters
  private static final double kStowHeight = 0.0;
  private static final double kLowHeight = 0.35;
  private static final double kMidHeight = 0.75;
  private static final double kHighHeight = 1.15;

  private double m_activeGoal = kStowHeight;

  @Override
  public void robotPeriodic() {
    m_elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_elevator.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    // 1. Check Setpoint Buttons (A, B, X, Y)
    if (m_controller.a().getAsBoolean()) {
      m_activeGoal = kStowHeight;
    } else if (m_controller.b().getAsBoolean()) {
      m_activeGoal = kLowHeight;
    } else if (m_controller.x().getAsBoolean()) {
      m_activeGoal = kMidHeight;
    } else if (m_controller.y().getAsBoolean()) {
      m_activeGoal = kHighHeight;
    }

    // 2. Check Analog Left-Y Stick for Manual Override (with deadband)
    double manualInput = -MathUtil.applyDeadband(m_controller.getLeftY(), 0.08);

    if (Math.abs(manualInput) > 0.0) {
      // Manual joystick control: scale to ±12V
      m_elevator.setManualVoltage(manualInput * 12.0);
    } else {
      // Profiled closed-loop target control
      m_elevator.reachGoal(m_activeGoal);
    }
  }

  @Override
  public void disabledInit() {
    m_elevator.stop();
  }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
```

---

## 4. How Students Can Run and Experiment With This in Glass
1. Press `F5` in VS Code (**WPILib: Simulate Robot Code on Desktop**).
2. Check the box for **Sim GUI** and launch **Glass** from the top menu (`Tools -> Glass`).
3. Under **NetworkTables**:
   * Open **`SmartDashboard / Elevator Sim`** to view the live 2D moving elevator ligament.
   * Open **`SmartDashboard / Elevator`** to see graphs of position error, current draw, and applied voltage.
   * Open **`Tuning`** and right-click on `kG`, `kP`, or `MaxVelocity` $\to$ **Display As $\to$ Slider** to adjust numbers and watch how the elevator responds to joystick commands in real time.

