# Elevator HIL - UML Diagrams

This document contains standard, syntax-verified Mermaid UML diagrams for the **2026-Elevator-HIL** WPILib project.

---

## 1. Class Structure Diagram

```mermaid
classDiagram
    direction TB

    class Main {
        +main(args) void
    }

    class Robot {
        -Joystick m_joystick
        -Elevator m_elevator
        +Robot()
        +robotPeriodic() void
        +simulationPeriodic() void
        +teleopPeriodic() void
        +disabledInit() void
        +close() void
    }

    class Elevator {
        -DCMotor m_elevatorGearbox
        -ProfiledPIDController m_controller
        ~ElevatorFeedforward m_feedforward
        -Encoder m_encoder
        -PWMSparkMax m_motor
        -ElevatorSim m_elevatorSim
        -EncoderSim m_encoderSim
        -PWMSim m_motorSim
        -Mechanism2d m_mech2d
        -MechanismRoot2d m_mech2dRoot
        -MechanismLigament2d m_elevatorMech2d
        +Elevator()
        +simulationPeriodic() void
        +reachGoal(goal) void
        +stop() void
        +updateTelemetry() void
        +close() void
    }

    class Constants {
        <<utility>>
        +int kMotorPort
        +int kEncoderAChannel
        +int kEncoderBChannel
        +int kJoystickPort
        +double kElevatorKp
        +double kElevatorKi
        +double kElevatorKd
        +double kElevatorkS
        +double kElevatorkG
        +double kElevatorkV
        +double kElevatorkA
        +double kElevatorGearing
        +double kElevatorDrumRadius
        +double kCarriageMass
        +double kSetpointMeters
        +double kMinElevatorHeightMeters
        +double kMaxElevatorHeightMeters
        +double kElevatorEncoderDistPerPulse
    }

    class TimedRobot {
        <<WPILib>>
    }

    class AutoCloseable {
        <<interface>>
        +close() void
    }

    class ProfiledPIDController {
        <<WPILib>>
    }

    class ElevatorFeedforward {
        <<WPILib>>
    }

    class ElevatorSim {
        <<WPILib>>
    }

    class PWMSparkMax {
        <<WPILib>>
    }

    class Encoder {
        <<WPILib>>
    }

    class Mechanism2d {
        <<WPILib>>
    }

    Main ..> Robot : launches via startRobot
    Robot --|> TimedRobot : inherits
    Robot *-- Elevator : contains
    Elevator ..|> AutoCloseable : implements
    Robot ..> Constants : references
    Elevator ..> Constants : references

    Elevator *-- ProfiledPIDController : position profile controller
    Elevator *-- ElevatorFeedforward : velocity and gravity feedforward
    Elevator *-- PWMSparkMax : motor actuator
    Elevator *-- Encoder : feedback sensor
    Elevator *-- ElevatorSim : plant physics simulation
    Elevator *-- Mechanism2d : visual telemetry
```

---

## 2. Robot Mode Lifecycle State Machine

```mermaid
stateDiagram-v2
    [*] --> Uninitialized
    Uninitialized --> Disabled : robotInit completed

    state Disabled {
        [*] --> MotorOff
        MotorOff : Elevator stop called
        MotorOff : Motor output set to 0.0V
        MotorOff : Goal reset to 0.0m
    }

    state Teleoperated {
        [*] --> DrivingToZero : Teleop starts

        DrivingToZero --> DrivingToTarget : Trigger pressed
        DrivingToTarget --> DrivingToZero : Trigger released

        state DrivingToTarget {
            TargetState : Goal is 0.75m setpoint
            TargetState : ProfiledPID and Feedforward active
            TargetState : Motor voltage applied
        }

        state DrivingToZero {
            ZeroState : Goal is 0.0m floor
            ZeroState : ProfiledPID and Feedforward active
            ZeroState : Motor voltage applied
        }
    }

    Disabled --> Teleoperated : Driver Station Enable (Teleop)
    Teleoperated --> Disabled : Driver Station Disable or E-Stop
    Disabled --> ShuttingDown : Process termination
    Teleoperated --> ShuttingDown : Process termination
    ShuttingDown --> [*] : close() called
```

---

## 3. Elevator Operational State Machine

```mermaid
stateDiagram-v2
    [*] --> Stopped_Idle : Constructor initialized

    state Stopped_Idle {
        IdleState : Motor output 0.0V
        IdleState : Profile goal 0.0m
    }

    state Active_ClosedLoopControl {
        [*] --> MovingToZero : Default starting goal

        state MovingToTarget {
            MoveTarget : Goal 0.75m (kSetpointMeters)
            MoveTarget : Max velocity 2.45 m per s
            MoveTarget : Max acceleration 2.45 m per s2
            MoveTarget : V = PID(error) + FF(vel)
        }

        state HoldingTarget {
            HoldTarget : Position at 0.75m
            HoldTarget : Target velocity 0.0 m per s
            HoldTarget : Feedforward kG (0.762V) counters gravity
        }

        state MovingToZero {
            MoveZero : Goal 0.0m stowed
            MoveZero : Max velocity 2.45 m per s
            MoveZero : Max acceleration 2.45 m per s2
            MoveZero : V = PID(error) + FF(vel)
        }

        state RestAtZero {
            FloorRest : Position at 0.0m
            FloorRest : Target velocity 0.0 m per s
        }

        MovingToTarget --> HoldingTarget : Height reaches 0.75m
        HoldingTarget --> MovingToZero : Trigger released
        MovingToTarget --> MovingToZero : Trigger released mid-travel

        MovingToZero --> RestAtZero : Height reaches 0.0m
        RestAtZero --> MovingToTarget : Trigger pressed
        MovingToZero --> MovingToTarget : Trigger pressed mid-travel
    }

    Stopped_Idle --> Active_ClosedLoopControl : teleopPeriodic calls reachGoal
    Active_ClosedLoopControl --> Stopped_Idle : disabledInit calls stop
    Stopped_Idle --> Closed : close called
    Closed --> [*]
```

---

## 4. Control Loop & Simulation Sequence Diagram

```mermaid
sequenceDiagram
    autonumber
    actor Driver as Driver Joystick
    participant Robot as Robot (TimedRobot)
    participant Elevator as Elevator Subsystem
    participant Controller as ProfiledPID + FF
    participant Motor as PWMSparkMax
    participant Sim as ElevatorSim (Physics)
    participant NT as SmartDashboard (Mechanism2d)

    Note over Robot,NT: 20ms Periodic Execution

    rect rgb(240, 248, 255)
        Note over Robot,Elevator: robotPeriodic() - Telemetry
        Robot->>Elevator: updateTelemetry()
        Elevator->>NT: MechanismLigament2d.setLength(height)
    end

    rect rgb(255, 250, 240)
        Note over Robot,Elevator: teleopPeriodic() - Control Loop
        Robot->>Driver: getTrigger()
        alt Trigger is Pressed
            Robot->>Elevator: reachGoal(0.75)
        else Trigger is Released
            Robot->>Elevator: reachGoal(0.0)
        end
        Elevator->>Controller: calculate(encoderDistance) + FF
        Controller-->>Elevator: pidOutput + feedforwardOutput
        Elevator->>Motor: setVoltage(totalVoltage)
    end

    rect rgb(240, 255, 240)
        Note over Robot,Sim: simulationPeriodic() - Physics Model
        Robot->>Elevator: simulationPeriodic()
        Elevator->>Sim: setInput(speed * batteryVoltage)
        Elevator->>Sim: update(0.020s)
        Sim-->>Elevator: position, currentDraw
        Elevator->>Elevator: m_encoderSim.setDistance(position)
        Elevator->>Sim: RoboRioSim.setVInVoltage(batterySag)
    end
```
