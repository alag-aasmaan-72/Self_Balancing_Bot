

## Self Balancing Robot Using PID Controller

A self-balancing two-wheeled robot developed using **PID control** to automatically maintain its upright position.

---

## 📌 Project Overview

BALANCER_BOT is a self-balancing robot that uses an **MPU6050 IMU sensor** to measure its tilt angle and multiple PID controllers to generate corrective motor commands.

The robot works as a **closed-loop feedback control system**. It continuously measures its current state, calculates the error from the desired state, applies PID control, and adjusts the motors to maintain balance.

The project focuses on:

- Robotics
- Control Systems
- Embedded Programming
- PID Control
- Sensor Data Processing
- Motor Control
- Encoder Feedback

---

## ⚙️ Working Principle

The robot continuously performs the following operations:

1. Measure the tilt angle using the **MPU6050 IMU sensor**.
2. Compare the measured angle with the desired angle.
3. Calculate the error.
4. Apply PID control.
5. Convert the PID output into PWM signals.
6. Drive the motors to restore balance.
7. Continuously receive sensor feedback and repeat the process.

### Error Calculation

```text
Error = Desired Angle - Measured Angle
```

For a balanced robot:

```text
Desired Angle = 0°
```

The MPU6050 continuously provides the measured angle of the robot. The difference between the desired angle and measured angle is used as the input to the PID controller.

---

## 🧠 PID Control

The robot uses multiple PID control operations for different purposes.

### 1. Angle & Speed PID

The **Angle and Speed PID** controllers are responsible for balancing the robot.

They control the motor response according to the robot's tilt and speed.

### 2. Position PID

The **Position PID** is responsible for maintaining the desired position of the robot.

### 3. Yaw PID

The **Yaw PID** helps prevent unnecessary rotation and improves directional stability.

### PID Control Architecture

```text
                    PID CONTROL
                         |
          +--------------+--------------+
          |              |              |
          v              v              v
    ANGLE & SPEED     POSITION          YAW
         PID             PID             PID
          |              |              |
          v              v              v
       BALANCE       POSITION HOLD   ROTATION CONTROL
```

---

## 📐 PID Controller Formula

The PID controller consists of three terms:

### Proportional Term

```text
P = Kp × Error
```

The proportional term responds to the current error.

### Integral Term

```text
I = Ki × ∫ Error dt
```

The integral term considers the accumulated error over time.

### Derivative Term

```text
D = Kd × d(Error)/dt
```

The derivative term responds to the rate of change of the error.

### Complete PID Output

```text
PID Output = Kp × Error
           + Ki × ∫ Error dt
           + Kd × d(Error)/dt
```

Proper tuning of PID parameters is important for achieving:

- Fast response
- Stability
- Accurate balancing
- Reduced oscillation

---

# 🔄 Control Flow

```text
                         START
                           |
                           v
                +---------------------+
                | Initialize System   |
                |                     |
                | Arduino Nano        |
                | MPU6050             |
                | Motors              |
                | Encoders            |
                +----------+----------+
                           |
                           v
                  Read MPU6050 Data
                           |
                           v
                  Calculate Tilt Angle
                           |
                           v
                    Calculate Error
                           |
                           v
                  Run PID Controllers
                           |
                           v
                  Calculate Correction
                           |
                           v
                    Generate PWM
                           |
                           v
                     Drive Motors
                           |
                           v
                   Robot Corrects
                       Its Tilt
                           |
                           v
                    Sensor Feedback
                           |
                           +-------------> REPEAT
```

---

# 🏗️ System Architecture

```text
                       +---------------+
                       |    MPU6050    |
                       |   IMU Sensor  |
                       +-------+-------+
                               |
                               | Sensor Data
                               v
                    +--------------------+
                    |    Arduino Nano    |
                    |     ATmega328P     |
                    +---------+----------+
                              |
                              v
                    +--------------------+
                    |   PID Controller   |
                    |                    |
                    | Angle / Speed PID  |
                    | Position PID       |
                    | Yaw PID            |
                    +---------+----------+
                              |
                              | PID Output
                              v
                    +--------------------+
                    |    PWM Control     |
                    +---------+----------+
                              |
                              v
                    +--------------------+
                    |       L298N        |
                    |   Motor Driver     |
                    +---------+----------+
                              |
                              v
                  +------------------------+
                  |   N20 DC Gear Motors   |
                  |       + Encoders       |
                  +-----------+------------+
                              |
                              v
                       +-------------+
                       |    ROBOT    |
                       |  MOVEMENT   |
                       +------+------+ 
                              |
                              | Feedback
                              v
                           MPU6050
```

---

# 🔁 Closed-Loop Feedback System

The robot operates using a continuous feedback loop.

```text
             Desired State
                  |
                  v
             +---------+
             |  Error  |<-----------------------+
             +----+----+                        |
                  |                             |
                  v                             |
             +---------+                        |
             |   PID   |                        |
             |Controller|                       |
             +----+----+                        |
                  |                             |
                  v                             |
             Control Signal                     |
                  |                             |
                  v                             |
             +---------+                        |
             | Motors  |                        |
             +----+----+                        |
                  |                             |
                  v                             |
             Robot Movement                     |
                  |                             |
                  v                             |
             +---------+                        |
             | MPU6050 |------------------------+
             | Sensor  |
             +---------+
```

---

# 🔧 Hardware Components

| Component | Description |
|-----------|-------------|
| **Arduino Nano** | Main microcontroller based on ATmega328P |
| **MPU6050** | IMU sensor used for measuring tilt/orientation |
| **L298N** | Motor driver |
| **N20 DC Gear Motors** | DC geared motors with encoders |
| **N20 Wheels** | Robot wheels |
| **MG90 Servo Motors** | Servo motors |
| **HC-05 Module** | Bluetooth communication module |
| **Switch** | Power/control switch |
| **3S 30C 2200mAh Battery** | 12V battery supply |
| **Robot Chassis** | 3D-printed chassis designed using SolidWorks |

---

# 💻 Software Components

The software and control components used in the project are:

- PID Control Algorithm
- Sensor Data Processing
- PWM Motor Control
- Encoder Feedback
- Embedded Programming

---

# 🛠️ Technologies Used

## Hardware

```text
Arduino Nano
MPU6050
L298N
N20 DC Gear Motors
N20 Encoders
MG90 Servo Motors
HC-05 Bluetooth Module
3S 30C 2200mAh Battery
```

## Software

```text
PID Control
Sensor Data Processing
PWM Motor Control
Encoder Feedback
Embedded Programming
```
