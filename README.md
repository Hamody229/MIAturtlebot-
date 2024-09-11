Here is the updated README file including the movement section and motor selection details:

---

## Table of Contents
1. [Shooting Mechanism](#1-robot-shooting-mechanism-and-motor-selection)
    - [Introduction](#1-introduction)
    - [Shooting Mechanism](#2-shooting-mechanism)
    - [Motor Selection](#3-motor-selection)
    - [Comparisons of Different Approaches](#4-comparisons-of-different-approaches)
2. [Movement](#2-robot-movement-motor-selection)
    - [Minimum Requirements](#1-minimum-requirements)
    - [Calculations](#2-calculations)
    - [Motor Selection](#3-motor-selection)

---

# 1. Robot Shooting Mechanism and Motor Selection

## 1. Introduction

This document explains the working mechanism of the shooting functionality in the ABU robot, focusing on the selection of motors for optimal performance. It includes calculations of force, torque, and motor power requirements, and compares different approaches to achieving precise and efficient shooting.

## 2. Shooting Mechanism

   - **Flywheel Shooter** : Two spinning wheels with rope in between to launch the projectile which is the ball.

        ### Case Study 1: Shooting Mechanism with an 8.5 Meter Range (longest possible range)

        - **Wheel Speed Calculation:** Using the projectile formula to calculate the initial velocity that the ball needs to reach the basket:
        $$y = x \tan(\alpha) - \frac{1}{2} g \left( \frac{x}{v \cdot \cos(\alpha)} \right)^2$$

        - To convert angular velocity to revolutions per minute (RPM):
        $$\text{RPM} = \frac{\omega \times 60}{2 \pi}$$

        - **Projectile Energy:**
        $$KE = \frac{1}{2} m v^2$$

        - **Wheel Material (Rotational Kinetic Energy of the Wheel):**
        $$E = \frac{1}{2} I \omega^2$$

        #### Results
        After testing, the needed motor should have an output of RPM = 640 and power = 500W, which is too high. By setting a max power of 150W, we reverse engineer the case:

        ### Case Study 2: Reverse Engineering with a 150W Motor
        Analyzing the performance impact on the wheel mechanism and shooting range.

        1. **Speed**: Calculate the max speed the motor can shoot the ball.
        2. **Range**: Calculate the max shooting range based on max velocity.

        #### Results
        - **Less range**: 
        $$range = 3.44 m$$

        ### Detailed Calculations in These Papers
        <div style="display: flex;">
            <img src="WhatsApp Image 2024-09-11 at 05.27.52_f7fdb899-1.jpg" style="width: 30%; height: auto; margin-right:50px; margin-left:20%;">
            <img src="WhatsApp Image 2024-09-11 at 05.28.06_f2e717c4.jpg" style="width: 30%; height: auto;">
        </div>

## 3. Motor Selection

### Possible Motor Types
- **DC Motor**: Cheap and high torque compared to size.
- **Brushless Motors**: Highly efficient for high-speed, long-duration tasks.

### Selection Criteria
1. **Power Requirements**:
   - Motor power is calculated based on the desired shooting speed: 500W for high range and 150W for low range.
   - BLDC motors are more energy efficient.
2. **Speed and Torque**:
    $$P = \tau \omega$$
    - We need RPM = 640 for the first case and 440 RPM for the second.
3. **Cost**: DC motor is more economical.
4. **Weight and Size**: Must fit the robot’s design.

### Conclusion
DC motor is more suitable for this task.

## 4. Comparisons of Different Approaches

| power  | range   | Speed | Recommended motor  |
|--------|---------|-------|--------------------|
| 500 W  | 8.5 m   | Fast  | [500W DC-motor1](https://www.ebay.com/itm/134043091709) | 
| 150 W  | 3.87 m  | Fast  | [150W DC-motor1](https://www.ebay.com/itm/126537966517) | 
|||       | [150W DC-motor2](https://www.brushless.com/12v-150w-brushless-dc-motor) |

---

# 2. Robot Movement Motor Selection

## 1. Minimum Requirements
* Torque: 3.8259 Kg/cm
* RPM: 358.0986 rev/min
* Power: 143.47125 W
* Type: Brushless DC motor
* Voltage: Less than 24V

How did we calculate these numbers?

## 2. Calculations

### Weight Assumption
- Assumed robot weight: **30 Kg** (50 Kg total for two robots + safety margin).

### Friction Force
- Friction force = μ × F_normal
- Coefficient of friction: 0.3
- Friction force: F = 0.3 × (30 × 9.81) = 88.29 N

### Torque
- τ = (F_normal + F_friction) × r
- Wheel radius: 7 cm
- Total torque: 26.7813 Nm
- For 4 wheels, each motor needs: 6.695325 Nm (68.27 Kg/cm)

### Speed
- To cross 15m in **5 seconds**: Linear speed = 3 m/s
- RPM: 409.2556 rev/min

### Power
- Angular velocity: ω = 42.8571 rad/s
- Power: P = 286.9425 W

### Conclusion
* Torque = 68.2733145365626 Kg/cm or 6695.325 milli Nm
* RPM = 409.255567951 rev/min
* Power = 286.9425 watt
* Type: brushless DC motor
* Voltage supply must be less than 24V


The 30 kg weight assumption led to a high power requirement of 286.94 watts, making motor selection difficult, finding a motor that met requirements at this power level, while staying within the 24V limit, posed a significant challenge. To address this, we revised the weight to 15 kg, reducing power demands and easing motor selection.

### New Assumptions
* Robot weight = 15Kg
* Wheel radius is now 8cm (for better RPM)

### Updated Calculations:
* Friction Force: 44.145 N
* Torque per motor: 3.8259 Nm (39.01 Kg/cm)
* RPM: 358.0986 rev/min
* Power: 143.47125 W

### Final requirements
* Torque: 3.8259 Nm
* RPM: 358.0986 rev/min
* Power: 143.47125 W
* Voltage: Less than 24V

## 3. Motor Selection

### Selected Motor: **30ECT64 Ultra EC** (18V variant)
- [Datasheet](https://www.portescap.com/-/media/project/automation-specialty/portescap/portescap/pdf/specification-pdfs/specifications_30ect64.pdf)
- [Store](https://shop.portescap.com/shop/brushless-dc-motors/brushless-slotless/30ect64-10b-4)
- ![30ECT64](https://shop.portescap.com/-/media/images/portescap_catalog/product_bldc_ultra_ec_30ect64.png)

* **Specifications**:
  - 18V supply
  - Max power: 187 watts
  - Max torque: 137 mNm
  - Max speed: 28550 RPM


* Here the RPM is high, but the torque is not enough, to fix this we can apply a 50:1 gear ratio to the motor
* This gives us 571 RPM speed, and 6.85 Nm torque

### Other Considered Motors
- **30ECT90 Ultra EC**: Not selected due to its 24V nominal voltage. Which is too close for comfort. The limit required by the robocon 25 rules is 24V.

- **22ECS45 Ultra EC**: Not enough power (120W max).
