## Table of Contents
1. [Shooting Mechanism](#1-robot-shooting-mechanism-and-motor-selection)
    - [Introduction](#1-introduction)
    - [Shooting Mechanism](#2-shooting-mechanism)
    - [Motor Selection](#3-motor-selection)
    - [Comparisons of Different Approaches](#4-comparisons-of-different-approaches)
2. [Driblling Mechanism] (#2. Robot      


---

# 1. Robot Shooting Mechanism and Motor Selection

## 1. Introduction

This document explains the working mechanism of the shooting functionality in the ABU robot, focusing on the selection of motors for optimal performance. It includes calculations of force, torque, and motor power requirements, and compares different approaches to achieving precise and efficient shooting.

## 2. Shooting Mechanism

   - **Flywheel Shooter** :  Two spinning wheels with rope in between to launch the projectile which is the ball

        - ### Case Study 1: Shooting Mechanism with an 8.5 Meter Range ( longest possible range )

            - **Wheel Speed calc :** using the projectile formula to calculate the intial velocity that the ball needs to reach the bascket
            $$y = x \tan(\alpha) - \frac{1}{2} g \left( \frac{x}{v \cdot \cos(\alpha)} \right)^2$$

            - To convert angular velocity to revolutions per minute (RPM):

            $$\text{RPM} = \frac{\omega \times 60}{2 \pi}$$

            - **Projectile energy :** 
            $$KE = \frac{1}{2} m v^2$$

            - **Wheel Material:** Rotational Kinetic Energy of the wheel .
                    $$E = \frac{1}{2} I \omega^2$$

            #### Results

            After testing, the needed motor should has an output RPM = 640 and power = 500W which is a high power so if we set a max power of 150W we can reverse engineer that case as following


        ### Case Study 2: 
        Reverse Engineering with a 150W Motor ( if we consider it as the max power can be provided bythe system ) to analyzing the performance impact on the wheel mechanism and range of shooting.

        1. **Speed :** calculate the max speed that motor is able to shoot the ball with 

        2. **Range :** calculate the max range of shooting based on max velocity


        #### Results
        - **less range:** 
        $$range = 3.44 m$$


        ### Detailed calculations in these papers
        <div style="display: flex;">
            <img src="WhatsApp Image 2024-09-11 at 05.27.52_f7fdb899-1.jpg" style="width: 30%; height: auto; margin-right:50px; margin-left:20%;">
            <img src="WhatsApp Image 2024-09-11 at 05.28.06_f2e717c4.jpg" style="width: 30%; height: auto;">
        </div>

## 3. Motor Selection

### Possible Motor Types to use 
- **DC Motor**: cheap ,and high torque compared to size motor
- **Brushless Motors**: Highly efficient for high-speed, long-duration tasks.


### Selection Criteria
1. **Power Requirements**:
   - Motor power is calculated based on the the desired shooting speed 500 W and 150 W for low range
   - BLDC is more energy efficient so it wins this point

2. **Speed and Torque**:
    $$P = \tau \omega$$
    - We can adjust the speed and torque based on power by this formula in our case we need RPM = 640 for the first case and 440 RPM for second case .
    - since theres no need for high speed in this task then no need for BLDC


3. **cost**:
    - this point for the DC motor

4. **Weight and Size**:
   - The motor’s weight and dimensions must fit within the robot’s design, without causing balance issues.

### conclusion : DC motor is more suitable for this task 


## 4. Comparisons of Different Approaches


| power       | range    | Speed  |recommeded motor   |
|-------------|----------|--------|-------------------|
| 500 W       | 8.5 m    | Fast   | [500W DC-motor1]( https://www.ebay.com/itm/134043091709?chn=ps&var=433488060086&_trkparms=ispr%3D1&amdata=enc%3A1gYBR_8KkQv6R-8qDjRVb5g44&norover=1&mkevt=1&mkrid=711-167653-786486-8&mkcid=2&itemid=433488060086_134043091709&targetid=325425753764&device=c&mktype=pla&googleloc=9112344&poi=&campaignid=20784063734&mkgroupid=158211146320&rlsatarget=pla-325425753764&abcId=&merchantid=101696517&gad_source=1&gclid=CjwKCAjw3P-2BhAEEiwA3yPhwCujOd31pH74_cCd1j18fxLIUe8kLhMUDDMMaiR_9kcwKhEkcJzhIRoCBzsQAvD_BwE ) | 
| 150 W       | 3.87 m   | Fast   | [150W DC-motor1](https://www.ebay.com/itm/126537966517?chn=ps&_trkparms=ispr%3D1&amdata=enc%3A1Uvq5_iUuS6u5cfC59VoLNA38&norover=1&mkevt=1&mkrid=711-167653-786486-8&mkcid=2&itemid=126537966517&targetid=325425753764&device=c&mktype=pla&googleloc=9112344&poi=&campaignid=20784063734&mkgroupid=158211146320&rlsatarget=pla-325425753764&abcId=&merchantid=101695362&gad_source=1&gclid=CjwKCAjw3P-2BhAEEiwA3yPhwNmFAmhhyG91O_4upMU8QU9JQIsDPmN14HmfBsZ8Q7St1lAtIV_isBoCAlkQAvD_BwE)
||||[150W DC-motor2](https://www.brushless.com/12v-150w-brushless-dc-motor)  | 




# 2. Robot Driblling Mechanism and Motor Selection


  the working mechanism of the driblling functionality in the ABU robot, focusing on the selection of motors for optimal performance. It includes calculations of force, torque, and motor power requirements .

  ## 2. Driblling Mechanism
    ## mechanism 1

    https://github.com/Hamody229/MIAturtlebot-/blob/Task-12_2/mech%201.1.jpg
    
    

  


