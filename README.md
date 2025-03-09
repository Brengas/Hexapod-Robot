# Hexapod Robot

This repository contains the design, code, and documentation for a hexapod robot controlled by an Arduino Nano and 18 servo motors. The project is currently a work in progress, with plans to upgrade to an ESP32 for enhanced control and connectivity.

### [YouTube Demonstration](https://www.youtube.com/shorts/8HReUtSeT0g)


## Project Overview
- **Objective**: To design and build a fully functional hexapod robot capable of walking and performing basic movement funcionalities, aditionally, this robot is ment to be driven by an ESP32 camera which will allow the robot to move through the enviroment, avoiding obstacles and functionalities like those.  
- **Technologies Used**: Arduino Nano, PCA9685 servo drivers, 18 servo motors, Fusion 360, 3D printing.  
- **Features**:  
  - 18 servo motors for movement across 6 legs.  
  - PCA9685 drivers for precise servo control.  
  - Custom-designed and 3D-printed mechanical components.
 
## CAD Design
<p align="center">
<img src="https://raw.githubusercontent.com/Brengas/Hexapod-Robot/main/images/hexapod.jpg" height="70%" width="70%" alt="CAD design Fusion 360"/>
<img src="https://raw.githubusercontent.com/Brengas/Hexapod-Robot/main/images/hexapod.jpg" height="70%" width="70%" alt="Robot in Ready-to-walk position"/>

## Current Status
- The robot’s mechanical structure has been fully assembled, and basic servo control has been implemented.  
- Currently addressing power supply and battery issues to ensure stable operation.  
- Future plans include upgrading to an ESP32 for enhanced control and adding features like remote control and autonomous navigation.  

## How It Works
1. The Arduino Nano sends control signals to the PCA9685 drivers, which manage the 18 servo motors.  
2. The robot’s legs are designed to move in a coordinated manner to achieve walking motion through inverse kinematics.  
3. The 3D-printed components are lightweight and durable, ensuring efficient movement.  

## Challenges
- **Power Supply**: Currently troubleshooting power supply and battery issues to ensure stable operation.  
- **Control Algorithm**: Developing a robust control algorithm for smooth and coordinated movement.  

## Future Improvements
- Upgrade to an ESP32 for enhanced control and connectivity.  
- Implement remote control via Bluetooth or Wi-Fi.  
- Add sensors for obstacle detection and autonomous navigation.
