# Maze_Solver_RoboCup_2024
The robot navigates a maze-like disaster environment using DFS, detects letter and color victims via Raspberry Pi, deploys rescue kits, avoids hazards, handles obstacles, restarts from checkpoints if stuck, and returns to start for exit bonuses, operating fully autonomously.



## Table of Contents

1. [Project Overview](#project-overview)
2. [Competition Requirements](#competition-requirements)
3. [System Architecture](#system-architecture)

   * [Mechanical Design](#mechanical-design)
   * [Hardware Components](#hardware-components)
4. [Software Architecture](#software-architecture)

   * [Arduino Layer](#arduino-layer)
   * [Raspberry Pi Layer](#raspberry-pi-layer)
5. [Control and Navigation Algorithms](#control-and-navigation-algorithms)
6. [Performance and Testing](#performance-and-testing)
7. [Future Improvements](#future-improvements)
8. [Team](#team)



## Project Overview

The **RCJ Rescue Maze Robot** was designed to meet the challenges of autonomous navigation in rescue maze environments.
The robot integrates:

* Arduino Mega 2560 for low-level control.
* Raspberry Pi 5 for computer vision and decision-making.
* Custom algorithms including Depth-First Search (DFS) and PID control.

The primary objective is to ensure robust navigation, real-time victim detection, and obstacle avoidance in compliance with RoboCup Rescue Maze competition standards.



## Competition Requirements

The RoboCup Rescue Maze competition specifies that the robot must:

* Navigate the maze autonomously without external control.
* Detect and avoid obstacles.
* Recognize victim identifiers such as letters and colors.
* Perform precise movements through narrow and complex paths.
* Manage power efficiently to sustain long runs.



## System Architecture

### Mechanical Design

* Multi-tier modular structure:

  * Bottom tier: Arduino, motor drivers, line/color sensors.
  * Middle tier: Infrared distance sensors.
  * Upper tier: Raspberry Pi 5 with dual cameras.
  * Top tier: Lithium batteries and power bank.

This design ensures stability, modularity, and ease of maintenance.

### Hardware Components

* **Microcontroller**: Arduino Mega 2560
* **Motors**: Two 12V DC motors with encoders
* **Sensors**:

  * Sharp IR distance sensors (front, back, left, right)
  * MPU6050 gyroscope/accelerometer
  * TCS34725 color sensor
  * TCRT500 line follower sensor
  * Limit switch
* **Vision Unit**: Raspberry Pi 5 with two camera modules


## Software Architecture

### Arduino Layer

Responsible for low-level motor control, sensor fusion, and real-time navigation:

* Initialization of sensors, motors, and timers.
* Depth-First Search (DFS) for maze exploration.
* Maze represented as a 70x70 grid with 5-bit cells (walls and visited status).
* Movement functions for forward, backward, rotation, and stopping.
* PID control for smooth acceleration, deceleration, and orientation stability.

### Raspberry Pi Layer

Handles vision and high-level decision making:

* Image capture with dual cameras.
* OpenCV-based color detection (HSV masks).
* OCR using Tesseract for victim letter recognition.
* Decision-making logic for sending commands to Arduino via serial.
* Noise reduction and image preprocessing (erosion/dilation filters).
* Automated startup via systemd service.



## Control and Navigation Algorithms

* **Depth-First Search (DFS)**: Used for pathfinding and backtracking.
* **PID Control**: Maintains smooth motor response and direction.

  * Proportional, Integral, and Derivative components tuned for precision.
* **Gyroscope Integration**: Angle tracking for accurate turns.
* **Map Navigation**: Maze matrix updated with visited nodes and wall data.



## Performance and Testing

### Simulation Testing

* 95% success rate in reaching targets across multiple maze configurations.
* Effective loop and dead-end handling through DFS and visited stack.
* IR sensor error margin: approximately 6%.

### Real-World Testing

* 93% navigation accuracy in physical mazes with dynamic obstacles.
* Continuous operation of 4 hours per battery cycle.
* Smooth turns and stable navigation with PID response time below 0.5s.

### Vision Testing (Raspberry Pi)

* Accurate detection of red, yellow, green, and black tiles.
* Reliable OCR recognition of victim letters.
* Stable frame processing rate on Raspberry Pi 5.



## Future Improvements

* Implement heuristic algorithms such as A\* or Dijkstra for optimal pathfinding.
* Introduce sensor fusion with LIDAR or ultrasonic sensors for robustness.
* Adaptive PID tuning for real-time control optimization.
* Advanced power management system for extended operation.
* Refactor software using state machines or behavior trees for modularity.
* Explore reinforcement learning for adaptive maze-solving strategies.


## Team

**Maram Ouelhazi** - [maramOuelhazii](https://github.com/maramOuelhazii)

**Wissal Naouaii** - [WissalNaouaiii](https://github.com/WissalNaouaii)



*You can consult the detailed specifications document [here](RCJRescueMaze2024RulesDraft.pdf),*
*and the Engineering Journal [here](rescue_maze.pdf).*
