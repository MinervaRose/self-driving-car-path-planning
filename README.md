<div align="center">

# 🛣️ Self-Driving Car — Highway Path Planning

### Autonomous Highway Navigation and Trajectory Generation

![C++](https://img.shields.io/badge/C++-Autonomous_Systems-blue?style=for-the-badge&logo=cplusplus)
![Path Planning](https://img.shields.io/badge/Path_Planning-Trajectory_Generation-green?style=for-the-badge)
![Autonomous Driving](https://img.shields.io/badge/Autonomous_Driving-Decision_Making-orange?style=for-the-badge)
![Robotics](https://img.shields.io/badge/Robotics-Motion_Planning-red?style=for-the-badge)
![Self Driving Cars](https://img.shields.io/badge/Domain-Highway_Driving-purple?style=for-the-badge)

Udacity Self-Driving Car Engineer Nanodegree Project

</div>

---

# Overview

Once a vehicle can perceive its environment and localize itself, it must decide:

> Where should I go next?

This project implements a highway path planner capable of safely navigating among surrounding traffic while respecting speed limits, comfort constraints, and collision avoidance requirements.

The planner continuously generates smooth trajectories that allow the vehicle to:

- Stay within lane boundaries
- Adjust speed to traffic conditions
- Overtake slower vehicles
- Change lanes safely
- Avoid collisions
- Minimize acceleration and jerk

The result is a complete autonomous highway-driving behavior system operating in simulation.

---

# Project Objectives

The goals of this project were to:

- Navigate a multi-lane highway autonomously
- Drive near the speed limit
- Avoid collisions
- Perform safe lane changes
- Generate smooth trajectories
- Minimize acceleration and jerk
- Complete a full highway loop without incidents

---

# Highway Driving Architecture

```text
Localization
        ↓
Sensor Fusion
        ↓
Traffic Analysis
        ↓
Behavior Planning
        ↓
Lane Selection
        ↓
Trajectory Generation
        ↓
Vehicle Motion
```

---

# Sensor Fusion

The planner continuously analyzes surrounding traffic using sensor fusion data.

Information available for nearby vehicles includes:

- Vehicle position
- Velocity
- Lane assignment
- Relative distance
- Predicted future position

This information is used to identify:

- Slower vehicles ahead
- Available adjacent lanes
- Potential collision risks
- Overtaking opportunities

---

# Traffic Analysis

The planner evaluates surrounding traffic conditions in real time.

Key decisions include:

### Following

If a slower vehicle occupies the current lane, the ego vehicle adapts its speed to maintain a safe distance. :contentReference[oaicite:1]{index=1}

### Overtaking

When possible, the planner searches for a faster adjacent lane and performs a lane change. :contentReference[oaicite:2]{index=2}

### Collision Avoidance

Lane changes are only allowed when sufficient space exists both ahead and behind the vehicle in the target lane. :contentReference[oaicite:3]{index=3}

---

# Frenet Coordinate System

The project uses Frenet coordinates:

- **s** = distance along the road
- **d** = lateral position within the roadway

This representation simplifies:

- Lane tracking
- Lane changes
- Trajectory generation
- Highway navigation

The highway map consists of waypoints describing the road geometry. :contentReference[oaicite:4]{index=4}

---

# Trajectory Generation

The planner generates future vehicle trajectories by:

1. Determining a target lane
2. Selecting future waypoints
3. Transforming coordinates
4. Fitting a smooth spline
5. Sampling trajectory points

```text
Target Lane
      ↓
Future Waypoints
      ↓
Spline Fit
      ↓
Trajectory Points
      ↓
Vehicle Motion
```

---

# Smooth Driving with Splines

Abrupt steering changes create uncomfortable and unsafe motion.

To produce smooth trajectories, the planner uses cubic spline interpolation. :contentReference[oaicite:5]{index=5}

Benefits include:

- Smooth lane changes
- Reduced jerk
- Reduced acceleration spikes
- Passenger comfort
- Stable vehicle behavior

---

# Speed Control

The vehicle speed is continuously adjusted according to:

- Traffic conditions
- Desired cruising speed
- Safety margins
- Lane change decisions

A hysteresis-based controller prevents unstable speed oscillations and produces gradual acceleration and deceleration. :contentReference[oaicite:6]{index=6}

---

# Technical Skills Demonstrated

## Autonomous Systems

- Path Planning
- Behavior Planning
- Motion Planning
- Decision Making

## Robotics

- Trajectory Generation
- Frenet Coordinates
- Motion Control

## Software Engineering

- Modern C++
- Numerical Methods
- Real-Time Algorithms

## Autonomous Driving

- Highway Navigation
- Lane Selection
- Collision Avoidance
- Traffic Interaction

---

# Repository Structure

```text
src/
├── main.cpp
├── spline.h
├── helpers.h

data/
├── highway_map.txt

README.md
WRITEUP.md
```

---

# Results

The planner successfully:

✅ Maintains lane discipline

✅ Avoids collisions

✅ Respects speed limits

✅ Performs safe overtakes

✅ Generates smooth trajectories

✅ Completes highway loops without incident

The resulting behavior resembles a simplified autonomous highway-driving stack.

---

# Key Concepts Explored

- Path Planning
- Motion Planning
- Behavior Planning
- Highway Driving
- Sensor Fusion
- Trajectory Generation
- Frenet Coordinates
- Collision Avoidance
- Autonomous Navigation

---

# Why This Project Matters

Path planning sits at the center of autonomous driving.

A vehicle may accurately perceive the world and know where it is, but it still requires a decision-making layer capable of selecting safe future actions.

The techniques explored here form the basis of planning systems used in:

- Autonomous vehicles
- Mobile robots
- Autonomous drones
- Planetary rovers
- Warehouse automation systems

---

# Related Self-Driving Car Projects

This repository is part of a larger autonomous driving portfolio:

- Finding Lane Lines
- Advanced Lane Finding
- Traffic Sign Classifier
- Behavioral Cloning
- Extended Kalman Filter Sensor Fusion
- Kidnapped Vehicle Localization
- Highway Path Planning
- PID Controller

Together these projects cover perception, localization, planning, control, and autonomous navigation.

---

# Learning Outcomes

This project provided practical experience with:

- Highway traffic behavior
- Motion planning
- Trajectory optimization
- Autonomous decision making
- Real-time navigation systems

---

# Disclaimer

This repository is provided for educational and portfolio purposes.

Students may study the code and reports for learning purposes, but submitting this work as coursework would constitute plagiarism and may violate academic integrity policies.

Copyright © Sabrina Palis
