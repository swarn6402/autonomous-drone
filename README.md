# 🚁 Autonomous Quadcopter Prototype

## Overview

This project presents the design and implementation of a quadcopter prototype integrating embedded systems, control systems, sensor fusion, and onboard video streaming.

The system combines a MultiWii flight controller with a Raspberry Pi to enable stabilized flight, GPS-assisted navigation, and real-time video transmission. The objective was to develop a reliable and cost-effective UAV platform capable of both manual and semi-autonomous operation.

---

## System Architecture

### Core Components

- Flight Controller: MultiWii
- Onboard Computer: Raspberry Pi
- Motors: Brushless DC Motors
- ESC: 18A Electronic Speed Controllers
- Battery: 3S 11.1V 3000mAh LiPo (25C)
- Propellers: 8045 SF (8-inch diameter, 4.5-inch pitch)
- Sensors: IMU (Gyroscope + Accelerometer), GPS, Barometer
- Telemetry: 3DR Radio Modules

---

## Mechanical Design

- Frame size: 420 mm motor-to-motor distance
- Frame material: ABS plastic
- Structural validation using FEM (SolidWorks and ANSYS)
- Safety factor: 3.7
- Average arm stress: 1.7 N/mm²

The frame was designed to maintain rigidity while minimizing weight to optimize flight time and responsiveness.

---

## Control System

The quadcopter employs a PID-based control system for stabilizing:

- Pitch
- Roll
- Yaw
- Throttle

Supported flight modes:

- ACRO (Gyroscope-based manual control)
- ANGLE (Stabilized mode)
- HORIZON (Hybrid mode)

Sensor fusion combines IMU, GPS, and barometer data to maintain orientation, altitude, and positional stability.

---

## Autonomous Capabilities

- GPS-based position hold
- Waypoint navigation
- Return-to-launch functionality
- Long-range telemetry communication

---

## Onboard Video Streaming

- Raspberry Pi used for onboard processing
- NGINX server configured with RTMP module
- Video pipeline implemented using GStreamer
- 2-axis servo-based camera gimbal for stabilization
- Live stream accessible over local network

---

## Power System

- 3S LiPo Battery (11.1V, 3000mAh, 25C)
- Maximum continuous current: 75A
- Average current draw: ~15A
- Estimated flight time: ~12 minutes

---

## Testing & Validation

- 50+ indoor and outdoor flight tests
- PID tuning via MultiWii Config interface
- Motor and propeller balancing
- Sensor calibration and telemetry validation
- FEM-based structural stress analysis

---

## Future Improvements

- Fully Raspberry Pi-based flight stack
- Vision-based obstacle detection
- SLAM-based navigation
- Enhanced power efficiency
