# Autonomous Welding-Fume Extraction Robot Arm

A ROS 2–based robotic prototype that automatically positions the hood of a mobile fume-extraction unit near an active welding point.

The project was developed as part of the master’s thesis “Autonomous Positioning of a Mobile Extraction Unit for Welding Processes” at the University of Lübeck, in cooperation with the Hamburg Port Authority.


## Motivation

Welding fumes should be captured as close as possible to their source. With conventional mobile extraction units, the welder must repeatedly stop and reposition the extraction hood when working along longer seams.


## Prototype Overview

The prototype is a compact articulated robot arm built mainly from fischertechnik and custom 3D-printed parts. Its kinematic design contains four actively controlled revolute joints and one passive joint:

base rotation,

shoulder,

elbow,

a passive tilting connection for the extraction hood,

end-effector rotation.



The passive joint keeps the hood directed downward through gravity, while the final active joint compensates for base rotation so that the selected hood tilt remains aligned with the operator.

## Main hardware

Raspberry Pi 5, 8 GB RAM

Arduino Uno for time-critical sensor acquisition

Raspberry Pi Camera Module 3

shade 11 welding glass for optical filtering

fischertechnik encoder motors and PI-F5 adapter

HC-SR04 ultrasonic distance sensors

INA219 current sensors

MMA8452Q acceleration sensor

I²C multiplexer

emergency-stop and release buttons

WS2812 LED ring

3D-printed extraction hood, sensor mounts, joints, and gears

## Computing architecture

The Raspberry Pi performs image processing, kinematics, motion planning, high-level control, and ROS 2 communication. The Arduino handles operations that are sensitive to timing, including ultrasonic measurements and encoder pulse counting, and transfers the data to the Raspberry Pi over USB.

## Software Stack

The target platform is:

Ubuntu 24.04

ROS 2 Jazzy

MoveIt 2

ros2_control

RViz 2

C++ and Python

Arduino firmware

MoveIt 2 is used for robot modelling, inverse kinematics, collision checking, motion planning, and execution. 


## Core Functions

Welding-point detection

The welding arc is observed with an RGB camera fitted with welding glass. Because the filter blocks most ambient light, the active welding point appears as the brightest region in the image.

The image-processing pipeline is designed to:

search for the brightest connected region rather than a single pixel,

restrict detection to a region of interest when tracking is stable,

reject invalid or implausible detections,

reduce jumps caused by reflections and temporary occlusion, and

provide the image-space offset from the desired hood position.

The current prototype performs two-dimensional tracking. A future stereo-camera system could add depth estimation for three-dimensional weld paths.

## Robot motion

The controller converts the detected image offset into robot motion. Two approaches were investigated:

planned Cartesian target movements through MoveIt 2, and

direct Cartesian velocity control using a custom inverse-kinematics implementation.

A dead band around the image centre prevents continuous small corrections. Motion commands are limited to reduce oscillation, overshoot, and the effect of outdated camera measurements during longer trajectories.

Motor and encoder control

The robot uses fischertechnik motors that require a minimum drive level to start reliably. A dedicated PWM loop separates motor switching from the lower-frequency ROS control cycle, improving startup behaviour and reducing irregular motion.

Encoder data is used to estimate joint movement. Because the available encoders provide pulse counts without direction, the commanded motor direction is used as part of the position estimate. Mechanical back-driving, coasting, and movement without an active command remain limitations of the prototype.

## Safety monitoring
The experimental safety layer includes:

ultrasonic distance monitoring around the hood,

motor-current monitoring for unexpected loads,

acceleration monitoring for sudden impacts,

software position and speed limits,

reduced motor power, and

an emergency-stop input.

The prototype emergency stop is processed in software and does not physically disconnect motor power. A full-scale system requires safety-rated emergency-stop circuitry, safe torque off, complete sensor coverage, and compliance with all applicable machinery and collaborative-robot standards.

## Getting Started

1. Install ROS 2 and MoveIt 2

Install ROS 2 Jazzy and MoveIt 2 on Ubuntu 24.04. Source the ROS installation before building:

`source /opt/ros/jazzy/setup.bash`

2. Create a workspace

```
mkdir -p ~/robotarm_ws/src
cd ~/robotarm_ws/src
git clone https://github.com/michaelastein/robotarm.git
``` 

Copy or link the packages from robotarm/ROS_WS/src into the workspace source directory:

`cp -r robotarm/ROS_WS/src/* ~/robotarm_ws/src/`

3. Install dependencies

```
cd ~/robotarm_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build

```
colcon build --symlink-install
source install/setup.bash
```

All terminals used to run the project must source both ROS 2 and the workspace:

```
source /opt/ros/jazzy/setup.bash
source ~/robotarm/ROS_WS/install/setup.bash
```

## Path Planning:

Temrinal 1: 
`ros2 launch robotarm_software   robotarm_path_controller.launch.py `

Terminal 2: 
`ros2 run robotarm_software start_position_path.py -- -s  `
To move to start position, then
`ros2 launch robotarm_software hotspot_path_planning.launch.py`


## Velocity Control:

Temrinal 1: 
`ros2 launch robotarm_software   robotarm_velocity_controller.launch.py`

Terminal 2: 
`ros2 run robotarm_software start_position_velocity.py -- -s`
To move to start position, then
`ros2 launch robotarm_software hotspot_velocity.launch.py`


See the camera image under http://192.168.137.2:8000/ (replace with Raspberry Pi`s IP address)

## Current Limitations

The prototype tracks the welding arc only while it is active.

Tracking is two-dimensional and does not directly measure depth.

Reflections from metal surfaces can produce false detections.

The welding torch can temporarily occlude the arc.

The filtered camera image is black when no arc is present.

The robot must currently be moved to its initial position through the software interface.

Ultrasonic coverage is incomplete and does not protect every part of the arm.

Encoder direction is inferred rather than measured directly.

Low-cost motors and gear trains produce backlash and comparatively rough motion.

The software emergency stop is not safety-rated.

The prototype is not approved for unsupervised or industrial operation.

## Roadmap

stereo-camera tracking for three-dimensional weld paths,

welding cameras or auto-darkening filters,

tool detection while the arc is inactive,

improved reflection and occlusion handling,

encoders with absolute position and direction,

smoother motors and lower-backlash transmissions,

full-body obstacle and proximity sensing,

torque and contact sensing,

safety-rated emergency-stop and drive shutdown,

glove-friendly manual guidance and controls, and

validation on a full-scale mobile extraction unit.

## Thesis Context

The project accompanies the master’s thesis:

Michaela SteinAutonomous Positioning of a Mobile Extraction Unit for Welding ProcessesInstitute of Medical Electrical EngineeringRobotics and Autonomous Systems, University of LübeckDeveloped in cooperation with the Hamburg Port Authority, 2026.

## Acknowledgements

Parts of the initial 4-DOF kinematics and ROS 2 structure were adapted from the RobotArm project by M. Ursu (mu1492/RobotArm) and subsequently modified for this prototype.
