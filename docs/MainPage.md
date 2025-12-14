@mainpage Overview

## Term Project
This project was completed as part of **ME 405: Mechatronics** at Cal Poly San Luis Obispo. The objective was to design and implement a fully autonomous Romi robot capable of reliably completing a printed line-following time-trial course using onboard sensing, real-time control, and a cooperative multitasking software architecture.

The final system integrates closed-loop motor control, encoder-based feedback, reflectance sensor processing, and task-level coordination to achieve repeatable and robust performance. Design decisions prioritized reliability, modularity, and predictable behavior over purely maximizing speed.

## Documentation

- @ref hardware
- @ref software_architecture
- @ref control_design
- @ref state_estimation_odometry
- @ref getting_started

---

## System Capabilities
The final Romi system demonstrates the following capabilities:

- Fully autonomous line following using onboard sensing and control
- Closed-loop wheel velocity regulation using encoder feedback
- Cooperative multitasking firmware with deterministic task execution
- Odometry-based position and heading estimation
- Real-time data streaming to a PC for debugging and analysis

---

## Line-Following Course
The robot was evaluated on a printed line-following course used for time trials and final demonstrations.

@image html Game_Track.svg "Printed line-following course used for time trials and final demonstration." width=80%

---

## Hardware & Electrical Design
The robot is built on a Pololu Romi chassis with two DC motors driven by onboard motor drivers and instrumented with quadrature encoders for wheel position and velocity feedback. Line detection is performed using a reflectance sensor array mounted at the front of the chassis and positioned to provide consistent contrast while minimizing sensitivity to ambient lighting.

Power is supplied by **six AA batteries**. Several electrical design decisions were made to ensure predictable startup behavior, including modifying default pull-up configurations and selecting motor control pins to prevent unintended motion on power-up. Encoder wiring, sensor placement, and cable management were refined to improve repeatability and robustness during operation.

---

## Software Architecture
The firmware is organized using a **cooperative multitasking framework** implemented in MicroPython. Independent tasks handle sensing, control, actuation, and communication, and they exchange information using thread-safe shared variables and queues.

This structure allows time-critical operations, such as motor updates and sensor sampling, to execute deterministically while higher-level logic runs concurrently without blocking. The modular task design simplified debugging and enabled incremental development throughout the quarter.

---

## Closed-Loop Motor Control
Wheel encoders provide continuous feedback for closed-loop motor control. A proportional–integral controller regulates wheel velocity, allowing the robot to track commanded setpoints consistently across operating conditions.

@image html run3_V_sp15.0_Kp5.00_Ki0.05_left_vel.png "Representative closed-loop velocity response for the left motor. The controller tracks a commanded velocity setpoint using tuned proportional–integral gains, demonstrating stable and repeatable behavior." width=80%

---

## Final Demonstration
The video below shows the robot navigating the printed course during a final demonstration run and successfully reaching the designated checkpoint.

@htmlonly
<div class="image" style="text-align: center;">
  <video controls width="80%">
    <source src="Romi_Demo.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
  <div class="caption">
    Final demonstration run of the Romi robot navigating the printed line-following
    course and successfully reaching the designated checkpoint.
  </div>
</div>
@endhtmlonly

---

## Repository
All source code, documentation, and design files for this project are available in the associated GitHub repository. The repository is organized to allow the system to be reproduced or extended in future work.
