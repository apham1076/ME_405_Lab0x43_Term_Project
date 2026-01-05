@page state_estimation_odometry State Estimation and Odometry

## Motivation
As part of Lab 0x05, we explored state estimation techniques with the goal of reconstructing the Romi’s motion using a mathematical model combined with sensor measurements. The motivation was to move beyond raw encoder data and instead maintain a consistent estimate of key system states such as wheel velocities, arc length traveled, and heading.

While this approach aligned well with the theoretical objectives of the lab, implementing and validating the estimator on real hardware revealed important limitations. The work ultimately informed our decision to rely on odometry for the term project, but the process of designing, implementing, and testing the estimator provided valuable insight into the behavior of the system.

---

## State Estimator Implementation
The state estimator developed in Lab 0x05 was based on a discrete-time observer formulation. A continuous-time state-space model was first constructed and then discretized for implementation on the microcontroller. The estimator state vector included wheel velocities, arc length, and heading, and incorporated encoder and IMU measurements through an observer gain matrix.

The estimator was implemented as its own task within the cooperative multitasking framework. At each update, it propagated the system state forward in time using the discrete model and corrected the estimate using incoming measurements. In principle, this structure was intended to provide smoother and more reliable state information than raw encoder measurements alone.

---

## Comparison to Encoder Measurements
To evaluate estimator performance, we compared state-estimated values directly against quantities derived from encoder measurements under several controlled test cases. The following plots show results for straight-line motion, in-place pivoting, and arc motion, all conducted at a fixed effort of 50%.

@image html estimator_vs_encoder_straight.svg "State-estimated and encoder-derived values during straight-line motion at 50% effort." width=75%

@image html estimator_vs_encoder_pivot.svg "State-estimated and encoder-derived values during in-place pivot motion at 50% effort." width=75%

@image html estimator_vs_encoder_arc.svg "State-estimated and encoder-derived values during arc motion at 50% effort." width=75%

Across these tests, the estimated arc length and heading tracked the encoder-derived values reasonably well. However, the estimated wheel velocity consistently differed in magnitude from the measured velocity, often appearing significantly lower. This discrepancy persisted across multiple trials and parameter adjustments.

---

## Limitations and Design Decision
Although portions of the estimator behaved as expected, the inconsistency in the velocity estimates raised concerns about the reliability of the overall state vector. Because wheel velocity is a foundational quantity used throughout the control architecture, uncertainty in this estimate limited our confidence in using the estimator for real-time decision-making during the term project.

As a result, we made the deliberate decision to rely on odometry-based calculations for estimating absolute position and heading. This approach was simpler, more transparent, and more consistent with the behavior observed on the physical robot. Importantly, this decision was informed directly by experimental results rather than theoretical considerations alone.

---

## Odometry-Based Pose Estimation
For the term project, absolute position and heading were computed using odometry derived from wheel encoder measurements. Incremental wheel motion was integrated over time to estimate:

- Total arc length traveled
- Absolute X and Y position
- Absolute heading angle

These calculations were implemented in a dedicated task referred to as the *spectator task*. Conceptually, this task serves a similar role to a state estimator in that it provides higher-level information about the robot’s motion, but without relying on a full observer model.

---

## Odometry Validation on Circular Line Following
To evaluate the effectiveness of the odometry calculations, the robot was commanded to follow a circular line for approximately two full revolutions. During this test, both a real-world video of the robot and logged odometry data were recorded.

The video below shows the physical robot following the circular path:

@htmlonly
<div class="image" style="text-align: center;">
  <video controls width="70%">
    <source src="Romi_Line_Following.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
  <div class="caption">
    Romi following a circular line for approximately two full revolutions during odometry data collection.
  </div>
</div>
@endhtmlonly

Using the recorded odometry data (absolute X, Y, and heading), we generated an animation of the robot’s reconstructed motion. The animation closely matches the robot’s observed position and orientation throughout the test, providing strong qualitative validation of the odometry calculations.

@htmlonly
<div class="image" style="text-align: center;">
  <video controls width="70%">
    <source src="romi_odometry_two_circles.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
  <div class="caption">
    Odometry-based reconstruction of Romi’s position and orientation during the same circular line-following test.
  </div>
</div>
@endhtmlonly

---

## Discussion
The combination of state estimation and odometry work highlights an important aspect of system design: choosing the right level of model complexity for a given application. While the state estimator provided valuable insight and reinforced theoretical concepts from the course, odometry proved to be a more reliable and interpretable solution for our final system.

This experience emphasized the importance of validating algorithms against real hardware behavior and making design decisions based on experimental evidence rather than assumptions alone.
