@page control_design Control Design and Iterative Development

## Overview
Achieving reliable line-following performance required both stable closed-loop control and accurate measurement of wheel motion. Throughout development, significant effort was spent refining encoder velocity estimation, task timing, and control parameters to improve smoothness and consistency.

This section highlights the iterative process used to characterize and improve system behavior.

---

## Step Response Testing
Open-loop step response tests were used to evaluate how changes in motor effort translated to wheel velocity. These tests exposed limitations in early velocity estimation approaches and informed subsequent improvements.

@image html images/romi_all_step_responses(1).png \
"Initial open-loop step response tests showing significant noise in the measured wheel velocity. These results motivated changes to encoder velocity estimation and task timing." width=90%

---

## Measurement Refinement
Improvements to encoder velocity calculations and adjustments to the motor control task period significantly reduced measurement noise. Increasing the time interval between velocity calculations improved numerical stability while preserving responsiveness.

@image html images/romi_all_step_responses(3).svg \
"Refined step response results after improving encoder velocity calculations and increasing the motor control task period. The measured velocity is significantly smoother and more consistent across effort steps." width=90%

---

## Control Implications
These measurement improvements directly enabled more effective closed-loop control. With reduced noise in velocity feedback, proportional–integral gains could be tuned to achieve stable tracking without excessive oscillation or overshoot.

The refined control behavior contributed to reliable line-following performance, particularly during transitions involving sharp curvature or rapid changes in commanded motion.

---

## Circular Line-Following Test
The video below shows the robot following a circular trajectory during early line-following development.

@video html images/driving-in-a-circle.mp4 width=70%

---

## Summary
Rather than relying solely on controller tuning, performance gains were achieved through careful attention to measurement quality and task timing. This iterative approach proved essential for achieving smooth, repeatable behavior in the final system.
