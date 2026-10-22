# Robotic Manipulator Kinematics and Trajectory Generation
A practical robotics project focused on modeling, trajectory planning, and numerical simulation for a 6-DOF **Stäubli TX-60l** industrial robotic arm using Python and PyBullet.
---
### Project Overview
The objective of this lab is to control the end-effector (tool) of a 6-axis manipulator robot so that it moves smoothly along a predefined Cartesian trajectory between two points ($A$ and $B$) under precise kinematic constraints.
The project translates task-space instructions (positions, velocities, accelerations) into joint-space commands executed at a fixed sample period ($T_e \in [1, 10]\text{ ms}$).
