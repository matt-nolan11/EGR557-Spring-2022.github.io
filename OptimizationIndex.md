---
title: Design Optimization
---

# Design Optimization, Experiment Design, Data Collection, and Analysis

Based on the results and lengthy run-time of our System Dynamics code, the team decided against using software-based design iteration / optimization routines to optimize the design for most parameters, the exception being Gait Size vs. Kinetic Energy. For the other parameters, we chose to utilize a modular test platform to experimentally optimize certain design parameters. Although this approach is limited to experimentally tested values and does not necessarily find the absolutely most optimal design, it does help the team understand how changing each design parameter affects the performance of the system. Furthermore, the results of this experimental approach are guaranteed to directly apply to real-world systems. This is not always the case for simulation-based optimization, which makes several assumptions about the underlying physics. In particular, the performance of walking mechanisms is heavily dependent on ground reaction forces, which are especially difficult to simulate accurately.

Several specific research questions are posed in the Optimization Plan, and each of them are answered in the successive documents.

[Optimization Plan](\Optimization_Plan.pdf)\
[Stiffness vs. Speed (Viraj)](stiffness_vs_speed.pdf)\
[Geometry vs.Stiffness (Viraj)]()\
[Stiffness vs. Height (Matthew)](https://nbviewer.org/github/matt-nolan11/matt-nolan11.github.io/blob/main/Stiffness_vs_Height.ipynb)\
[Gait Size vs. Kinetic Energy (Nathan)](https://nbviewer.org/github/matt-nolan11/matt-nolan11.github.io/blob/main/Gait_Size_vs_Kinetic_Energy.ipynb)\
[Gait Size vs. Speed (Nathan)](https://nbviewer.org/github/matt-nolan11/matt-nolan11.github.io/blob/main/Gait_Size_vs_Speed.ipynb)\
[Mass vs. Speed (Shubang)](mass_vs_speed.pdf)
