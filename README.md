
# Analysis of Multi Agent Systems

This repository contains the code and documentation for simulating multi-agent systems using MATLAB and Webots. The code forms part of the dissertation titled *Effects of increased connectivity
amongst leaders on the performance of
a multi-agent system* which was submitted to The University of Sheffield.

## Introduction
The objective of this dissertation was to develop a simulation framework for multi-agent systems using MATLAB and Webots. The simulation framework allows for the study of various multi-agent system scenarios, such as stability analysis and target tracking, using different communication and control strategies.

The simulation framework comprises two main components: MATLAB code and Webots code. The MATLAB code implements the control algorithms for the agents with different system parameters and control gains. The Webots code implements the physics and graphics engine, which provides a realistic simulation environment for the agents.

## Getting started
To run the simulations, you will need MATLAB and Webots installed on your machine. The MATLAB code is located in the `/matlab` directory, and the Webots code is located in the `/webots` directory.

To run the simulations in MATLAB, follow these steps:
1. Open MATLAB and navigate to the `/matlab` directory.
2. Open the `simulatemas.m` script and modify the simulation parameters as desired.
3. Run the `simulatemas.m` script to start the simulation.

To run the simulations in Webots, follow these steps:
1. Open Webots and navigate to the `/webots` directory.
2. Open the `/webots/worlds` directory and and modify the simulation parameters as desired in `epuck_positioner.py`. Run the Python script to implement the changes. Alternatively, skip to the next step to run the simulation with the default parameters.
3. Run the simulation by clicking the "Play" button in Webots.

## Documentation
The documentation for this project is located in the docs directory. The documentation includes the dissertation, which provides a detailed description of the simulation framework, the control algorithms used, and the simulation scenarios studied.

## Acknowledgements
I would like to acknowledge the support and guidance of my supervisor, Dr. Anton Selivanov. I would also like to thank the contributors to the MATLAB and Webots open-source communities for their invaluable resources and support.