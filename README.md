# Project Title

Parameter Identification with MuJoCo Simulations

## Overview

This repository focuses on identifying the stiffness and damping characteristics of a cardstock material by comparing real-life oscillation data collected using Tracker software with simulated oscillations generated in MuJoCo. The process involves using random stiffness and damping values in MuJoCo to generate simulated oscillations, and then applying `scipy.optimize` to minimize the difference between experimental and simulated data, thereby estimating the material parameters.

## Contents

- **Code**: Python scripts and Jupyter Notebooks used for data analysis and simulation.
- **Data**: Experimental data obtained using Tracker software and simulation outputs.
- **Documentation**: Explanations of methodologies and results.

## Tracker Software

Tracker (https://physlets.org/tracker/) is an open-source video analysis and modeling tool. It allows users to track objects in video footage to extract quantitative data about their motion. This tool was used in this project to collect real-life oscillation data by analyzing the motion of a physical system.

### How Tracker Was Used:
1. **Setup**: A real-life oscillatory system was recorded using a camera.
2. **Data Extraction**: The video was imported into Tracker, where key frames were analyzed to extract position and time data for the oscillating object.
3. **Data Export**: The processed data, including time, position, velocity, and acceleration, was exported as CSV files for further analysis.

### Why Tracker?

Tracker provided an accurate and user-friendly way to capture the dynamics of real-world motion, offering data that could be directly compared to MuJoCo simulations.

## MuJoCo Simulation

MuJoCo (Multi-Joint dynamics with Contact) is a physics engine commonly used for simulating and analyzing the dynamics of mechanical systems.

### Key Steps in Simulation:
1. **Model Design**: The oscillatory system was modeled in MuJoCo using its XML-based modeling framework.
2. **Parameter Initialization**: Random stiffness and damping values were assigned to generate initial simulated oscillations.
3. **Execution**: The simulation was run to produce oscillation data under controlled conditions.
4. **Optimization**: `scipy.optimize` was used to iteratively adjust stiffness and damping values, minimizing the difference between experimental and simulated data.
5. **Data Collection**: The optimized stiffness and damping values were identified, and the resulting simulation data was exported for validation.

### Features of MuJoCo:
- High accuracy in simulating mechanical systems.
- Support for complex dynamics, including joint constraints and friction.
- Fast computation, making it ideal for iterative analyses.

### Comparison Analysis:

The real-life data from Tracker and the simulation data from MuJoCo were compared using the following metrics:
- **Amplitude**: Peak displacement values were compared.
- **Frequency**: The periodicity of oscillations was analyzed.
- **Phase Difference**: The time shift between the datasets was calculated.
- **Damping**: The decay rate of oscillations was compared.

## Results

The project demonstrated:
- Accurate estimation of stiffness and damping parameters for the cardstock material.
- Validation of MuJoCo's effectiveness in replicating real-world oscillatory behavior.

## Prerequisites

To use this repository, ensure the following tools are installed:
- [Tracker Software](https://physlets.org/tracker/)
- MuJoCo (with a valid license)
- Python (3.8 or higher)
- Required Python libraries: numpy, pandas, matplotlib, scipy

## How to Run

1. **Install Tracker Software** and analyze your video to generate CSV data.
2. **Clone this Repository**:
   ```bash
   git clone https://github.com/pranaypalem/mujoco-parameter-identification.git
   ```
3. **Run Simulations**: Use the provided scripts to generate MuJoCo data.
4. **Optimize Parameters**: Use the optimization scripts to refine stiffness and damping values.
5. **Compare Data**: Utilize the provided Jupyter Notebooks to visualize and analyze differences.

## Acknowledgments

- The Tracker development team for their open-source tool.
- MuJoCo for providing a powerful simulation framework.
- Contributors and reviewers for their support.


