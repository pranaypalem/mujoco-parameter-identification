# MuJoCo Parameter Identification

[![CI](https://github.com/pranaypalem/mujoco-parameter-identification/workflows/CI/badge.svg)](https://github.com/pranaypalem/mujoco-parameter-identification/actions)
[![codecov](https://codecov.io/gh/pranaypalem/mujoco-parameter-identification/branch/main/graph/badge.svg)](https://codecov.io/gh/pranaypalem/mujoco-parameter-identification)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

**Parameter identification for material properties using MuJoCo simulations and optimization techniques.**

This project identifies stiffness and damping characteristics of cardstock material by comparing real-world oscillation data with MuJoCo physics simulations, using scipy optimization to minimize the difference between experimental and simulated results.

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

## Key Features

- **Material Parameter Identification**: Automatically determines stiffness and damping parameters
- **MuJoCo Integration**: Uses advanced physics simulation for accurate modeling  
- **Optimization Pipeline**: Scipy-based parameter fitting with MSE minimization
- **Visualization**: Generate GIFs and plots of simulation results
- **Robust Testing**: Comprehensive test suite with CI/CD integration
- **Easy Setup**: One-command local environment setup

## Prerequisites

- Python 3.8 or higher
- [Tracker Software](https://physlets.org/tracker/) (for experimental data collection)
- MuJoCo physics engine

## Repository Structure

```
├── data/                     # Experimental data files
├── notebooks/               # Jupyter notebooks for analysis
├── src/                     # Python source code
│   ├── gif_generator.py     # GIF generation from simulations
│   └── parameter_identification.py  # Core parameter identification
├── tests/                   # Unit tests
├── media/                   # Generated media files (GIFs, videos)
├── results/                 # Analysis results
├── Makefile                 # Build and test automation
├── requirements.txt         # Python dependencies
└── pyproject.toml          # Project configuration
```

## Quick Start

1. **Clone the repository**:
   ```bash
   git clone https://github.com/pranaypalem/mujoco-parameter-identification.git
   cd mujoco-parameter-identification
   ```

2. **Set up environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   make install-dev  # Installs all dependencies including dev tools
   ```

3. **Verify setup**:
   ```bash
   make test
   make lint
   ```

4. **Generate simulation visualization**:
   ```bash
   make generate-gif
   ```

### Available Make Commands

| Command | Description |
|---------|-------------|
| `make help` | Show all available commands |
| `make install` | Install dependencies |
| `make test` | Run unit tests |
| `make lint` | Run code linting |
| `make format` | Format code with black and isort |
| `make generate-gif` | Generate GIF from MuJoCo simulation |
| `make clean` | Clean temporary files |

## Results

The optimization process successfully identifies:
- **Stiffness parameters (k)** with high accuracy
- **Damping coefficients (b)** for material characterization  
- **Mean square error** minimization between experimental and simulated data
- **Visual validation** through comparative plots and animations

## Research Applications

This methodology can be extended to:
- Various material types (plastics, metals, composites)
- Different loading conditions and geometries
- Multi-parameter optimization problems
- Real-time parameter estimation systems

## Acknowledgments

- Tracker development team for the open-source motion analysis tool
- MuJoCo team for the advanced physics simulation framework


