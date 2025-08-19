#!/usr/bin/env python3
"""
Parameter Identification Module for MuJoCo Simulations

This module contains the core functionality for identifying stiffness and
damping parameters of materials using MuJoCo simulations and optimization.
"""

from typing import Optional, Tuple

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd
import scipy.interpolate as si
import scipy.optimize as so
from scipy.signal import find_peaks


class ParameterIdentification:
    """
    Class for identifying material parameters using MuJoCo simulations.
    """

    def __init__(
        self,
        gsm: float = 140,
        length: float = 0.0254 * 8.5,
        width: float = 0.0254 * 2,
        thickness: float = 0.00016,
    ):
        """
        Initialize parameter identification.

        Args:
            gsm: Paper weight in GSM
            length: Length of the material in meters
            width: Width of the material in meters
            thickness: Thickness of the material in meters
        """
        self.gsm = gsm
        self.length = length
        self.width = width
        self.thickness = thickness
        self.mass_paper = (gsm * length * width) * 0.001  # Kg conversion

    def load_experimental_data(self, csv_path: str) -> pd.DataFrame:
        """
        Load experimental data from CSV file.

        Args:
            csv_path: Path to CSV file with experimental data

        Returns:
            DataFrame with experimental data
        """
        df = pd.read_csv(csv_path, sep=",")
        return df

    def interpolate_data(
        self, t: np.ndarray, x: np.ndarray, y: np.ndarray, time_step: float = 0.002
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Interpolate experimental data to create smooth functions.

        Args:
            t: Time array
            x: X position array
            y: Y position array
            time_step: Time step for interpolation

        Returns:
            Tuple of (new_time, interpolated_positions)
        """
        xy = np.array([x, y]).T
        f = si.interp1d(t, xy.T, fill_value="extrapolate", kind="quadratic")
        new_t = np.r_[0 : t[-1] : time_step]
        new_t = new_t[1:]  # removing timestep = 0

        # Create 3D array with y as z-coordinate for MuJoCo
        f_result = np.array([[f(t_val)[0], 0, f(t_val)[1]] for t_val in new_t])

        return new_t, f_result

    def create_mujoco_model(self, num_joints: int, k_i: float, b_i: float) -> str:
        """
        Create MuJoCo XML model string.

        Args:
            num_joints: Number of joints in the model
            k_i: Stiffness parameter for each joint
            b_i: Damping parameter for each joint

        Returns:
            XML string for MuJoCo model
        """
        main_template = """
        <mujoco>
            <option gravity="0 0 -9.81">
                <flag contact="disable"/>
            </option>
            <worldbody>
                <light name="top" pos="0 0 1"/>
                <body name="body_1" pos="{l1_2} 0 0">
                    <joint name="joint_1" type="hinge" axis="0 1 0" pos="-{l1_2} 0 0"
                           stiffness="{k}" damping="{b}"/>
                    <geom type="box" size="{l1_2} 0.0508 0.00016" pos="{l1_2} 0 0"
                          rgba="1 0 0 1" mass="{m}"/>
                    {inner}
                </body>
            </worldbody>
        </mujoco>
        """

        body_template = """
        <body name="body_{ii}" pos="{l1} 0 0">
            <joint name="joint_{ii}" type="hinge" axis="0 1 0" pos="0 0 0"
                   stiffness="{k}" damping="{b}"/>
            <geom type="box" size="{l1_2} 0.0508 0.00016" pos="{l1_2} 0 0"
                  rgba="1 0 0 1" mass="{m}"/>
            {inner}
        </body>
        """

        test_point = """
        <geom type="sphere" size="0.0254" pos="{l1} 0 0" rgba="0 0 1 1" mass="0.001"/>
        """

        n_bodies = num_joints
        l_i = self.length / n_bodies
        m_i = self.mass_paper / n_bodies
        body_numbers = np.r_[n_bodies:1:-1]

        s = ""
        for item in body_numbers:
            if item == body_numbers.max():
                s = body_template.format(
                    inner=test_point.format(l1="{l1}"),
                    ii=item,
                    l1="{l1}",
                    l1_2="{l1_2}",
                    b="{b}",
                    k="{k}",
                    m="{m}",
                )
            else:
                s = body_template.format(
                    inner=s,
                    ii=item,
                    l1="{l1}",
                    l1_2="{l1_2}",
                    b="{b}",
                    k="{k}",
                    m="{m}",
                )

        s = main_template.format(
            inner=s, l1="{l1}", l1_2="{l1_2}", b="{b}", k="{k}", m="{m}"
        )

        xml = s.format(l1=l_i, l1_2=l_i / 2, k=k_i, b=b_i, m=m_i)
        return xml

    def run_simulation(
        self,
        num_joints: int,
        k_i: float,
        b_i: float,
        start_sec: float = 4.0,
        end_sec: float = 12.0,
        duration: float = 20.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run MuJoCo simulation.

        Args:
            num_joints: Number of joints in the model
            k_i: Stiffness parameter
            b_i: Damping parameter
            start_sec: Start time for data collection
            end_sec: End time for data collection
            duration: Total simulation duration

        Returns:
            Tuple of (time_array, position_array)
        """
        xml = self.create_mujoco_model(num_joints, k_i, b_i)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)

        t = []
        xyz = []

        mujoco.mj_resetData(model, data)

        while data.time < duration:
            mujoco.mj_step(model, data)

            if start_sec < data.time <= end_sec + 0.0666:
                t.append(data.time)
                xyz.append(data.xpos.copy())

        return np.array(t), np.array(xyz)

    def calculate_error(
        self,
        params: Tuple[float, float],
        num_joints: int,
        experimental_data: np.ndarray,
        experimental_time: np.ndarray,
    ) -> float:
        """
        Calculate mean squared error between simulation and experimental data.

        Args:
            params: Tuple of (k, b) parameters
            num_joints: Number of joints
            experimental_data: Experimental position data
            experimental_time: Experimental time data

        Returns:
            Mean squared error
        """
        k, b = params
        _, xyz_sim = self.run_simulation(num_joints, k, b)

        # Time adjustment to align peaks
        peaks_sim, _ = find_peaks(xyz_sim[:, -1, 2])
        peaks_exp, _ = find_peaks(experimental_data[:, 2])

        if len(peaks_sim) == 0 or len(peaks_exp) == 0:
            return float("inf")

        z_sim = xyz_sim[peaks_sim[0] :, -1, 2]
        z_exp = experimental_data[peaks_exp[0] :, 2]

        # Match lengths
        if len(z_sim) <= len(z_exp):
            z_exp = z_exp[: len(z_sim)]
        else:
            z_sim = z_sim[: len(z_exp)]

        # Z amplitude adjustment
        if len(z_sim) > 0 and len(z_exp) > 0:
            diff = abs(z_exp[-1]) - abs(z_sim[-1])
            if z_exp[-1] >= z_sim[-1]:
                z_sim = z_sim + diff
            else:
                z_sim = z_sim - diff

        mse = np.mean((z_sim - z_exp) ** 2)
        return mse

    def optimize_parameters(
        self,
        experimental_data: np.ndarray,
        experimental_time: np.ndarray,
        num_joints: int,
        initial_guess: Tuple[float, float] = (1e-3, 1.95e-5),
        bounds: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None,
    ) -> dict:
        """
        Optimize stiffness and damping parameters.

        Args:
            experimental_data: Experimental position data
            experimental_time: Experimental time data
            num_joints: Number of joints in simulation
            initial_guess: Initial parameter guess (k, b)
            bounds: Parameter bounds

        Returns:
            Dictionary with optimization results
        """
        if bounds is None:
            bounds = ((1e-4, 1e-2), (1e-6, 1e-4))

        def objective(params):
            return self.calculate_error(
                params, num_joints, experimental_data, experimental_time
            )

        result = so.minimize(objective, initial_guess, method="powell", bounds=bounds)

        return {
            "k_optimal": result.x[0],
            "b_optimal": result.x[1],
            "error": result.fun,
            "success": result.success,
            "message": result.message,
        }

    def plot_comparison(
        self,
        experimental_time: np.ndarray,
        experimental_data: np.ndarray,
        simulation_time: np.ndarray,
        simulation_data: np.ndarray,
        title: str = "Simulation vs Experimental Data",
    ):
        """
        Plot comparison between simulation and experimental data.

        Args:
            experimental_time: Experimental time array
            experimental_data: Experimental position data
            simulation_time: Simulation time array
            simulation_data: Simulation position data
            title: Plot title
        """
        plt.figure(figsize=(10, 6))
        plt.plot(
            experimental_time,
            experimental_data[:, 2],
            label="Experimental Data",
            linewidth=2,
        )
        plt.plot(
            simulation_time,
            simulation_data[:, -1, 2],
            label="Simulation Data",
            linewidth=2,
            alpha=0.8,
        )
        plt.xlabel("Time (s)")
        plt.ylabel("Z Position (m)")
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
