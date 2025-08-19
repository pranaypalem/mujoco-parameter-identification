#!/usr/bin/env python3
"""
GIF Generator for MuJoCo Parameter Identification Project

This module converts MP4 video frames from MuJoCo simulations to GIF format.
"""

from pathlib import Path

import mediapy as media
import mujoco
import numpy as np


def run_simulation_and_generate_gif(
    num_joints=100,
    k_total=1e-3,
    b_total=1.92e-5,
    gsm=140,
    length_long=0.0254 * 8.5,
    width=0.0254 * 2,
    thickness=0.00016,
    output_dir="media",
    filename="simulation.gif",
    duration=20,
    framerate=30,
):
    """
    Run MuJoCo simulation and generate GIF from frames.

    Args:
        num_joints: Number of joints in the simulation
        k_total: Total stiffness parameter
        b_total: Total damping parameter
        gsm: Paper weight in GSM
        length_long: Length of the paper in meters
        width: Width of the paper in meters
        thickness: Thickness of the paper in meters
        output_dir: Directory to save the GIF
        filename: Name of the output GIF file
        duration: Simulation duration in seconds
        framerate: Video framerate

    Returns:
        Path to the generated GIF file
    """

    # Create output directory if it doesn't exist
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # Calculate parameters
    mass_paper = (gsm * length_long * width) * 0.001  # Kg conversion
    k_i = num_joints * k_total
    b_i = num_joints * b_total

    # XML template for MuJoCo model
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

    # Build XML model
    n_bodies = num_joints
    total_length = length_long  # Total Length in m
    length_per_body = total_length / n_bodies
    mass_per_body = mass_paper / n_bodies
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
                inner=s, ii=item, l1="{l1}", l1_2="{l1_2}", b="{b}", k="{k}", m="{m}"
            )

    s = main_template.format(
        inner=s, l1="{l1}", l1_2="{l1_2}", b="{b}", k="{k}", m="{m}"
    )

    xml = s.format(
        l1=length_per_body, l1_2=length_per_body / 2, k=k_i, b=b_i, m=mass_per_body
    )

    # Create MuJoCo model and simulation
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model)

    frames = []
    mujoco.mj_resetData(model, data)

    print(f"Running simulation for {duration} seconds...")

    while data.time < duration:
        mujoco.mj_step(model, data)

        # Capture frame every 1/framerate seconds
        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            pixels = renderer.render()
            frames.append(pixels)

    print(f"Simulation complete. Captured {len(frames)} frames.")

    # Generate GIF
    gif_path = output_path / filename
    print(f"Generating GIF: {gif_path}")

    media.write_video(str(gif_path), frames, fps=framerate)

    print(f"GIF generated successfully: {gif_path}")
    return gif_path


def main():
    """Main function to generate simulation GIF."""
    print("MuJoCo Parameter Identification - GIF Generator")
    print("=" * 50)

    # Generate GIF with default parameters
    gif_path = run_simulation_and_generate_gif(
        num_joints=100,
        filename="mujoco_simulation_100_joints.gif",
        duration=8,  # Shorter duration for GIF
        framerate=15,  # Lower framerate for smaller file size
    )

    print(f"\nGIF saved to: {gif_path}")
    print("Generation complete!")


if __name__ == "__main__":
    main()
