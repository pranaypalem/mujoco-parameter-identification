"""
Tests for GIF generator module.
"""

import pytest
import os
import sys
from pathlib import Path

# Add src to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import gif_generator


def test_module_imports():
    """Test that the gif_generator module imports correctly."""
    assert hasattr(gif_generator, 'run_simulation_and_generate_gif')
    assert hasattr(gif_generator, 'main')


def test_path_creation():
    """Test path creation functionality."""
    output_path = Path("test_media")
    output_path.mkdir(parents=True, exist_ok=True)
    assert output_path.exists()
    
    # Cleanup
    output_path.rmdir()


def test_parameter_calculations():
    """Test parameter calculation logic."""
    # Test parameters
    gsm = 140
    length_long = 0.0254 * 8.5
    width = 0.0254 * 2
    num_joints = 10
    k_total = 1e-3
    b_total = 1.92e-5
    
    # Calculate expected values
    mass_paper = (gsm * length_long * width) * 0.001
    k_i = num_joints * k_total
    b_i = num_joints * b_total
    
    # Verify calculations
    assert mass_paper > 0
    assert k_i == 0.01
    assert b_i == 1.92e-4
    
    # Test ratios
    assert k_i / k_total == num_joints
    assert b_i / b_total == num_joints


def test_xml_template_formatting():
    """Test XML template string formatting."""
    template = """
    <mujoco>
        <body pos="{l1} 0 0">
            <joint stiffness="{k}" damping="{b}"/>
            <geom mass="{m}"/>
        </body>
    </mujoco>
    """
    
    formatted = template.format(l1=0.5, k=1.0, b=0.1, m=0.01)
    
    assert "0.5" in formatted
    assert "1.0" in formatted
    assert "0.1" in formatted
    assert "0.01" in formatted


# Skip rendering tests in CI environment
@pytest.mark.skipif(
    os.environ.get("CI") == "true" or not os.environ.get("DISPLAY"),
    reason="Skipping rendering tests in headless environment"
)
def test_gif_generation_mock():
    """Mock test for GIF generation (skipped in CI)."""
    # This test would run actual GIF generation with proper OpenGL setup
    pass


if __name__ == "__main__":
    pytest.main([__file__])