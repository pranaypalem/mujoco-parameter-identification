"""
Tests for parameter identification module.
"""

import pytest
import numpy as np
import pandas as pd
import os
import sys
from pathlib import Path
import tempfile

# Add src to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from parameter_identification import ParameterIdentification


def test_parameter_identification_init():
    """Test ParameterIdentification initialization."""
    pi = ParameterIdentification()
    
    # Test default values
    assert pi.gsm == 140
    assert pi.length == 0.0254 * 8.5
    assert pi.width == 0.0254 * 2
    assert pi.thickness == 0.00016
    assert pi.mass_paper > 0
    
    # Test custom values
    pi_custom = ParameterIdentification(gsm=200, length=0.1, width=0.05)
    assert pi_custom.gsm == 200
    assert pi_custom.length == 0.1
    assert pi_custom.width == 0.05


def test_interpolation_data():
    """Test data interpolation functionality."""
    pi = ParameterIdentification()
    
    # Create test data
    t = np.array([0.0, 0.1, 0.2, 0.3, 0.4])
    x = np.array([0.0, 0.1, 0.2, 0.1, 0.0])
    y = np.array([0.0, 0.05, 0.1, 0.05, 0.0])
    
    new_t, f_result = pi.interpolate_data(t, x, y, time_step=0.05)
    
    # Check output shapes
    assert len(new_t) > len(t)  # Should have more time points
    assert f_result.shape[1] == 3  # Should have x, y, z coordinates
    assert f_result.shape[0] == len(new_t)
    
    # Check that z-coordinate uses y values from interpolation
    # The function should put y-values in the z-coordinate (index 2)
    # and zeros in the y-coordinate (index 1)
    assert np.all(f_result[:, 1] == 0.0)  # y-coordinate should be zero
    assert len(f_result[:, 2]) > 0  # z-coordinate should have interpolated values


def test_create_mujoco_model():
    """Test MuJoCo XML model creation."""
    pi = ParameterIdentification()
    
    xml = pi.create_mujoco_model(num_joints=3, k_i=0.001, b_i=0.00001)
    
    # Check that XML contains expected elements
    assert "<mujoco>" in xml
    assert "<body" in xml
    assert "<joint" in xml
    assert "<geom" in xml
    assert "stiffness=" in xml
    assert "damping=" in xml
    
    # Check that parameters are included
    assert "0.001" in xml
    assert "1e-05" in xml


def test_csv_loading():
    """Test CSV data loading functionality."""
    pi = ParameterIdentification()
    
    # Create temporary CSV file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        f.write("t,x,y\n")
        f.write("0.0,0.0,0.0\n")
        f.write("0.1,0.1,0.05\n")
        f.write("0.2,0.2,0.1\n")
        csv_path = f.name
    
    try:
        df = pi.load_experimental_data(csv_path)
        
        # Check loaded data
        assert isinstance(df, pd.DataFrame)
        assert len(df) == 3
        assert list(df.columns) == ['t', 'x', 'y']
        assert df.iloc[0]['t'] == 0.0
        assert df.iloc[-1]['t'] == 0.2
    finally:
        os.unlink(csv_path)


def test_mass_calculation():
    """Test mass calculation logic."""
    # Test different GSM values
    pi_light = ParameterIdentification(gsm=80, length=0.1, width=0.1)
    pi_heavy = ParameterIdentification(gsm=200, length=0.1, width=0.1)
    
    assert pi_heavy.mass_paper > pi_light.mass_paper
    assert pi_light.mass_paper == (80 * 0.1 * 0.1) * 0.001
    assert pi_heavy.mass_paper == (200 * 0.1 * 0.1) * 0.001


def test_xml_body_numbering():
    """Test XML body numbering logic."""
    pi = ParameterIdentification()
    
    # Test with different number of joints
    xml_2 = pi.create_mujoco_model(num_joints=2, k_i=0.001, b_i=0.00001)
    xml_5 = pi.create_mujoco_model(num_joints=5, k_i=0.001, b_i=0.00001)
    
    # Should have different number of body elements
    body_count_2 = xml_2.count("<body")
    body_count_5 = xml_5.count("<body")
    
    assert body_count_5 > body_count_2


@pytest.mark.skipif(
    os.environ.get("CI") == "true" or not os.environ.get("DISPLAY"),
    reason="Skipping MuJoCo simulation tests in headless environment"
)
def test_simulation_mock():
    """Mock test for MuJoCo simulation (skipped in CI)."""
    # This test would run actual MuJoCo simulation with proper OpenGL setup
    pass


if __name__ == "__main__":
    pytest.main([__file__])