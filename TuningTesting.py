# The programme CarTuning is bigger than usual and takes a lot of user input.
# It also deals with threading, plotting and dynamic programming behavior
# This whole thing (Tuning1.py) cannot be tested as it is because of that.
# So, we're doing test cases for testable functions and logic

# Python's built-in testing framework
import unittest
# Used to compare floating point values with a tolerance
from math import isclose
import sys
# Ensures standard output is reset in environments like PyCharm
sys.stdout = sys.__stdout__

# Import specific testable components from the simulator code
from Tuning1 import (
    # Function to calculate atmospheric pressure based on elevation
    calculate_atmospheric_pressure,
    # Constant for converting kPa to PSI
    KPA_TO_PSI,
    # Core simulation function
    simulate_engine_data
)

# ---------------------------------------------------------------

# Unit tests for verifying key components of the engine simulation logic.
# Focused on atmospheric pressure calculation and constant values.
class TestEngineSimulator(unittest.TestCase):

    def test_calculate_atmospheric_pressure_sea_level(self):
        # Test atmospheric pressure at sea level (0 meters).
        # Expected pressure ≈ 14.7 PSI (standard atmospheric pressure).
        print("\n--- Test: Atmospheric Pressure at Sea Level ---")
        result = calculate_atmospheric_pressure(0)
        print(f"Calculated Pressure: {result:.2f} PSI (Expected ≈ 14.7 PSI)")
        self.assertAlmostEqual(result, 14.7, places=1)  # Allow ±0.1 tolerance
        print("Passed: Sea level pressure calculation is correct.")

    def test_calculate_atmospheric_pressure_high_altitude(self):
        # Test atmospheric pressure at a high elevation (4000 meters).
        # Atmospheric pressure should significantly drop at high altitudes.
        print("\n--- Test: Atmospheric Pressure at 4000m Elevation ---")
        result = calculate_atmospheric_pressure(4000)
        print(f"Calculated Pressure: {result:.2f} PSI (Expected ≈ 8.94 PSI)")
        self.assertAlmostEqual(result, 8.94, places=2)  # Compare with expected drop in pressure
        print("Passed: High altitude pressure calculation is correct.")

    def test_calculate_atmospheric_pressure_negative_elevation(self):
        # Test atmospheric pressure below sea level (-300 meters).
        # Pressure should increase slightly due to denser air at lower elevations.
        print("\n--- Test: Atmospheric Pressure at -300m (Below Sea Level) ---")
        result = calculate_atmospheric_pressure(-300)
        print(f"Calculated Pressure: {result:.2f} PSI (Expected ≈ 15.23 PSI)")
        self.assertAlmostEqual(result, 15.23, places=2)
        print("Passed: Negative elevation pressure calculation is correct.")

    def test_kpa_to_psi_constant(self):
        # Verify the accuracy of the KPA_TO_PSI conversion constant.
        # 100 kPa should be approximately 14.5 PSI.
        print("\n--- Test: kPa to PSI Conversion Constant ---")
        kpa = 100
        expected_psi = 14.5
        calculated_psi = kpa * KPA_TO_PSI
        print(f"100 kPa = {calculated_psi:.2f} PSI (Expected ≈ {expected_psi} PSI)")
        self.assertTrue(isclose(calculated_psi, expected_psi, abs_tol=0.1))  # Allow ±0.1 PSI tolerance
        print("Passed: kPa to PSI conversion is within acceptable tolerance.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# This block runs all tests when this file is executed directly
if __name__ == '__main__':
    unittest.main()

# ---------------------------------------------------------------
