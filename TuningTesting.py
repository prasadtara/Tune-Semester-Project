# The programme CarTuning is bigger than usual and takes a lot of user input.
# It also deals with threading, plotting and dynamic programming behavior
# This whole thing (Tuning1.py) cannot be tested as it is because of that.
# So, we're doing test cases for testable functions and logic
import unittest
from math import isclose
import sys
import self
sys.stdout = sys.__stdout__

# Importing to test for specific functions:
from Tuning1 import (calculate_atmospheric_pressure, KPA_TO_PSI,
                     calculate_atmospheric_pressure, simulate_engine_data)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class TestEngineSimulator(unittest.TestCase):

    def test_calculate_atmospheric_pressure_sea_level(self):
        print("\n--- Test: Atmospheric Pressure at Sea Level ---")
        result = calculate_atmospheric_pressure(0)
        print(f"Calculated Pressure: {result:.2f} PSI (Expected ≈ 14.7 PSI)")
        self.assertAlmostEqual(result, 14.7, places=1)
        print("Passed: Sea level pressure calculation is correct.")

    def test_calculate_atmospheric_pressure_high_altitude(self):
        print("\n--- Test: Atmospheric Pressure at 4000m Elevation ---")
        result = calculate_atmospheric_pressure(4000)
        print(f"Calculated Pressure: {result:.2f} PSI (Expected ≈ 8.94 PSI)")
        self.assertAlmostEqual(result, 8.94, places=2)
        print("Passed: High altitude pressure calculation is correct.")

    def test_calculate_atmospheric_pressure_negative_elevation(self):
        print("\n--- Test: Atmospheric Pressure at -300m (Below Sea Level) ---")
        result = calculate_atmospheric_pressure(-300)
        print(f"Calculated Pressure: {result:.2f} PSI (Expected ≈ 15.23 PSI)")
        self.assertAlmostEqual(result, 15.23, places=2)
        print("Passed: Negative elevation pressure calculation is correct.")

    def test_kpa_to_psi_constant(self):
        print("\n--- Test: kPa to PSI Conversion Constant ---")
        kpa = 100
        expected_psi = 14.5
        calculated_psi = kpa * KPA_TO_PSI
        print(f"100 kPa = {calculated_psi:.2f} PSI (Expected ≈ {expected_psi} PSI)")
        self.assertTrue(isclose(calculated_psi, expected_psi, abs_tol=0.1))
        print("Passed: kPa to PSI conversion is within acceptable tolerance.")

if __name__ == '__main__':
    unittest.main()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
