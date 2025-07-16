# Import necessary libraries
import threading     # To run engine simulation on separate thread
import time          # For timing and delays
import random        # To simulate dynamic engine variability
import numpy as np   # For numerical operations and initializing data arrays
import matplotlib    # For graph plotting

# Set the backend for matplotlib to 'Agg',
# allowing plots to be saved without a GUI window (headless mode)
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # Used for plotting MAP (Manifold Absolute Pressure) graph
import collections               # Provides deque for efficient rolling window of data
import datetime                 # Used for timestamping saved plot files

# --- Constants and User Inputs ---

# Kilopascal to Pound-Force per Square Inch
KPA_TO_PSI = 0.14503773773 # Conversion factor: 1 kPa = ~0.145 PSI

# --- Engine State Parameters ---
# IDLE state: throttle is closed
IDLE_TPS = 0  # Throttle Position Sensor reading (%)

# CRUISE state: mid-range RPMs and moderate throttle
CRUISE_RPM_MIN_DEFAULT = 2000
CRUISE_RPM_MAX_DEFAULT = 3500
CRUISE_TPS_MIN = 10
CRUISE_TPS_MAX = 30

# ACCELERATION state: user-defined RPM and full-throttle
ACCEL_RPM_MIN = 5000
ACCEL_TPS_MIN = 70
ACCEL_TPS_MAX = 100

# DECELERATION state: low throttle and declining RPM
# Factors are used to dynamically calculate these values based on user input
DECEL_RPM_MIN_FACTOR = 1.1  # Slightly above idle
DECEL_RPM_MAX_FACTOR = 0.9  # Slightly below acceleration threshold
DECEL_MAP_KPA = 20          # Manifold pressure during deceleration (vacuum)
DECEL_TPS = 0               # Throttle closed during decel

# --- Boost Logic ---
# Boost begins above atmospheric pressure (user input + elevation adjustment)
BOOST_ACTIVE_PSI = 0.0  # Will be computed from user-defined atmospheric pressure


# --- User Input Placeholders ---

# Stock engine horsepower (HP) (no boost)
BASE_NATURALLY_ASPIRATED_PEAK_HP = 0.0
# RPM at which that HP is achieved
BASE__HP_RPM = 0
# Estimated MAP during NA wide-open throttle (will be derived)
ASSUMED_NA_WOT_MAP_PSI = 0.0

# Peak boost pressure user wants to simulate
USER_MAX_BOOST_PSI = 0.0
# RPM where peak boost is reached
RPM_AT_MAX_BOOST = 0

# --- Simulation Control ---

simulation_running = True   # Main loop control flag
stop_event = threading.Event()  # Thread-safe stop flag

# --- Data Tracking for Visualization ---

# Store last 100 MAP values and timestamps for plotting
map_history = collections.deque(np.zeros(450), maxlen=450)
# From -45 to 0 seconds (0.1s intervals)
time_history = collections.deque(np.arange(450) * -0.1, maxlen=450)

# Dictionary holding the live simulation state (to be updated by simulation loop)
current_sim_data = {
    "RPM": 0,
    "MAP": 0.0, # Manifold Absolute Pressure (in PSI)
    "TPS": 0,   # Throttle Position (%)
    "BoostStatus": "Waiting...",
    "EstimatedHP": 0.0  # Calculated power based on RPM and MAP
}

# Track maximum performance values during simulation
max_hp_achieved = 0.0
max_rpm_achieved = 0
max_boost_achieved_psi = 0.0
# Boost level at which peak HP occurred
boost_at_max_hp = 0.0

# --- Visualization Scaling (Adjusted Based on User Inputs) ---

USER_DEFINED_MAX_RPM_GAUGE = 8000     # Top RPM on graph
USER_DEFINED_MAX_MAP_GAUGE = 29.0     # Top MAP (PSI) on graph

# Atmospheric pressure at user's elevation (default = sea level = 14.7 PSI)
ATMOSPHERIC_PRESSURE_PSI = 14.7


# --- Matplotlib Setup ---

# Create a new figure and axis for the plot
fig, ax_plot = plt.subplots(1, 1, figsize=(10, 6))
# Set light gray background for readability
fig.set_facecolor('#f0f0f0')


# --- Graph Setup Function ---
def setup_live_plot(ax):
    # Configures the appearance of the live MAP graph.
    # Adds threshold lines and labels for boost threshold and atmospheric pressure.
    ax.set_title("MAP Over Time", fontsize=16)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("MAP (PSI)")
    ax.set_ylim(0, USER_DEFINED_MAX_MAP_GAUGE + 5)
    ax.set_xlim(-45, 0)
    ax.grid(True)

    # Orange dashed line = boost threshold
    ax.axhline(USER_MAX_BOOST_PSI, color='orange', linestyle='--',
               label=f'Maximum Boost: {USER_MAX_BOOST_PSI:.1f} PSI')

    # Red dotted line = atmospheric pressure reference
    ax.axhline(ATMOSPHERIC_PRESSURE_PSI, color='red', linestyle=':',
               label=f'Atmospheric Pressure: {ATMOSPHERIC_PRESSURE_PSI:.1f} PSI')

    ax.legend(loc='upper left')


# --- Graph Update Function ---
def update_graphs_for_save():
    # Clears and redraws the MAP graph with current simulation data.
    # Also adds a summary of peak HP, RPM, and boost values to the plot title.
    ax_plot.clear()
    setup_live_plot(ax_plot)

    # Plot the current MAP over time
    ax_plot.plot(time_history, map_history, color='blue', linewidth=2)

    # Determine boost summary for title
    display_max_boost = max_boost_achieved_psi
    if display_max_boost < BOOST_ACTIVE_PSI:
        boost_display_str = "N/A (No Boost Hit)"
    else:
        boost_display_str = f"{display_max_boost:.1f} PSI"

    # Show boost level at peak HP, if any HP was achieved
    boost_at_max_hp_str = f" @ {boost_at_max_hp:.1f} PSI" if max_hp_achieved > 0 else ""

    # Title includes key stats and reference pressure
    fig.suptitle(
        f"Simulated ECU Data (Peak Levels)\n"
        f"Peak HP: {max_hp_achieved:.0f}{boost_at_max_hp_str} | "
        f"Peak RPM: {max_rpm_achieved} | "
        f"Highest Boost: {boost_display_str}\n"
        f"Atmospheric Pressure (at elevation): {ATMOSPHERIC_PRESSURE_PSI:.1f} PSI",
        fontsize=14,
        y=0.98
    )

    fig.tight_layout(rect=[0, 0, 1, 0.9])  # Leave space for title


# --- Core Engine Simulation Function ---
def simulate_engine_data(duration_seconds=30):
    # Simulates engine behavior under various throttle and RPM conditions,
    # mimicking idle, cruising, acceleration (with or without boost), and deceleration states.
    # Runs for `duration_seconds` or until manually stopped.

    # Declare shared variables for simulation and stats tracking
    global simulation_running, max_hp_achieved, max_rpm_achieved, \
        max_boost_achieved_psi, HP_UNIT_FACTOR, boost_at_max_hp
    global IDLE_RPM, IDLE_MAP, CRUISE_RPM_MIN, CRUISE_RPM_MAX, \
        CRUISE_MAP_MIN, CRUISE_MAP_MAX, ACCEL_RPM_MIN, ACCEL_RPM_MAX
    global USER_MAX_BOOST_PSI, RPM_AT_MAX_BOOST, BOOST_ACTIVE_PSI, \
        ATMOSPHERIC_PRESSURE_PSI, ASSUMED_NA_WOT_MAP_PSI
    global DECEL_RPM_MIN, DECEL_RPM_MAX, DECEL_MAP_KPA, ACCEL_MAP_NA_MAX, BASE_PEAK_HP_RPM

    # Inform user that simulation has started
    print(f"Simulation started for {duration_seconds} seconds.")
    print(f"Simulation running...")

    # --- Initialize Simulation State ---
    current_state = "idle"   # Start in idle state
    state_duration = 0       # Time spent in current state
    max_state_duration = 5   # Max time to remain in any one state
    start_time = time.time() # Start the simulation timer

    # Reset max boost tracker to boost threshold
    max_boost_achieved_psi = BOOST_ACTIVE_PSI

    # --- Calibrate MAP values based on atmospheric pressure ---
    IDLE_MAP = ATMOSPHERIC_PRESSURE_PSI * 0.3
    CRUISE_MAP_MIN = ATMOSPHERIC_PRESSURE_PSI * 0.4
    CRUISE_MAP_MAX = ATMOSPHERIC_PRESSURE_PSI * 0.7
    ACCEL_MAP_NA_MAX = ATMOSPHERIC_PRESSURE_PSI * 0.95
    ASSUMED_NA_WOT_MAP_PSI = ACCEL_MAP_NA_MAX  # Used in HP estimation

    # Estimate where peak NA horsepower happens — roughly 80% of redline
    BASE_PEAK_HP_RPM = int(ACCEL_RPM_MAX * 0.8)
    if BASE_PEAK_HP_RPM < ACCEL_RPM_MIN:
        BASE_PEAK_HP_RPM = ACCEL_RPM_MIN + 500

    # --- Calculate HP Unit Factor ---
    # This factor translates MAP and RPM into estimated horsepower
    if BASE_PEAK_HP_RPM > 0 and ASSUMED_NA_WOT_MAP_PSI > 0:
        HP_UNIT_FACTOR = (BASE_NATURALLY_ASPIRATED_PEAK_HP /
                          (BASE_PEAK_HP_RPM * ASSUMED_NA_WOT_MAP_PSI))
    else:
        HP_UNIT_FACTOR = 0

    # Convert deceleration MAP from kPa to PSI
    DECEL_MAP_PSI = DECEL_MAP_KPA * KPA_TO_PSI

    # --- Main Simulation Loop ---
    while time.time() - start_time < duration_seconds and not stop_event.is_set():
        # Default sensor values for each loop iteration
        rpm_val, map_val, tps_val = 0, 0, 0

        # ----- Simulate Each Engine State -----

        # Idle State
        if current_state == "idle":
            rpm_val = random.randint(IDLE_RPM - 50, IDLE_RPM + 50)
            map_val = random.uniform(IDLE_MAP - 0.5, IDLE_MAP + 0.5)
            tps_val = IDLE_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "accel_na", "accel_boost"])
                state_duration = 0

        # Cruise State
        elif current_state == "cruise":
            rpm_val = random.randint(CRUISE_RPM_MIN, CRUISE_RPM_MAX)
            map_val = random.uniform(CRUISE_MAP_MIN - 0.5, CRUISE_MAP_MAX + 0.5)
            tps_val = random.randint(CRUISE_TPS_MIN, CRUISE_TPS_MAX)
            if state_duration > random.randint(3, max_state_duration + 2):
                current_state = random.choice(["accel_boost", "accel_na", "decel", "idle"])
                state_duration = 0

        # Naturally Aspirated Acceleration (No Boost)
        elif current_state == "accel_na":
            rpm_val = random.randint(ACCEL_RPM_MIN, min(ACCEL_RPM_MAX, RPM_AT_MAX_BOOST - 500))
            map_val = random.uniform(ACCEL_MAP_NA_MAX - 0.5, ACCEL_MAP_NA_MAX + 0.5)
            tps_val = random.randint(ACCEL_TPS_MIN, ACCEL_TPS_MAX)
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "decel", "accel_boost", "idle"])
                state_duration = 0

        # Boosted Acceleration
        elif current_state == "accel_boost":
            target_rpm = random.randint(ACCEL_RPM_MIN, ACCEL_RPM_MAX)

            # Simulate climbing RPM towards boost target
            if current_sim_data["RPM"] < target_rpm:
                rpm_val = min(current_sim_data["RPM"] +
                              random.randint(100, 300), ACCEL_RPM_MAX)
            else:
                rpm_val = random.randint(target_rpm - 100, target_rpm + 100)

            # Calculate boost MAP dynamically based on RPM climb
            if RPM_AT_MAX_BOOST > ACCEL_RPM_MIN:
                boost_progress = (rpm_val - ACCEL_RPM_MIN) / (RPM_AT_MAX_BOOST - ACCEL_RPM_MIN)
                boost_progress = max(0, min(1, boost_progress))  # Clamp to [0,1]

                target_map = (ATMOSPHERIC_PRESSURE_PSI + boost_progress *
                              (USER_MAX_BOOST_PSI - ATMOSPHERIC_PRESSURE_PSI))
                target_map = max(ACCEL_MAP_NA_MAX, target_map)
            else:
                target_map = USER_MAX_BOOST_PSI

            # Add random variation to MAP for realism
            map_val = random.uniform(target_map - 0.5, target_map + 0.5)
            map_val = max(ATMOSPHERIC_PRESSURE_PSI * 0.1, min(map_val, USER_MAX_BOOST_PSI))
    


            tps_val = random.randint(ACCEL_TPS_MIN, ACCEL_TPS_MAX)

            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "decel", "idle"])
                state_duration = 0

        # Deceleration
        elif current_state == "decel":
            rpm_val = random.randint(DECEL_RPM_MIN, DECEL_RPM_MAX)
            map_val = random.uniform(DECEL_MAP_PSI - 0.5, DECEL_MAP_PSI + 0.5)
            tps_val = DECEL_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = "idle"
                state_duration = 0

        # Update Current Simulation Data

        current_sim_data["RPM"] = rpm_val
        current_sim_data["MAP"] = float(map_val)
        current_sim_data["TPS"] = tps_val

        # Estimate Horsepower (if MAP & RPM are sufficient)
        if HP_UNIT_FACTOR == 0 or current_sim_data["RPM"] < 500 or current_sim_data["MAP"] < 2:
            estimated_hp = 0.0
        else:
            estimated_hp = HP_UNIT_FACTOR * current_sim_data["RPM"] * current_sim_data["MAP"]
            estimated_hp = min(estimated_hp, BASE_NATURALLY_ASPIRATED_PEAK_HP * 2.5)  # Cap extreme output

        current_sim_data["EstimatedHP"] = estimated_hp

        # Track Maximum Stats
        if estimated_hp > max_hp_achieved:
            max_hp_achieved = estimated_hp
            boost_at_max_hp = map_val
            max_rpm_achieved = rpm_val

        if map_val > BOOST_ACTIVE_PSI:
            max_boost_achieved_psi = max(max_boost_achieved_psi, map_val)

        # Append to MAP History for Plotting
        map_history.append(map_val)

        # Wait 0.1s before next sample
        time.sleep(0.1)
        state_duration += 0.1

    print("Simulation finished.")
    simulation_running = False


# Function to calculate atmospheric pressure based on elevation (Barometric Formula)
def calculate_atmospheric_pressure(elevation_m):
    # Constants used in the barometric formula (SI units)
    P0_kpa = 101.325  # Sea level standard pressure in kilopascals
    L = 0.0065     # Temperature lapse rate (K/m)
    T0 = 288.15    # Standard temperature at sea level in Kelvin
    g = 9.80665    # Gravitational acceleration (m/s^2)
    M = 0.0289644  # Molar mass of Earth's air (kg/mol)
    R = 8.31447    # Universal gas constant (J/(mol·K))

    # If elevation is unrealistically high,
    # return 0 pressure to avoid math domain error
    if (L * elevation_m) >= T0:
        return 0.0
    # Apply the barometric formula to get pressure at given elevation in kPa
    pressure_kpa = P0_kpa * (1 - L * elevation_m / T0) ** (g * M / (R * L))
    # Convert kPa to PSI using the conversion constant and return
    return pressure_kpa * KPA_TO_PSI


# -- Main script execution starts here --
if __name__ == "__main__":

    # Function to get validated float input from the user
    def get_float_input(prompt, min_val=None, max_val=None):
        while True:
            try:
                val = float(input(prompt))
                if min_val is not None and val < min_val:
                    print(f"Value must be at least {min_val}. Try again.")
                    continue
                if max_val is not None and val > max_val:
                    print(f"Value must be no more than {max_val}. Try again.")
                    continue
                return val
            except ValueError:
                print("Invalid input. Please enter a number.")

    # Function to get validated integer input from the user (wraps float input for reuse)
    def get_int_input(prompt, min_val=None, max_val=None):
        return int(get_float_input(prompt, min_val, max_val))

    print("***Welcome To Your Engine Performance Simulator!***")

    # Ask for elevation to determine base atmospheric pressure
    user_elevation_m = get_float_input(
        "Enter elevation (meters): ", min_val=-400, max_val=8000)
    ATMOSPHERIC_PRESSURE_PSI = calculate_atmospheric_pressure(user_elevation_m)
    print(f"Estimated atmospheric pressure at {user_elevation_m} meters: "
          f"{ATMOSPHERIC_PRESSURE_PSI:.2f} PSI")

    # Ask for peak HP to calibrate the simulation’s power output calculations
    BASE_NATURALLY_ASPIRATED_PEAK_HP = get_float_input(
        "Enter naturally aspirated peak horsepower: ", min_val=1)

    # Ask for max boost pressure the engine can achieve
    # (must be higher than atmospheric + 2 PSI)
    USER_MAX_BOOST_PSI = get_float_input("Enter target peak boost pressure (PSI):\n"
                                         "*To find target peak boost PSI, add your boost system's\n" 
                                         "pressure to the atmospheric pressure. For example, if\n"
                                         "the atmospheric pressure is 10 PSI and your boost\n"
                                         "system provides 6 PSI, the target peak boost pressure would\n" 
                                         "be 10+6=16 PSI.*",
                                         min_val=round(ATMOSPHERIC_PRESSURE_PSI),
                                         max_val=45)
    # Ask for engine redline RPM
    ACCEL_RPM_MAX = get_int_input("Enter redline RPM: ", min_val=5000, max_val=10000)
    # Ask for engine idle RPM
    IDLE_RPM = get_int_input("Enter idle RPM: ", min_val=500, max_val=1000)

    # Derived Configuration Based on User Input
    BOOST_ACTIVE_PSI = ATMOSPHERIC_PRESSURE_PSI  # Pressure at which boost is active
    # Boost should reach its max at ~90% redline
    RPM_AT_MAX_BOOST = int(ACCEL_RPM_MAX * 0.9)
    if RPM_AT_MAX_BOOST < ACCEL_RPM_MIN:
        # Ensure boost RPM is realistic
        RPM_AT_MAX_BOOST = ACCEL_RPM_MIN + 500

    # Define RPM ranges for cruise and decel based on idle and accel thresholds
    CRUISE_RPM_MAX = int(ACCEL_RPM_MIN * 0.9)
    CRUISE_RPM_MIN = int(IDLE_RPM * 1.5)

    DECEL_RPM_MIN = IDLE_RPM + 100
    DECEL_RPM_MAX = ACCEL_RPM_MIN - 100

    # Set max values for graph axes to ensure complete visibility
    USER_DEFINED_MAX_RPM_GAUGE = ACCEL_RPM_MAX + 500
    USER_DEFINED_MAX_MAP_GAUGE = USER_MAX_BOOST_PSI + 7.5

    # Setup and Launch Simulation
    # Configure the plot
    setup_live_plot(ax_plot)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    # Launch the engine simulation in a background thread for responsiveness
    simulation_thread = threading.Thread(target=simulate_engine_data, args=(45,), daemon=True)
    simulation_thread.start()
    # Update the plot live while simulation is running
    while simulation_running:
        update_graphs_for_save()
        fig.canvas.draw()
        time.sleep(0.05)
    # Final update after simulation ends
    update_graphs_for_save()
    fig.canvas.draw()

    # Save the final simulation graph to png
    output_filename = f"engine_simulation.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"Simulation complete. Image saved to {output_filename}")

    # Close the figure window to clean up resources
    plt.close(fig)
