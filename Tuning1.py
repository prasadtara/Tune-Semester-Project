# Import necessary modules
import threading  
import time       
import random     
import numpy as np  
import matplotlib

# Use 'Agg' backend for headless rendering (no GUI)
matplotlib.use('Agg')
import matplotlib.pyplot as plt  
import collections               
import datetime                  

# --- Conversion Constant ---
KPA_TO_PSI = 0.14503773773

# Idling state

IDLE_TPS = 0    # Throttle position sensor %

# Cruise state ranges
CRUISE_RPM_MIN_DEFAULT = 2000
CRUISE_RPM_MAX_DEFAULT = 3500

CRUISE_TPS_MIN = 10
CRUISE_TPS_MAX = 30

# Hard acceleration defaults to 5000
ACCEL_RPM_MIN = 5000
ACCEL_TPS_MIN = 70
ACCEL_TPS_MAX = 100

# Deceleration
DECEL_RPM_MIN_FACTOR = 1.1 # Decel RPM min is slightly above idle
DECEL_RPM_MAX_FACTOR = 0.9 # Decel RPM max is below accel start
DECEL_MAP_KPA = 20 
DECEL_TPS = 0

# Boost threshold - MAP pressure above this indicates boost
# This will be derived from atmospheric pressure
BOOST_THRESHOLD_PSI = 0.0 # Will be calculated

# Base values for upcoming user input
BASE_NATURALLY_ASPIRATED_PEAK_HP = 0.0 # User input
BASE_PEAK_HP_RPM = 0 # Derived
ASSUMED_NA_WOT_MAP_PSI = 0.0 # Will be calculated based on atmospheric pressure

USER_MAX_BOOST_PSI = 0.0 # User input
RPM_AT_MAX_BOOST = 0     # Derived from user input for redline

# Simulation control variables
simulation_running = True
stop_event = threading.Event()

# Maintain last 100 MAP values and timestamps
map_history = collections.deque(np.zeros(100), maxlen=100)
time_history = collections.deque(np.arange(100) * -0.1, maxlen=100)

# Dictionary holding current simulated engine data
current_sim_data = {
    "RPM": 0,
    "MAP": 0.0,
    "TPS": 0,
    "BoostStatus": "Waiting...",
    "EstimatedHP": 0.0
}

# Tracking max values for report
max_hp_achieved = 0.0
max_rpm_achieved = 0
max_boost_achieved_psi = 0.0
boost_at_max_hp = 0.0

# Limits for plotting axes (will be dynamically set based on user input for max boost)
USER_DEFINED_MAX_RPM_GAUGE = 8000
USER_DEFINED_MAX_MAP_GAUGE = 29.0

# Global for atmospheric pressure (will be calculated in PSI)
ATMOSPHERIC_PRESSURE_PSI = 14.7 

# Setup a matplotlib figure and axis
fig, ax_plot = plt.subplots(1, 1, figsize=(10, 6))
fig.set_facecolor('#f0f0f0')

# Configure plot appearance
def setup_live_plot(ax):
    ax.set_title("MAP Over Time", fontsize=16)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("MAP (PSI)")
    ax.set_ylim(0, USER_DEFINED_MAX_MAP_GAUGE + 5)
    ax.set_xlim(-10, 0)
    ax.grid(True)

    ax.axhline(BOOST_THRESHOLD_PSI, color='orange', linestyle='--',
               label=f'Boost Threshold ({BOOST_THRESHOLD_PSI:.1f} PSI)')
    ax.axhline(ATMOSPHERIC_PRESSURE_PSI, color='red', linestyle=':',
               label=f'Atmospheric Pressure ({ATMOSPHERIC_PRESSURE_PSI:.1f} PSI)')
    ax.legend(loc='upper left')

# Redraws the graph with updated data and summary stats
def update_graphs_for_save():
    ax_plot.clear()
    setup_live_plot(ax_plot)
    ax_plot.plot(time_history, map_history, color='blue', linewidth=2)

    # Determine boost text
    display_max_boost = max_boost_achieved_psi
    if display_max_boost < BOOST_THRESHOLD_PSI:
        boost_display_str = "N/A (No Boost Hit)"
    else:
        boost_display_str = f"{display_max_boost:.1f} PSI"

    if max_hp_achieved > 0: # Check if any HP was achieved
        boost_at_max_hp_str = f" @ {boost_at_max_hp:.1f} PSI"
    else:
        boost_at_max_hp_str = ""

    fig.suptitle(f"Simulated ECU Data (Peak Levels)\n"
                 f"Peak HP: {max_hp_achieved:.0f}{boost_at_max_hp_str} | Peak RPM: {max_rpm_achieved} | Highest Boost: {boost_display_str}\n"
                 f"Atmospheric Pressure (at elevation): {ATMOSPHERIC_PRESSURE_PSI:.1f} PSI",
                 fontsize=14, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.9])


# Core simulation function
def simulate_engine_data(duration_seconds=30):
    global simulation_running, max_hp_achieved, max_rpm_achieved, max_boost_achieved_psi, HP_UNIT_FACTOR, boost_at_max_hp
    global IDLE_RPM, IDLE_MAP, CRUISE_RPM_MIN, CRUISE_RPM_MAX, CRUISE_MAP_MIN, CRUISE_MAP_MAX, ACCEL_RPM_MIN, ACCEL_RPM_MAX
    global USER_MAX_BOOST_PSI, RPM_AT_MAX_BOOST, BOOST_THRESHOLD_PSI, ATMOSPHERIC_PRESSURE_PSI, ASSUMED_NA_WOT_MAP_PSI
    global DECEL_RPM_MIN, DECEL_RPM_MAX, DECEL_MAP_KPA, ACCEL_MAP_NA_MAX, BASE_PEAK_HP_RPM

    print(f"Simulation started for {duration_seconds} seconds.")
    print(f"Simulation running...")
    current_state = "idle"
    state_duration = 0
    max_state_duration = 5 # Max time before switching states
    start_time = time.time()

    max_boost_achieved_psi = BOOST_THRESHOLD_PSI

    # Calibrate initial MAP values based on atmospheric pressure
    IDLE_MAP = ATMOSPHERIC_PRESSURE_PSI * 0.3
    CRUISE_MAP_MIN = ATMOSPHERIC_PRESSURE_PSI * 0.4
    CRUISE_MAP_MAX = ATMOSPHERIC_PRESSURE_PSI * 0.7
    ACCEL_MAP_NA_MAX = ATMOSPHERIC_PRESSURE_PSI * 0.95
    ASSUMED_NA_WOT_MAP_PSI = ACCEL_MAP_NA_MAX

    # Assuming naturally aspirated peak HP occurs around 80% of redline
    BASE_PEAK_HP_RPM = int(ACCEL_RPM_MAX * 0.8)
    if BASE_PEAK_HP_RPM < ACCEL_RPM_MIN: 
        BASE_PEAK_HP_RPM = ACCEL_RPM_MIN + 500 

    # HP_UNIT_FACTOR is a calibration factor for estimated HP
    if BASE_PEAK_HP_RPM > 0 and ASSUMED_NA_WOT_MAP_PSI > 0:
        HP_UNIT_FACTOR = BASE_NATURALLY_ASPIRATED_PEAK_HP / (BASE_PEAK_HP_RPM * ASSUMED_NA_WOT_MAP_PSI)
    else:
        HP_UNIT_FACTOR = 0

    # DECEL_MAP PSI
    DECEL_MAP_PSI = DECEL_MAP_KPA * KPA_TO_PSI

    while time.time() - start_time < duration_seconds and not stop_event.is_set():
        # Default values for each loop iteration
        rpm_val, map_val, tps_val = 0, 0, 0

        # Simulate behavior for each engine state
        if current_state == "idle":
            rpm_val = random.randint(IDLE_RPM - 50, IDLE_RPM + 50)
            map_val = random.uniform(IDLE_MAP - 0.5, IDLE_MAP + 0.5)
            tps_val = IDLE_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "accel_na", "accel_boost"])
                state_duration = 0

        elif current_state == "cruise":
            rpm_val = random.randint(CRUISE_RPM_MIN, CRUISE_RPM_MAX)
            map_val = random.uniform(CRUISE_MAP_MIN - 0.5, CRUISE_MAP_MAX + 0.5)
            tps_val = random.randint(CRUISE_TPS_MIN, CRUISE_TPS_MAX)
            if state_duration > random.randint(3, max_state_duration + 2):
                current_state = random.choice(["accel_boost", "accel_na", "decel", "idle"])
                state_duration = 0

        elif current_state == "accel_na":
            rpm_val = random.randint(ACCEL_RPM_MIN, min(ACCEL_RPM_MAX, RPM_AT_MAX_BOOST - 500))
            map_val = random.uniform(ACCEL_MAP_NA_MAX - 0.5, ACCEL_MAP_NA_MAX + 0.5)
            tps_val = random.randint(ACCEL_TPS_MIN, ACCEL_TPS_MAX)
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "decel", "accel_boost", "idle"])
                state_duration = 0

        elif current_state == "accel_boost":
            target_rpm = random.randint(ACCEL_RPM_MIN, ACCEL_RPM_MAX)

            if current_sim_data["RPM"] < target_rpm:
                rpm_val = min(current_sim_data["RPM"] + random.randint(100, 300), ACCEL_RPM_MAX)
            else:
                rpm_val = random.randint(target_rpm - 100, target_rpm + 100)

            if RPM_AT_MAX_BOOST > ACCEL_RPM_MIN:
                # Scale boost based on RPM from ACCEL_RPM_MIN to RPM_AT_MAX_BOOST
                boost_progress = (rpm_val - ACCEL_RPM_MIN) / (RPM_AT_MAX_BOOST - ACCEL_RPM_MIN)
                boost_progress = max(0, min(1, boost_progress))

                
                target_map = ATMOSPHERIC_PRESSURE_PSI + boost_progress * (USER_MAX_BOOST_PSI - ATMOSPHERIC_PRESSURE_PSI)
                target_map = max(ACCEL_MAP_NA_MAX, target_map) 
            else: 
                target_map = USER_MAX_BOOST_PSI 

            map_val = random.uniform(target_map - 0.5, target_map + 0.5)
            map_val = max(ATMOSPHERIC_PRESSURE_PSI * 0.1, min(map_val, USER_MAX_BOOST_PSI + 1.0))

            tps_val = random.randint(ACCEL_TPS_MIN, ACCEL_TPS_MAX)

            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "decel", "idle"])
                state_duration = 0

        elif current_state == "decel":
            rpm_val = random.randint(DECEL_RPM_MIN, DECEL_RPM_MAX)
            map_val = random.uniform(DECEL_MAP_PSI - 0.5, DECEL_MAP_PSI + 0.5)
            tps_val = DECEL_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = "idle"
                state_duration = 0

        
        current_sim_data["RPM"] = rpm_val
        current_sim_data["MAP"] = float(map_val)
        current_sim_data["TPS"] = tps_val

        
        if map_val > BOOST_THRESHOLD_PSI:
            current_sim_data["BoostStatus"] = "*** BOOST ACTIVE! ***"
        elif map_val > (ATMOSPHERIC_PRESSURE_PSI * 0.9):
            current_sim_data["BoostStatus"] = "No Boost (Atmospheric)"
        else:
            current_sim_data["BoostStatus"] = "No Boost (Vacuum)"

        # Estimate HP based on MAP and RPM
        if HP_UNIT_FACTOR == 0 or current_sim_data["RPM"] < 500 or current_sim_data["MAP"] < 2:
            estimated_hp = 0.0
        else:
            estimated_hp = HP_UNIT_FACTOR * current_sim_data["RPM"] * current_sim_data["MAP"]
            estimated_hp = min(estimated_hp, BASE_NATURALLY_ASPIRATED_PEAK_HP * 2.5) # Cap max HP for realism

        current_sim_data["EstimatedHP"] = estimated_hp

        # Track peak stats
        if current_sim_data["EstimatedHP"] > max_hp_achieved:
            max_hp_achieved = current_sim_data["EstimatedHP"]
            boost_at_max_hp = current_sim_data["MAP"]
            max_rpm_achieved = current_sim_data["RPM"]

        if current_sim_data["MAP"] > BOOST_THRESHOLD_PSI:
            max_boost_achieved_psi = max(max_boost_achieved_psi, current_sim_data["MAP"])

        # Append MAP to plot history
        map_history.append(current_sim_data["MAP"])

        time.sleep(0.1)
        state_duration += 0.1

    print("Simulation finished.")
    simulation_running = False

# Function to calculate atmospheric pressure based on elevation (Barometric Formula)
def calculate_atmospheric_pressure(elevation_m):
    P0_kpa = 101.325  
    L = 0.0065    
    T0 = 288.15   
    g = 9.80665   
    M = 0.0289644 
    R = 8.31447   

    if (L * elevation_m) >= T0:
        return 0.0

    pressure_kpa = P0_kpa * (1 - L * elevation_m / T0)**(g * M / (R * L))
    return pressure_kpa * KPA_TO_PSI # Return in PSI

# -- Main script execution starts here --
if __name__ == "__main__":
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

    def get_int_input(prompt, min_val=None, max_val=None):
        return int(get_float_input(prompt, min_val, max_val))

    # User Inputs
    user_elevation_m = get_float_input("Enter elevation (meters): ", min_val=-400, max_val=8000)
    ATMOSPHERIC_PRESSURE_PSI = calculate_atmospheric_pressure(user_elevation_m)
    print(f"Estimated atmospheric pressure at {user_elevation_m} meters: {ATMOSPHERIC_PRESSURE_PSI:.2f} PSI")

    BASE_NATURALLY_ASPIRATED_PEAK_HP = get_float_input("Enter naturally aspirated peak horsepower: ", min_val=1)
    USER_MAX_BOOST_PSI = get_float_input("Enter target peak boost pressure (PSI): ",
                                         min_val=ATMOSPHERIC_PRESSURE_PSI + 2, # Must be higher than atmospheric + a little
                                         max_val=45) # Reasonable max boost in PSI

    ACCEL_RPM_MAX = get_int_input("Enter redline RPM: ", min_val=5000, max_val=10000)
    IDLE_RPM = get_int_input("Enter idle RPM: ", min_val=500, max_val=1000)


    BOOST_THRESHOLD_PSI = ATMOSPHERIC_PRESSURE_PSI + 0.7 
    RPM_AT_MAX_BOOST = int(ACCEL_RPM_MAX * 0.9) 
    if RPM_AT_MAX_BOOST < ACCEL_RPM_MIN: 
        RPM_AT_MAX_BOOST = ACCEL_RPM_MIN + 500


    CRUISE_RPM_MAX = int(ACCEL_RPM_MIN * 0.9)
    CRUISE_RPM_MIN = int(IDLE_RPM * 1.5)

    DECEL_RPM_MIN = IDLE_RPM + 100
    DECEL_RPM_MAX = ACCEL_RPM_MIN - 100

    USER_DEFINED_MAX_RPM_GAUGE = ACCEL_RPM_MAX + 500
    USER_DEFINED_MAX_MAP_GAUGE = USER_MAX_BOOST_PSI + 7.5

    setup_live_plot(ax_plot)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    simulation_thread = threading.Thread(target=simulate_engine_data, args=(45,), daemon=True)
    simulation_thread.start()

    while simulation_running:
        update_graphs_for_save()
        fig.canvas.draw()
        plt.pause(0.05)
        time.sleep(0.05)

    update_graphs_for_save()
    fig.canvas.draw()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"engine_simulation_{timestamp}.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"Simulation complete. Image saved to {output_filename}")

    plt.close(fig)
