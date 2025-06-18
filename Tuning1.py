import threading
import time
import random
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import collections
import datetime

IDLE_RPM = 800
IDLE_MAP = 30
IDLE_TPS = 0

CRUISE_RPM_MIN_DEFAULT = 2000
CRUISE_RPM_MAX_DEFAULT = 3500
CRUISE_MAP_MIN = 40
CRUISE_MAP_MAX = 70
CRUISE_TPS_MIN = 10
CRUISE_TPS_MAX = 30

ACCEL_RPM_MIN = 3000
ACCEL_RPM_MAX = 8000
ACCEL_MAP_NA_MAX = 95
ACCEL_TPS_MIN = 70
ACCEL_TPS_MAX = 100

DECEL_RPM_MIN = 1000
DECEL_RPM_MAX = 2000
DECEL_MAP = 20
DECEL_TPS = 0

BOOST_THRESHOLD_KPA = 105

BASE_NATURALLY_ASPIRATED_PEAK_HP = 0.0
BASE_PEAK_HP_RPM = 0
ASSUMED_NA_WOT_MAP = 95

USER_MAX_BOOST_KPA = 0.0
RPM_AT_MAX_BOOST = 0

simulation_running = True
stop_event = threading.Event()

map_history = collections.deque(np.zeros(100), maxlen=100)
time_history = collections.deque(np.arange(100) * -0.1, maxlen=100)

current_sim_data = {
    "RPM": 0,
    "MAP": 0.0,
    "TPS": 0,
    "BoostStatus": "Waiting...",
    "EstimatedHP": 0.0
}

max_hp_achieved = 0.0
max_rpm_achieved = 0
max_boost_achieved_kpa = 0.0
boost_at_max_hp = 0.0

USER_DEFINED_MAX_RPM_GAUGE = 8000
USER_DEFINED_MAX_MAP_GAUGE = 200

fig, ax_plot = plt.subplots(1, 1, figsize=(10, 6))
fig.set_facecolor('#f0f0f0')


def setup_live_plot(ax):
    ax.set_title("MAP Over Time", fontsize=16)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("MAP (kPa)")
    ax.set_ylim(0, USER_DEFINED_MAX_MAP_GAUGE + 20)
    ax.set_xlim(-10, 0)
    ax.grid(True)

    ax.axhline(BOOST_THRESHOLD_KPA, color='orange', linestyle='--', label=f'Boost Threshold ({BOOST_THRESHOLD_KPA} kPa)')
    ax.legend(loc='upper left')


def update_graphs_for_save():
    ax_plot.clear()
    setup_live_plot(ax_plot)
    ax_plot.plot(time_history, map_history, color='blue', linewidth=2)

    display_max_boost = max_boost_achieved_kpa
    if display_max_boost < BOOST_THRESHOLD_KPA: 
         boost_display_str = "N/A (No Boost Hit)"
    else:
        boost_display_str = f"{display_max_boost:.0f} kPa"

    if max_hp_achieved > BASE_NATURALLY_ASPIRATED_PEAK_HP:
        boost_at_max_hp_str = f" @ {boost_at_max_hp:.0f} kPa"
    else:
        boost_at_max_hp_str = ""
        
    fig.suptitle(f"Simulated ECU Data (Peak Levels)\n"
                 f"Peak HP: {max_hp_achieved:.0f}{boost_at_max_hp_str} | Peak RPM: {max_rpm_achieved} | Highest Boost: {boost_display_str}",
                 fontsize=14, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.9])

def simulate_engine_data(duration_seconds=30):
    global simulation_running, max_hp_achieved, max_rpm_achieved, max_boost_achieved_kpa, HP_UNIT_FACTOR, boost_at_max_hp
    global IDLE_RPM, CRUISE_RPM_MIN, CRUISE_RPM_MAX, ACCEL_RPM_MIN, ACCEL_RPM_MAX, DECEL_RPM_MIN, DECEL_RPM_MAX
    global USER_MAX_BOOST_KPA, RPM_AT_MAX_BOOST, BOOST_THRESHOLD_KPA

    print(f"Simulation started for {duration_seconds} seconds.")
    print(f"Simulation running...")
    current_state = "idle"
    state_duration = 0
    max_state_duration = 5
    start_time = time.time()
    
    max_boost_achieved_kpa = BOOST_THRESHOLD_KPA 

    while time.time() - start_time < duration_seconds and not stop_event.is_set():
        rpm_val, map_val, tps_val = 0, 0, 0

        if current_state == "idle":
            rpm_val = random.randint(IDLE_RPM - 50, IDLE_RPM + 50)
            map_val = random.randint(IDLE_MAP - 5, IDLE_MAP + 5)
            tps_val = IDLE_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "accel_na", "accel_boost"])
                state_duration = 0

        elif current_state == "cruise":
            rpm_val = random.randint(CRUISE_RPM_MIN, CRUISE_RPM_MAX)
            map_val = random.randint(CRUISE_MAP_MIN, CRUISE_MAP_MAX)
            tps_val = random.randint(CRUISE_TPS_MIN, CRUISE_TPS_MAX)
            if state_duration > random.randint(3, max_state_duration + 2):
                current_state = random.choice(["accel_boost", "accel_na", "decel", "idle"])
                state_duration = 0

        elif current_state == "accel_na":
            rpm_val = random.randint(ACCEL_RPM_MIN, min(ACCEL_RPM_MAX, RPM_AT_MAX_BOOST - 500))
            map_val = random.randint(ACCEL_MAP_NA_MAX - 10, ACCEL_MAP_NA_MAX)
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
            
            if RPM_AT_MAX_BOOST > BOOST_THRESHOLD_KPA:
                boost_progress = (rpm_val - BOOST_THRESHOLD_KPA) / (RPM_AT_MAX_BOOST - BOOST_THRESHOLD_KPA)
                boost_progress = max(0, min(1, boost_progress))
                
                target_map = BOOST_THRESHOLD_KPA + boost_progress * (USER_MAX_BOOST_KPA - BOOST_THRESHOLD_KPA)
                
                map_val = int(target_map + random.randint(-5, 5))
                map_val = max(BOOST_THRESHOLD_KPA, min(map_val, USER_MAX_BOOST_KPA + 10))
            else:
                 map_val = random.randint(BOOST_THRESHOLD_KPA, USER_MAX_BOOST_KPA) 
                 if rpm_val < BOOST_THRESHOLD_KPA: 
                     map_val = random.randint(IDLE_MAP, ACCEL_MAP_NA_MAX)

            tps_val = random.randint(ACCEL_TPS_MIN, ACCEL_TPS_MAX)

            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "decel", "idle"])
                state_duration = 0

        elif current_state == "decel":
            rpm_val = random.randint(DECEL_RPM_MIN, DECEL_RPM_MAX)
            map_val = random.randint(DECEL_MAP - 5, DECEL_MAP + 5)
            tps_val = DECEL_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = "idle"
                state_duration = 0

        current_sim_data["RPM"] = rpm_val
        current_sim_data["MAP"] = float(map_val)
        current_sim_data["TPS"] = tps_val

        if map_val > BOOST_THRESHOLD_KPA:
            current_sim_data["BoostStatus"] = "*** BOOST ACTIVE! ***"
        elif map_val > 90:
            current_sim_data["BoostStatus"] = "No Boost (Atmospheric)"
        else:
            current_sim_data["BoostStatus"] = "No Boost (Vacuum)"

        if HP_UNIT_FACTOR == 0 or current_sim_data["RPM"] < 500 or current_sim_data["MAP"] < 20:
            estimated_hp = 0.0
        else:
            estimated_hp = HP_UNIT_FACTOR * current_sim_data["RPM"] * current_sim_data["MAP"]
            estimated_hp = min(estimated_hp, BASE_NATURALLY_ASPIRATED_PEAK_HP * 2.5)

        current_sim_data["EstimatedHP"] = estimated_hp


        if current_sim_data["EstimatedHP"] > max_hp_achieved:
            max_hp_achieved = current_sim_data["EstimatedHP"]
            boost_at_max_hp = current_sim_data["MAP"]
            max_rpm_achieved = current_sim_data["RPM"]

        if current_sim_data["MAP"] > BOOST_THRESHOLD_KPA:
            max_boost_achieved_kpa = max(max_boost_achieved_kpa, current_sim_data["MAP"])

        map_history.append(current_sim_data["MAP"])

        time.sleep(0.1)
        state_duration += 0.1

    print("Simulation finished.")
    simulation_running = False

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

    user_hp = get_float_input("Enter your engine's stock naturally aspirated peak horsepower: ", min_val=1)
    user_rpm = get_int_input(f"Enter the RPM at which your engine hits {user_hp:.0f} HP: ", min_val=1000)

    BASE_NATURALLY_ASPIRATED_PEAK_HP = user_hp
    BASE_PEAK_HP_RPM = user_rpm

    HP_UNIT_FACTOR = BASE_NATURALLY_ASPIRATED_PEAK_HP / (BASE_PEAK_HP_RPM * ASSUMED_NA_WOT_MAP)

    BOOST_THRESHOLD_KPA = get_float_input("Enter the boost pressure (where boost begins) in kPa: ", min_val=80, max_val=120)
    
    USER_MAX_BOOST_KPA = get_float_input("Enter the target peak boost pressure in kPa: ", min_val=BOOST_THRESHOLD_KPA + 10, max_val=300)
    RPM_AT_MAX_BOOST = get_int_input(f"Enter the RPM at which your system reaches {USER_MAX_BOOST_KPA:.0f} kPa boost: ", min_val=user_rpm, max_val=10000)

    IDLE_RPM = get_int_input("Enter idle RPM: ", min_val=500, max_val=1000)
    
    ACCEL_RPM_MAX = get_int_input("Enter maximum RPM for simulation (redline): ", min_val=RPM_AT_MAX_BOOST, max_val=10000)
    ACCEL_RPM_MIN = get_int_input("Enter RPM at which hard acceleration begins: ", min_val=int(user_rpm * 0.8), max_val=RPM_AT_MAX_BOOST - 500)

    CRUISE_RPM_MAX = int(ACCEL_RPM_MIN * 0.9)
    CRUISE_RPM_MIN = int(IDLE_RPM * 1.5)

    DECEL_RPM_MIN = IDLE_RPM + 100
    DECEL_RPM_MAX = ACCEL_RPM_MIN - 100

    USER_DEFINED_MAX_RPM_GAUGE = ACCEL_RPM_MAX + 500
    USER_DEFINED_MAX_MAP_GAUGE = USER_MAX_BOOST_KPA + 50

    setup_live_plot(ax_plot)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    simulation_thread = threading.Thread(target=simulate_engine_data, args=(45,), daemon=True)
    simulation_thread.start()

    while simulation_running:
        update_graphs_for_save()
        fig.canvas.draw()
        time.sleep(0.1)

    update_graphs_for_save()
    fig.canvas.draw()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"engine_simulation.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"Simulation complete. Image saved to {output_filename}")

    plt.close(fig)
