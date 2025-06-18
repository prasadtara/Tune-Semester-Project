import threading
import time
import random
import numpy as np
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import collections


IDLE_RPM = 800
IDLE_MAP = 30 
IDLE_TPS = 0

CRUISE_RPM_MIN = 2000
CRUISE_RPM_MAX = 3500
CRUISE_MAP_MIN = 40
CRUISE_MAP_MAX = 70 
CRUISE_TPS_MIN = 10
CRUISE_TPS_MAX = 30

ACCEL_RPM_MIN = 3000
ACCEL_RPM_MAX = 8000 
ACCEL_MAP_NA_MAX = 95 
ACCEL_MAP_BOOST_MIN = 105 
ACCEL_MAP_BOOST_MAX = 200 
ACCEL_TPS_MIN = 70
ACCEL_TPS_MAX = 100

DECEL_RPM_MIN = 1000
DECEL_RPM_MAX = 2000
DECEL_MAP = 20 
DECEL_TPS = 0

BOOST_THRESHOLD_KPA = 105


simulation_running = True
stop_event = threading.Event()


map_history = collections.deque(np.zeros(100), maxlen=100) 
time_history = collections.deque(np.arange(100) * -0.1, maxlen=100) 


current_sim_data = {
    "RPM": 0,
    "MAP": 0.0,
    "TPS": 0,
    "BoostStatus": "Waiting..."
}


fig, (ax_rpm, ax_map, ax_plot) = plt.subplots(1, 3, figsize=(15, 6)) # One row, three columns
fig.set_facecolor('#f0f0f0')


def draw_gauge(ax, value, min_val, max_val, title, unit, boost_threshold=None):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-0.1, 1.1) 

    
    ax.add_patch(Arc((0,0), 2, 2, angle=0, theta1=180, theta2=0, linewidth=8, color='lightgray'))

    
    if max_val - min_val > 0:
        angle_ratio = (value - min_val) / (max_val - min_val)
    else:
        angle_ratio = 0.5

    angle = 180 - (angle_ratio * 180) 
    angle = max(0, min(180, angle))

    rad = np.deg2rad(angle)
    x_tip = np.cos(rad) * 0.9
    y_tip = np.sin(rad) * 0.9
    ax.plot([0, x_tip], [0, y_tip], color='red', linewidth=3) # Needle

    
    ax.text(0, 0.4, f"{value:.0f} {unit}", ha='center', va='center', fontsize=20, weight='bold')
    ax.text(0, 0.9, title, ha='center', va='center', fontsize=16)

    
    for i in range(5):
        val = min_val + (max_val - min_val) * (i / 4.0)
        ang_label = 180 - (i / 4.0 * 180)
        rad_label = np.deg2rad(ang_label)
        ax.text(np.cos(rad_label) * 1.05, np.sin(rad_label) * 1.05, f"{val:.0f}",
                ha='center', va='center', fontsize=10)

    
    if boost_threshold is not None and title == "MAP":
        if max_val - min_val > 0:
            boost_angle_ratio = (boost_threshold - min_val) / (max_val - min_val)
        else:
            boost_angle_ratio = 0

        boost_angle = 180 - (boost_angle_ratio * 180)
        boost_rad = np.deg2rad(boost_angle)
        x_start = np.cos(boost_rad) * 0.8
        y_start = np.sin(boost_rad) * 0.8
        x_end = np.cos(boost_rad) * 1.0
        y_end = np.sin(boost_rad) * 1.0
        ax.plot([x_start, x_end], [y_start, y_end], color='orange', linewidth=2, linestyle='--')
        ax.text(np.cos(boost_rad) * 0.7, np.sin(boost_rad) * 0.7 + 0.1, f"Boost ({int(boost_threshold)})",
                ha='center', va='center', fontsize=9, color='orange')
    ax.axis('off') 


def setup_live_plot(ax):
    ax.set_title("MAP Over Time", fontsize=16)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("MAP (kPa)")
    ax.set_ylim(0, ACCEL_MAP_BOOST_MAX + 20)
    ax.set_xlim(-10, 0) 
    ax.grid(True)
    
    ax.axhline(BOOST_THRESHOLD_KPA, color='orange', linestyle='--', label=f'Boost Threshold ({BOOST_THRESHOLD_KPA} kPa)')
    ax.legend(loc='upper left')


def update_graphs_for_save():
    
    draw_gauge(ax_rpm, current_sim_data["RPM"], 0, 8000, "RPM", "")
    
    draw_gauge(ax_map, current_sim_data["MAP"], 0, ACCEL_MAP_BOOST_MAX + 20, "MAP", "kPa", BOOST_THRESHOLD_KPA)

    
    ax_plot.clear()
    setup_live_plot(ax_plot)
    ax_plot.plot(time_history, map_history, color='blue', linewidth=2)

    
    fig.suptitle(f"Simulated ECU Data | TPS: {current_sim_data['TPS']}% | Boost Status: {current_sim_data['BoostStatus']}",
                 fontsize=20, y=0.98) 
    fig.tight_layout(rect=[0, 0, 1, 0.95]) 



def simulate_engine_data(duration_seconds=30):
    global simulation_running
    print(f"Simulation started for {duration_seconds} seconds.")
    print(f"Simulation running...")
    current_state = "idle"
    state_duration = 0 
    max_state_duration = 5 
    start_time = time.time()

    while time.time() - start_time < duration_seconds and not stop_event.is_set():
        rpm_val, map_val, tps_val = 0, 0, 0

        
        if current_state == "idle":
            rpm_val = random.randint(IDLE_RPM - 50, IDLE_RPM + 50)
            map_val = random.randint(IDLE_MAP - 5, IDLE_MAP + 5)
            tps_val = IDLE_TPS
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "accel_na"])
                state_duration = 0

        elif current_state == "cruise":
            rpm_val = random.randint(CRUISE_RPM_MIN, CRUISE_RPM_MAX)
            map_val = random.randint(CRUISE_MAP_MIN, CRUISE_MAP_MAX)
            tps_val = random.randint(CRUISE_TPS_MIN, CRUISE_TPS_MAX)
            if state_duration > random.randint(3, max_state_duration + 2):
                current_state = random.choice(["accel_boost", "accel_na", "decel", "idle"])
                state_duration = 0

        elif current_state == "accel_na":
            rpm_val = random.randint(ACCEL_RPM_MIN, ACCEL_RPM_MAX)
            map_val = random.randint(ACCEL_MAP_NA_MAX - 10, ACCEL_MAP_NA_MAX)
            tps_val = random.randint(ACCEL_TPS_MIN, ACCEL_TPS_MAX)
            if state_duration > random.randint(2, max_state_duration):
                current_state = random.choice(["cruise", "decel", "idle"])
                state_duration = 0

        elif current_state == "accel_boost":
            rpm_val = random.randint(ACCEL_RPM_MIN, ACCEL_RPM_MAX + 1000)
            map_val = random.randint(ACCEL_MAP_BOOST_MIN, ACCEL_MAP_BOOST_MAX)
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

       
        map_history.append(current_sim_data["MAP"])

        time.sleep(0.1) 
        state_duration += 0.1

    print("Simulation finished.")
    simulation_running = False


if __name__ == "__main__":
    
    draw_gauge(ax_rpm, 0, 0, 8000, "RPM", "")
    draw_gauge(ax_map, 0, 0, ACCEL_MAP_BOOST_MAX + 20, "MAP", "kPa", BOOST_THRESHOLD_KPA)
    setup_live_plot(ax_plot)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    
    simulation_thread = threading.Thread(target=simulate_engine_data, args=(20,), daemon=True) 
    simulation_thread.start()

    
    while simulation_running:
        update_graphs_for_save()
        fig.canvas.draw() 
        time.sleep(0.1) 

    
    update_graphs_for_save()
    fig.canvas.draw()

   
    output_filename = "engine_simulation.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"Simulation complete. Image saved to {output_filename}")

    
    plt.close(fig)
