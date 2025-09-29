import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from flight_engine.flight_model import Aircraft

PRESETS = {
    "Paper Airplane": {
        "mass": 0.005, "wing_area": 0.017, "Cl": 0.6, "Cd": 0.1,
        "velocity": 7.0, "angle": 10.0, "altitude": 2.0
    },
    "Frisbee": {
        "mass": 0.175, "wing_area": 0.057, "Cl": 0.15, "Cd": 0.08,
        "velocity": 14.0, "angle": 5.0, "altitude": 1.5
    },
    "Heavy Glider": {
        "mass": 5.0, "wing_area": 0.5, "Cl": 0.8, "Cd": 0.05,
        "velocity": 25.0, "angle": 2.0, "altitude": 50.0
    },
    "Projectile (Cannonball)": {
        "mass": 2.0, "wing_area": 0.01, "Cl": 0.0, "Cd": 0.47,
        "velocity": 30.0, "angle": 45.0, "altitude": 0.0
    }
}

def run_simulation(aircraft_params, initial_conditions):
    aircraft = Aircraft(
        mass=aircraft_params['mass'],
        wing_area=aircraft_params['wing_area'],
        Cl=aircraft_params['Cl'],
        Cd=aircraft_params['Cd']
    )
    
    velocity = initial_conditions['velocity']
    angle = initial_conditions['angle']
    altitude = initial_conditions['altitude']
    vx0 = velocity * np.cos(np.radians(angle))
    vy0 = velocity * np.sin(np.radians(angle))
    initial_state = [0.0, altitude, vx0, vy0]

    t_span = [0, 60]  # simulate for 60 seconds
    t_eval = np.linspace(t_span[0], t_span[1], 1500)    

    # Define an event to stop the simulation when altitude (state[1]) is zero
    def ground_hit_event(t, state): return state[1]
    ground_hit_event.terminal = True  # Stop the integration when this event occurs
    ground_hit_event.direction = -1   # Trigger when crossing from positive to negative

    # Run the ODE solver
    solution = solve_ivp(
        fun=aircraft.get_state_derivative,
        t_span=t_span,
        y0=initial_state,
        t_eval=t_eval,
        events=ground_hit_event,
        dense_output=True
    )
    return solution

def plot_trajectory(solution):
    """
    Plots the flight path from the simulation solution and calculates key metrics.
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    
    x_coords = solution.y[0]
    y_coords = solution.y[1]

    # Handle cases where the simulation ends immediately
    if len(x_coords) == 0:
        return fig, {"range": 0, "max_altitude": 0, "flight_time": 0}

    # Plot the main trajectory
    ax.plot(x_coords, y_coords, label='Trajectory', color='dodgerblue', linewidth=2)
    ax.set_xlabel("Range (m)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Aircraft Flight Trajectory")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # Find and mark the peak altitude (apogee)
    peak_altitude_index = np.argmax(y_coords)
    peak_x = x_coords[peak_altitude_index]
    peak_y = y_coords[peak_altitude_index]
    ax.plot(peak_x, peak_y, 'ro', label=f'Peak Altitude: {peak_y:.1f} m')
    ax.axhline(y=peak_y, color='r', linestyle='--', alpha=0.5)
    
    ax.legend()
    ax.set_ylim(bottom=0)  # Ensure the y-axis starts at the ground
    ax.set_xlim(left=0)    # Ensure the x-axis starts at the launch point

    metrics = {
        "range": x_coords[-1],
        "max_altitude": peak_y,
        "flight_time": solution.t[-1]
    }
    
    return fig, metrics

# --- 3. STREAMLIT APP LAYOUT ---

st.set_page_config(page_title="Flight Simulator", layout="wide")

st.title("✈️ Interactive 2D Flight Dynamics Simulator")
st.write(
    "Select a preset or adjust the sliders in the sidebar to control the object's "
    "properties and launch conditions. The plot will update in real-time."
)

# --- Sidebar Setup ---
st.sidebar.header("Simulation Parameters")

# --- Preset Loading Logic using Session State ---
if 'preset' not in st.session_state:
    st.session_state.preset = "Paper Airplane"

def update_params_from_preset():
    """Callback function to update sliders based on the selected preset."""
    preset_values = PRESETS[st.session_state.preset]
    for key, value in preset_values.items():
        st.session_state[key] = value

# The selectbox for presets, which calls the update function on change
st.sidebar.selectbox(
    "Load a Preset",
    options=list(PRESETS.keys()),
    key='preset',
    on_change=update_params_from_preset
)
st.sidebar.markdown("---")

# Initialize state from the default preset on the very first run
if 'mass' not in st.session_state:
    update_params_from_preset()

# --- Sliders and Inputs ---
st.sidebar.subheader("Aircraft Properties")
# Each slider is linked to a key in st.session_state
mass = st.sidebar.slider("Mass (kg)", 0.001, 10.0, key='mass', format="%.3f")
wing_area = st.sidebar.slider("Wing Area (m²)", 0.001, 1.0, key='wing_area', format="%.3f")
cl = st.sidebar.slider("Lift Coefficient (CL)", 0.0, 2.0, key='Cl', step=0.01)
cd = st.sidebar.slider("Drag Coefficient (CD)", 0.0, 2.0, key='Cd', step=0.01)

st.sidebar.subheader("Initial Conditions")
velocity = st.sidebar.slider("Initial Velocity (m/s)", 1.0, 50.0, key='velocity', step=0.5)
angle = st.sidebar.slider("Launch Angle (degrees)", 0.0, 90.0, key='angle', step=1.0)
altitude = st.sidebar.number_input("Initial Altitude (m)", min_value=0.0, key='altitude', step=0.5)

# --- Main Panel for Outputs ---

# Pack parameters from session state into dictionaries for the simulation function
aircraft_params = {
    'mass': st.session_state.mass,
    'wing_area': st.session_state.wing_area,
    'Cl': st.session_state.Cl,
    'Cd': st.session_state.Cd
}
initial_conditions = {
    'velocity': st.session_state.velocity,
    'angle': st.session_state.angle,
    'altitude': st.session_state.altitude
}

# Run simulation and generate plot based on current slider values
solution = run_simulation(aircraft_params, initial_conditions)
print(solution)
fig, metrics = plot_trajectory(solution)

# Display the plot
st.pyplot(fig)

# Display key performance metrics below the plot
st.subheader("Flight Performance Metrics")
col1, col2, col3 = st.columns(3)
col1.metric("Horizontal Range", f"{metrics['range']:.1f} m")
col2.metric("Maximum Altitude", f"{metrics['max_altitude']:.1f} m")
col3.metric("Total Flight Time", f"{metrics['flight_time']:.2f} s")
