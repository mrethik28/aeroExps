import numpy as np

class Aircraft:

    def __init__(self, mass:float, wing_area:float, Cl:float, Cd:float):
        self.g = 9.81
        self.rho = 1.225

        self.mass = mass
        self.wing_area = wing_area
        self.Cl = Cl
        self.Cd = Cd

    def get_state_derivative(self, t: float, state: np.ndarray) -> list:
        x, y, vx, vy = state
        
        velocity_vector = np.array([vx, vy])
        speed = np.linalg.norm(velocity_vector)

        if speed == 0:
            ax = 0
            ay = -self.g
            return [vx, vy, ax, ay]

        # --- Calculate Aerodynamic Forces ---
        
        # Lift and Drag magnitudes
        lift_mag = 0.5 * self.rho * speed**2 * self.wing_area * self.Cl
        drag_mag = 0.5 * self.rho * speed**2 * self.wing_area * self.Cd

        # --- Calculate Force Vectors using NumPy ---
        # The direction of the velocity vector (as a unit vector)
        velocity_direction = velocity_vector / speed
        
        # Drag force vector (opposite to velocity)
        drag_force = -drag_mag * velocity_direction
        
        # Lift force vector (perpendicular to velocity)
        # For a 2D vector [vx, vy], a perpendicular vector is [-vy, vx]
        lift_direction = np.array([-vy, vx]) / speed
        lift_force = lift_mag * lift_direction
        
        # --- Gravitational Force Vector ---
        gravity_force = np.array([0, -self.mass * self.g])
        
        # --- Net Force and Acceleration ---
        net_force = drag_force + lift_force + gravity_force
        acceleration = net_force / self.mass
        ax, ay = acceleration
        
        # Return the derivative of the state vector: [dx/dt, dy/dt, dvx/dt, dvy/dt]
        return [vx, vy, ax, ay]