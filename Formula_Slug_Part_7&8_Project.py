

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np

@dataclass
class State:
    time: float = 0.0
    velocity: float = 0.0 # forward velocity 
    lateral_velocity: float = 0.0 # lateral velocity 
    active_part: int = 1
    done: bool = False

def step (state:State) -> State:
    dt = 0.1
    # Start by copying over the current state
    new_state = State(
        time=state.time + dt,
        velocity=state.velocity,
        lateral_velocity=state.lateral_velocity,
        active_part=state.active_part,
        done=state.done
    )

    # Part 7: Longitudinal Control (Brake + Throttle)
    if state.active_part == 7:
        mass = 300.0
        max_motor_torque = 180.0
        gear_ratio = 3.0
        wheel_radius = 0.216
        max_braking = 1850.0 #N

        # Test Timeline: 3s throttle. 2s coast, hard brake
        if state.time < 3.0:
            driver_input = state.time / 3.0
        elif state.time < 5.0:
            driver_input = 0.0 
        elif state.time < 8.0:
            driver_input = -1.0
        else:
            driver_input = 0.0
            new_state.done = True 

        if driver_input > 0.0:
            command_torque = driver_input * max_motor_torque
            longitudinal_force = (command_torque * gear_ratio) / wheel_radius
        elif driver_input < 0.0:
            longitudinal_force = driver_input * max_braking 
        else:
            longitudinal_force = 0.0

        acceleration = longitudinal_force / mass
        new_state.velocity = state.velocity + acceleration * dt

        # Make sure that braking doesn't put the car in reveerse
        if new_state.velocity <= 0.0 and driver_input < 0.0:
            new_state.velocity = 0.0
            new_state.done = True 

        print(f"Time: {state.time:.2f}s | Input: {driver_input:.2f}s | Vel: {new_state.velocity:.2f} m/s")

    # Part 8: Realistic Traciton
    if state.active_part == 8:
        mass = 300.0
        forward_speed = 15.0
        cornering_stiffness = 36000.0

        gravity = 9.81
        friction_coeff = 0.85 # frictions for tire on asphalt

        # Timeline
        if state.time < 5.0:
            steer_angle = 15.0 * (state.time / 5.0)
        else:
            steer_angle = 15.0

        if state.time >= 10.0:
            new_state.done = True

        steer_angle_rad = np.radians(steer_angle)

        # Ideal Lateral Force Calculation
        slip_angle = steer_angle_rad - (state.lateral_velocity / forward_speed)
        ideal_lateral_force = cornering_stiffness * slip_angle

        normal_force = mass * gravity
        max_grip = friction_coeff * normal_force

        # Limit the force
        if ideal_lateral_force > max_grip:
            actual_lateral_force = max_grip 
            is_sliding = True 
        elif ideal_lateral_force < -max_grip:
            actual_lateral_force = -max_grip
        else:
            actual_lateral_force = ideal_lateral_force
            is_sliding = False

        lateral_acceleration = actual_lateral_force / mass
        new_state.lateral_velocity = state.lateral_velocity + lateral_acceleration * dt
        new_state.velocity = forward_speed 

        slide_status = "SLIDING!" if is_sliding else "Grip OK"
        print(f"Time: {state.time:.1f}s | Steer: {steer_angle:.2f} deg | Force: {actual_lateral_force:.0f} N")

    return new_state 

# Animation Setup
current_state = State(active_part=3)

def animate (i):
    global current_state
    current_state = step(current_state)
 
fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)
ax.grid()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
# these lines are so the animation doesnt zoom in or out
plt.pause(3)
ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)
plt.show()