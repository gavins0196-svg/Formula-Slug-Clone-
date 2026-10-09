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

    if state.done:
        return new_state # do nothing if the simulation is done 
    
    # Forward Dynamics (Throttle, Drag, Brake)

    # PART 1: Throttle 
    if state.active_part == 1:
        mass = 300.0
        max_motor_torque = 180.0
        gear_ratio = 3.0
        wheel_radius = 0.216

        if state.time < 7.0:
            driver_input = state.time /7.0
        elif state.time <= 22.0:
            driver_input = 1.0
        else:
            driver_input = 0.0
            new_state.done = True

        command_torque = driver_input * max_motor_torque
        force_wheels = (command_torque * gear_ratio) / wheel_radius
        acceleration = force_wheels / mass

        new_state.velocity = state.velocity + acceleration * dt
        print(f"Part 1: time {state.time:.1f}s | Accel: {acceleration: .2f} m/s^2 | Vel: {new_state.velocity: .2f} m/s")

    # Part 2: Traction

    if state.active_part == 2:
        mass = 300.0
        forward_speed = 15.0
        cornering_stiffness = 36000.0
        
        if state.time < 3.0:
            steer_angle_deg = 5.0 * (state.time / 3.0)
        elif state.time <= 10.0:
            steer_angle_deg = 5.0
        else:
            steer_angle_deg = 0.0
            new_state.done = True
                
        steer_angle_rad = np.radians(steer_angle_deg)
        slip_angle = steer_angle_rad - (state.lateral_velocity / forward_speed)
        lateral_force = cornering_stiffness * slip_angle
        lateral_acceleration = lateral_force / mass
        
        new_state.lateral_velocity = state.lateral_velocity + lateral_acceleration * dt
        print(f"Part 2 | Time: {state.time:.1f}s | Slip: {np.degrees(slip_angle):.2f} deg | Lat Vel: {new_state.lateral_velocity:.2f} m/s")

    # Part 3: Drag
    if state.active_part == 3:
        mass = 300.0
        air_density = 1.2
        cross_area = 1.2
        drag_coeff = 0.7

        if state.time < 10.0:
            acceleration = 5.0
        else:
            acceleration = 0.0

        drag = 0.5 * cross_area * drag_coeff * air_density * (state.velocity**2)
        net_acceleration = acceleration - (drag/mass)
        new_state.velocity = state.velocity + net_acceleration *dt

        if state.time >= 10.0 and new_state.velocity <= 0.1:
            new_state.done = True 
        print(f"Time: {state.time:.1f}s | Drag: {drag:.1f} N | Vel: {new_state.velocity:.2f}")

    # Part 4: Motor Model and Back-EMF

    if state.active_part == 4:
        mass = 300.0
        max_propulsion = 2000.0
        vmax = 27.0

        if state.time < 3.0:
            throttle_input = state.time /3.0
        elif state.time <= 3.0:
            throttle_input = 1.0
            new_state.done = True
    
        propulsion_force = max_propulsion * throttle_input * (1.0 - (state.velocity / vmax))
        acceleration = propulsion_force / mass
        new_state.velocity = state.velocity + acceleration * dt

        print(f"Part 4 | Time: {state.time: .1f}s | Force: {propulsion_force} N | Vel: {new_state.velocity:.2f} m/s")

    # Part 5: Braking
    if state.active_part == 5:
        mass = 300.0
        max_braking = 1850.0

        # skip the acceleration phase
        if state.time == 0.0:
            state.velocity= 25.0 
            new_state.velocity =25.0

        if state.time < 2.0:
            brake_input =0.0
        else:
            brake_input = 1.0

        if state.time <2.0:
            breake_input= 0.0
        else:
            brake_input = 1.0
        
        brake_force = brake_input * max_braking
        acceleration = -(brake_force / mass)
        new_state.velocity = state.velocity + acceleration * dt

        if new_state.velocity <= 0.0:
            new_state.done = True

        print(f"Part 4 | Time: {state.time: .1f}s | Brake Force: {brake_force} N | Vel: {new_state.velocity:.2f} m/s")

    # Part 6: Unsimplify (Drag + Rolling Resistance )
    if state.active_part == 6:
        mass = 300.0 
        air_density = 1.2
        cross_area = 1.2
        drag_coeff = 0.7

        # new constants for rolling resistance 
        rolling_res_coeff = 0.015
        gravity = 9.81

        # Timeline: 10 seconds of gas, time coasting
        if state.time < 10.0:
            acceleration = 5.0
        else:
            acceleration = 0.0 
            
        # Calculation for Aerodynamic Drag 
        drag = 0.5 * cross_area * drag_coeff * air_density * (state.velocity**2)

        # Calculation for Rolling Resistance 
        if state.velocity > 0.0:
            rolling_resistance = rolling_res_coeff * mass * gravity 
        else:
            rolling_resistance = 0.0
            
        # Combine and Drag to find net acceleration + Updates Speedometer 
        total_resistive_force = drag + rolling_resistance 
        net_acceleration = acceleration - (total_resistive_force/mass)
        new_state.velocity = state.velocity + net_acceleration * dt 
                
        # Stop engine when the car comes to a halt 
        if state.time >= 10.0 and new_state.velocity <= 0.1:
            new_state.velocity = 0.0
            new_state.done = True
            
        print(f"Part 6 | Time: {state.time: .1f}s | Res Force: {total_resistive_force:.1f} | Vel: {new_state.velocity:.2f} m/s")


    return new_state 

# Animation Setup
current_state = State(active_part=6)

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