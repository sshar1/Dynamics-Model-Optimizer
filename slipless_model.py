import math

def dynamics(state: list[float], action: list[float], next_state: list[float], timestep: float, constants: list[float]):
    """
    Calculates the next state of a vehicle using a slipless kinematic model.

    Args:
        state: Current state [x, y, yaw, speed].
        action: Action to apply [swangle, torque].
        next_state: Output array to store the next state.
        timestep: Time delta in seconds.
        constants: A list of model parameters in the following order:
            [0] understeer_slope
            [1] cg_to_front
            [2] cg_to_rear
            [3] gear_ratio
            [4] saturating_motor_torque
            [5] torque_mode (0:AWD, 1:FWD, 2:RWD)
            [6] whl_radius
            [7] car_mass
            [8] rolling_drag
    """
    x, y, yaw, speed = state[0], state[1], state[2], state[3]
    swangle, torque = -action[0], action[1] * gear_ratio

    understeer_slope, cg_to_front, cg_to_rear, gear_ratio, saturating_motor_torque, torque_mode, whl_radius, car_mass, rolling_drag = constants

    saturating_tire_torque = saturating_motor_torque * 0.5 * gear_ratio

    slip_angle_expr = math.atan((cg_to_rear / (cg_to_front + cg_to_rear)) * math.tan(swangle / (1 + understeer_slope * speed))) if (1 + understeer_slope * speed) != 0 and math.cos(swangle / (1 + understeer_slope * speed)) != 0 else 0.0
    next_speed_expr = 0.0
    if torque_mode == 0: # AWD Mode
        next_speed_expr = max(0.0, speed + (((max(-saturating_tire_torque, min(torque * 0.5, saturating_tire_torque)) * math.cos(swangle - slip_angle_expr)) + (max(-saturating_tire_torque, min(torque * 0.5, saturating_tire_torque)) * math.cos(slip_angle_expr))) / (whl_radius * car_mass) - (rolling_drag / car_mass)) * timestep)
    elif torque_mode == 1: # FWD Mode
        next_speed_expr = max(0.0, speed + (((max(-saturating_tire_torque, min(torque, saturating_tire_torque)) * math.cos(swangle - slip_angle_expr))) / (whl_radius * car_mass) - (rolling_drag / car_mass)) * timestep)
    elif torque_mode == 2: # RWD Mode
        next_speed_expr = max(0.0, speed + (((max(-saturating_tire_torque, min(torque, saturating_tire_torque)) * math.cos(slip_angle_expr))) / (whl_radius * car_mass) - (rolling_drag / car_mass)) * timestep)
    
    dist_avg_speed_expr = (2.0 / 3.0 * (next_speed_expr**3 - speed**3) / (next_speed_expr**2 - speed**2)) if next_speed_expr != speed else speed

    kinematic_swangle_for_dist_avg_speed_expr = swangle / (1 + understeer_slope * dist_avg_speed_expr)

    angular_speed_expr = (math.copysign(dist_avg_speed_expr / math.sqrt(cg_to_rear**2 + ((cg_to_front + cg_to_rear) / math.tan(kinematic_swangle_for_dist_avg_speed_expr))**2), kinematic_swangle_for_dist_avg_speed_expr)) if kinematic_swangle_for_dist_avg_speed_expr != 0 else 0
    
    # --- Final State Calculations ---
    next_state[0] = (x + cg_to_rear * (math.cos(yaw + angular_speed_expr * timestep) - math.cos(yaw)) + ((cg_to_front + cg_to_rear) / math.tan(kinematic_swangle_for_dist_avg_speed_expr)) * (math.sin(yaw + angular_speed_expr * timestep) - math.sin(yaw))) if angular_speed_expr != 0 else (x + dist_avg_speed_expr * math.cos(yaw) * timestep)
    next_state[1] = (y + cg_to_rear * (math.sin(yaw + angular_speed_expr * timestep) - math.sin(yaw)) + ((cg_to_front + cg_to_rear) / math.tan(kinematic_swangle_for_dist_avg_speed_expr)) * (-math.cos(yaw + angular_speed_expr * timestep) + math.cos(yaw))) if angular_speed_expr != 0 else (y + dist_avg_speed_expr * math.sin(yaw) * timestep)
    next_state[2] = yaw + angular_speed_expr * timestep
    next_state[3] = next_speed_expr