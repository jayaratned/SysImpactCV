import random
import traci

def emergency_brake(state, vehicle_id, stop_position):
    """
    Emergency brake attack:
      - Monitors the specified vehicle_id.
      - When that vehicle reaches stop_position (meters), force it to a full stop.
    State keys:
      - attack_success: bool (has the attack completed?)
    """
    # Initialize state
    state.setdefault('attack_success', False)

    # If already done, nothing more to do
    if state['attack_success']:
        return

    # If the vehicle is still in the simulation
    if vehicle_id in traci.vehicle.getIDList():
        pos = traci.vehicle.getLanePosition(vehicle_id)
        if pos >= stop_position:
            traci.vehicle.setSpeedMode(vehicle_id, 0)
            traci.vehicle.setLaneChangeMode(vehicle_id, 0)
            traci.vehicle.setSpeed(vehicle_id, 0.0)
            state['attack_success'] = True
            print(f"[attack] Emergency brake applied to {vehicle_id} at {pos:.2f} m")
    else:
        # Vehicle left the sim before braking
        state['attack_success'] = True

def rear_end_collision(
    state,
    *,
    aggressive_accel,
    target_vehicles=None,
    target_type=None,
    leader_gap=250
):
    """
    Rear end collision by aggresive acceleration:
      - For each vehicle in `target_vehicles` (or every vehicle whose type
        string contains `target_type`), disables safety checks and applies
        continuous aggressive acceleration until collision.
    
    Args:
      state (dict): shared dict across timesteps. Will get:
        state['attack_success'] = { vehicle_id: bool, ... }
      aggressive_accel (float): acceleration (m/s²) to apply.
      target_vehicles (str or list[str], optional): exact vehicle ID(s) to target.
      target_type (str, optional): substring to match in traci.vehicle.getTypeID().
      leader_gap (float): look‐ahead distance (m) for traci.vehicle.getLeader.

    Raises:
      ValueError: if neither target_vehicles nor target_type is provided.
    """
    # Validate inputs
    if target_vehicles is None and target_type is None:
        raise ValueError("Must provide either target_vehicles or target_type")

    # Initialize per-vehicle success map
    success_map = state.setdefault('attack_success', {})

    # Determine which vehicles to target
    if target_vehicles is not None:
        vehicles = ([target_vehicles] if isinstance(target_vehicles, str)
                    else list(target_vehicles))
    else:
        vehicles = [
            vid for vid in traci.vehicle.getIDList()
            if target_type in traci.vehicle.getTypeID(vid)
        ]

    # Apply attack for each vehicle
    for vid in vehicles:
        # Initialize flag
        success_map.setdefault(vid, False)
        if success_map[vid]:
            continue  # already done

        # If vehicle has left the sim, mark done
        if vid not in traci.vehicle.getIDList():
            success_map[vid] = True
            continue

        # Disable safety checks and apply acceleration
        traci.vehicle.setSpeedMode(vid, 0)
        traci.vehicle.setLaneChangeMode(vid, 0)
        traci.vehicle.setAcceleration(vid, aggressive_accel, 1)

        # Check for collisions this step
        for coll in traci.simulation.getCollisions():
            if coll.collider == vid:
                victim = coll.victim
                # Halt both vehicles
                for v in (vid, victim):
                    traci.vehicle.setAcceleration(v, 0, 0)
                    traci.vehicle.setSpeed(v, 0)
                    traci.vehicle.setSpeedMode(v, -1)
                    traci.vehicle.setLaneChangeMode(v, -1)
                success_map[vid] = True
                print(f"[attack] Collision: {vid} -> {victim}")
                break

def lane_closure(state,
                 lane_id='E0_0',
                 merge_to_lane=1,
                 detection_min=3000,
                 detection_max=3500):
    """
    Lane closure attack:
      - For every CAV in `lane_id` within the detection zone, force a lane change.
      - If past the zone, force a stop then merge.
    State isn’t used here but kept for API consistency.
    """
    for vid in traci.vehicle.getIDList():
        if 'CAV' in traci.vehicle.getTypeID(vid):
            cur_lane = traci.vehicle.getLaneID(vid)
            pos = traci.vehicle.getLanePosition(vid)

            if cur_lane == lane_id:
                if detection_min <= pos < detection_max:
                    traci.vehicle.changeLane(vid, merge_to_lane, duration=2)
                elif pos >= detection_max:
                    traci.vehicle.setSpeed(vid, 0.0)
                    traci.vehicle.changeLane(vid, merge_to_lane, duration=2)
            else:
                # Release to free speed once merged
                traci.vehicle.setSpeed(vid, -1)

def vsl_control(
    state,
    vsl_mph,
    zone_min,
    zone_max,
    default_speed_mps=55.56,
    max_deceleration_m_s2=-3.0
):
    """
    Variable Speed Limit (VSL) control for CAVs in a specific zone.

    Args:
      state (dict): shared dict across timesteps. Will get/keep:
        state['slowing_vehicles'] = { vehicle_id: target_speed, ... }
      vsl_mph (float): target speed limit in the zone (mph)
      zone_min (float): meter position where zone begins
      zone_max (float): meter position where zone ends
      default_speed_mps (float): speed to restore when outside zone (m/s)
      max_deceleration_m_s2 (float): used to compute slowdown duration

    Behavior:
      - For each CAV in the zone, if its current speed > target, issues
        `traci.vehicle.slowDown()` to ramp down over the computed time.
      - Once at or below the target, it sets the max speed there.
      - Vehicles leaving the zone get their max speed reset, and are
        removed from the slowing_vehicles tracker.
    """
    # prepare state tracker
    sl = state.setdefault('slowing_vehicles', {})

    # convert mph → m/s once
    target_speed = vsl_mph * 0.44704

    for vid in traci.vehicle.getIDList():
        # only affect CAV types
        if 'CAV' not in traci.vehicle.getTypeID(vid):
            continue

        pos = traci.vehicle.getLanePosition(vid)
        curr_speed = traci.vehicle.getSpeed(vid)

        # inside the VSL zone?
        if zone_min < pos < zone_max:
            if curr_speed > target_speed:
                # compute time to decelerate
                decel_time = (curr_speed - target_speed) / abs(max_deceleration_m_s2)
                traci.vehicle.slowDown(vid, target_speed, decel_time)
                sl[vid] = target_speed
            else:
                # already at or below target: enforce it
                traci.vehicle.setMaxSpeed(vid, target_speed)
        else:
            # outside zone: restore default
            traci.vehicle.setMaxSpeed(vid, default_speed_mps)
            # clean up tracker
            sl.pop(vid, None)

def set_target_speed(
    state,
    target_speed_mps: float,
    accel_rate_m_s2: float,
    *,
    target_vehicles=None,
    target_type=None,
):
    """
    Override vehicle speeds by commanding acceleration/deceleration toward a target.

    Args:
      state (dict): shared dict across timesteps (unused here, but for API consistency).
      target_speed_mps (float): desired speed in m/s.
      accel_rate_m_s2 (float): positive for acceleration, will be negated for deceleration.
      target_vehicles (str | list[str], optional): exact vehicle ID(s) to target.
      target_type (str, optional): substring to match in vehicle type IDs.

    Raises:
      ValueError: if neither target_vehicles nor target_type is provided.
    """
    if target_vehicles is None and target_type is None:
        raise ValueError("Must provide target_vehicles or target_type")

    # build list of IDs
    if target_vehicles is not None:
        vids = (
            [target_vehicles]
            if isinstance(target_vehicles, str)
            else list(target_vehicles)
        )
    else:
        vids = [
            vid for vid in traci.vehicle.getIDList()
            if target_type in traci.vehicle.getTypeID(vid)
        ]

    for vid in vids:
        if vid not in traci.vehicle.getIDList():
            continue

        # disable safety checks so our speed commands take effect immediately
        traci.vehicle.setSpeedMode(vid, 0)
        traci.vehicle.setLaneChangeMode(vid, 0)

        curr = traci.vehicle.getSpeed(vid)
        # decide accelerate vs decelerate
        if curr < target_speed_mps:
            # accelerate up to target
            traci.vehicle.setAcceleration(vid, accel_rate_m_s2, 1)
        elif curr > target_speed_mps:
            # decelerate down to target
            decel = -abs(accel_rate_m_s2)
            duration = (curr - target_speed_mps) / abs(decel) if decel != 0 else 0
            if duration > 0:
                traci.vehicle.slowDown(vid, target_speed_mps, duration)

        # enforce an upper bound so they don’t overshoot
        traci.vehicle.setMaxSpeed(vid, target_speed_mps)