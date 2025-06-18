# metrics.py

import traci

def poll_detectors(det_ids):
    """
    Polls lane‐area detectors for vehicle counts and densities.

    Args:
      det_ids (list[str]): IDs of laneAreaDetector objects.

    Returns:
      List[tuple]: Each tuple is
        (time_s: float,
         detector_id: str,
         vehicle_count: int,
         density_veh_per_km: float)
    """
    rows = []
    t = traci.simulation.getTime()
    for det in det_ids:
        count = traci.lanearea.getLastStepVehicleNumber(det)
        # density = vehicles per meter * 1000 → veh/km
        density = (count / 100.0) * 1000
        rows.append((t, det, count, density))
    return rows


def poll_brakes(threshold):
    """
    Detects emergency braking events by acceleration threshold.

    Args:
      threshold (float): acceleration (m/s²) below which we consider an
                         emergency brake.

    Returns:
      List[tuple]: Each tuple is
        (time_s: float,
         vehicle_id: str,
         acceleration_m_s2: float)
    """
    rows = []
    t = traci.simulation.getTime()
    for vid in traci.vehicle.getIDList():
        acc = traci.vehicle.getAcceleration(vid)
        if acc < threshold:
            rows.append((t, vid, acc))
    return rows


def poll_collisions():
    """
    Retrieves all collisions that occurred this timestep.

    Returns:
      List[tuple]: Each tuple is
        (time_s: float,
         collider_id: str,
         victim_id: str,
         collider_speed_m_s: float,
         victim_speed_m_s: float,
         lane_id: str,
         position_m: float)
    """
    rows = []
    t = traci.simulation.getTime()
    for coll in traci.simulation.getCollisions():
        rows.append((
            t,
            coll.collider,
            coll.victim,
            coll.colliderSpeed,
            coll.victimSpeed,
            coll.lane,
            coll.pos
        ))
    return rows
