#!/usr/bin/env python3
import argparse
import traci
from configparser import ConfigParser

from sensors import load_detectors
from attacks import emergency_brake, aggressive_collision, lane_closure
from metrics import poll_detectors, poll_brakes, poll_collisions
import pandas as pd

# Map the scenario name from your INI to the function to call
SCENARIO_FUNCS = {
    "emergency_brake": emergency_brake,
    "collision":      aggressive_collision,
    "lane_closure":   lane_closure,
}

def load_ini(path):
    cfg = ConfigParser()
    cfg.optionxform = str    # preserve case of keys
    cfg.read(path)
    return cfg

def build_sumo_command(cfg, seed):
    sim = cfg["simulation"]
    det = cfg["detectors"]
    coll = cfg["collision parameters"]

    sumo_bin = "sumo-gui" if sim.getboolean("GUI", fallback=False) else "sumo"
    cmd = [
        sumo_bin,
        "-c", sim["sumoConfig_file"],
        "--seed", str(seed),
        "--step-length", sim["step_length"],
        "--begin", sim["begin_time"],
        "--end", sim["end_time"],
        "--additional-files", f"{det['additional_xml']},{det['xml_template']}"
    ]

    # collision overrides
    if coll.get("collision_action"):
        cmd += ["--collision.action", coll["collision_action"]]
    if coll.get("collision_stop_time"):
        cmd += ["--collision.stoptime", coll["collision_stop_time"]]
    if coll.get("time_to_teleport"):
        cmd += ["--time-to-teleport", coll["time_to_teleport"]]

    return cmd

def main():
    parser = argparse.ArgumentParser(
        description="Run a single SysImpactCV scenario via an INI"
    )
    parser.add_argument(
        "-c", "--config", required=True,
        help="Path to your scenario INI file"
    )
    args = parser.parse_args()

    cfg = load_ini(args.config)
    sim = cfg["simulation"]
    scenario = cfg["attack scenario"]["scenario"]

    if scenario not in SCENARIO_FUNCS:
        raise ValueError(f"Unknown scenario '{scenario}'. Valid options: {list(SCENARIO_FUNCS)}")

    # Load your detectors once
    detector_ids = load_detectors(cfg["detectors"]["additional_xml"])

    # Run one simulation per seed
    seed_count = sim.getint("seed_count")
    for seed in range(1, seed_count + 1):
        print(f"\n=== Running '{scenario}' with seed={seed} ===")
        state = {}
        cmd = build_sumo_command(cfg, seed)
        print("Launching SUMO:", " ".join(cmd))
        traci.start(cmd)

        end_time = sim.getfloat("end_time")
        attack_fn = SCENARIO_FUNCS[scenario]

        while traci.simulation.getTime() < end_time:
            traci.simulationStep()
            
            # Poll detectors
            detector_rows   = poll_detectors(detector_ids)
            ebrake_rows     = poll_brakes(cfg.getfloat("attack","ebrake_threshold"))
            collision_rows  = poll_collisions()
            
            # --- call your attack/control every step ---
            if scenario == "emergency_brake":
                # INI must define 'vehicle_id' and 'stop_position' under [attack scenario]
                vid  = cfg["attack scenario"]["vehicle_id"]
                pos = cfg["attack scenario"].getfloat("stop_position")
                attack_fn(state, vehicle_id=vid, stop_position=pos)

            elif scenario == "collision":
                # INI should define 'aggressive_accel' and either 'target_vehicles' or 'target_type'
                accel = cfg["collision parameters"].getfloat("aggressive_accel")
                # choose one of these in your INI:
                if cfg["attack scenario"].get("target_vehicles"):
                    attack_fn(
                        state,
                        aggressive_accel=accel,
                        target_vehicles=cfg["attack scenario"]["target_vehicles"]
                    )
                else:
                    attack_fn(
                        state,
                        aggressive_accel=accel,
                        target_type=cfg["attack scenario"]["target_type"]
                    )

            elif scenario == "lane_closure":
                # e.g. INI might define 'target_type' = "CAV"
                attack_fn(
                    state,
                    target_type=cfg["attack scenario"]["target_type"]
                )

            # … you can also poll metrics here …

        traci.close()
        print(f"Completed seed={seed}\n")

        df_det   = pd.DataFrame(detector_rows, columns=["Time","Detector","Count","Density"])
        df_brake = pd.DataFrame(ebrake_rows,   columns=["Time","Vehicle","Acceleration"])
        df_coll  = pd.DataFrame(collision_rows,columns=["Time","Collider","Victim","ColliderSpeed","VictimSpeed","Lane","Pos"])
        df_det.to_csv(...); df_brake.to_csv(...); df_coll.to_csv(...)

if __name__ == "__main__":
    main()
