#!/usr/bin/env python3
"""
sysimpactcv/runner.py
Launch both baseline and attack runs for each seed, collect metrics, and
dump CSVs with filenames data_<mode>_<seed>.csv, etc.
"""
import argparse, os, shutil
import xml.etree.ElementTree as ET
from configparser import ConfigParser
from pathlib import Path
import random

import traci
import pandas as pd

from .attacks  import emergency_brake, rear_end_collision, lane_closure, rsu_spoofing
from .sensors  import load_detectors
from .metrics  import poll_detectors, poll_brakes, poll_collisions

# Map your attack keys to functions
ATTACK_FN = {
    "emergency_brake": emergency_brake,
    "rear_end":        rear_end_collision,
    "lane_closure":    lane_closure,
    "scenario3":    rsu_spoofing,
    # add new attack: "my_new_attack": my_new_attack_fn
}

def read_cfg(path: Path) -> ConfigParser:
    cfg = ConfigParser()
    cfg.optionxform = str   # preserve case
    cfg.read(path)
    return cfg

def absolute(path: str, base: Path) -> str:
    """Resolve a relative path against base directory, or return as-is."""
    p = Path(path)
    return str(p if p.is_absolute() else base / p)

def build_cmd(sumocfg: str, seed: int, add_files: str, use_gui: bool) -> list:
    """Construct the TraCI command for SUMO."""
    bin_ = "sumo-gui" if use_gui else "sumo"
    return [
        bin_, "-c", sumocfg,
        "--seed", str(seed),
        "--additional-files", add_files
    ]

def main():
    # ─── parse CLI ────────────────────────────────────────────
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-s","--scenario", required=True,
        help="Name of the scenario folder under ./scenarios/"
    )
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    scen_dir  = repo_root / "scenarios" / args.scenario
    cfg_path  = scen_dir / "scenario.ini"
    if not cfg_path.exists():
        raise FileNotFoundError(f"Missing scenario INI at {cfg_path}")

    # ─── load config ─────────────────────────────────────────
    cfg     = read_cfg(cfg_path)
    sim     = cfg["simulation"]
    det_cfg = cfg["detectors"]
    atk_cfg = cfg["attack"]
    attack_type = atk_cfg["type"]
    attack_fn   = ATTACK_FN.get(attack_type)
    if attack_type not in ATTACK_FN:
        raise ValueError(f"Attack '{attack_type}' not implemented")

    # ─── determine run modes ──────────────────────────────────
    # default runs: baseline + whatever attack_type is
    modes = [
        m.strip() for m in
        sim.get("scenario_list", f"base,{attack_type}").split(",")
    ]

    # ─── load lane‐area detectors once ────────────────────────
    lane_xml  = absolute(det_cfg["additional_xml"], scen_dir)
    lane_ids  = load_detectors(lane_xml)

    # ─── seed loop ────────────────────────────────────────────
    seed_count = sim.getint("seed_count", 1)
    MAX_SEED   = 23423

    if seed_count > MAX_SEED:
        raise ValueError(f"seed_count ({seed_count}) > max available seeds ({MAX_SEED})")

    seeds = random.sample(range(1, MAX_SEED+1), seed_count)
    print("▶ Using random seeds:", seeds)

    for mode in modes:
        for seed in seeds:
            print(f"\n=== Mode '{mode}', Seed {seed} ===")

            # ─── ensure the output folder for E1 detectors exists ─────────────────
            e1_dir = scen_dir / "e1"
            e1_dir.mkdir(parents=True, exist_ok=True)

            # ─ prepare & patch E1 detector file ───────────────
            orig_e1 = scen_dir / "e1detectors.add.xml"
            mod_e1  = scen_dir / f"e1detectors_{mode}_{seed}.add.xml"
            shutil.copy(orig_e1, mod_e1)
            tree = ET.parse(mod_e1)
            for e in tree.getroot().findall("e1Detector"):
                # send output to e1/e1detectors_<mode>_<seed>.xml
                e.set("file", f"e1/e1detectors_{mode}_{seed}.xml")
            tree.write(mod_e1)

            # ─ build & launch SUMO ────────────────────────────
            sumocfg   = absolute(sim["sumoConfig_file"], scen_dir)
            add_files = f"{mod_e1},{lane_xml}"
            cmd       = build_cmd(sumocfg, seed, add_files,
                                  sim.getboolean("GUI", False))
            print("Launching:", " ".join(cmd))
            traci.start(cmd)

            # ─ init collectors ──────────────────────────────────
            state  = {}
            det_rows, brake_rows, coll_rows = [], [], []
            STEP_COUNTER = 0
            end_time     = sim.getfloat("end_time")

            # ─ simulation loop ──────────────────────────────────
            while traci.simulation.getTime() < end_time:
                traci.simulationStep()
                t = traci.simulation.getTime()
                STEP_COUNTER += 1

                # only inject when mode matches the attack
                if mode == attack_type:
                    if attack_type == "emergency_brake":
                        attack_fn(
                            state,
                            vehicle_id   = atk_cfg["vehicle_id"],
                            stop_position= atk_cfg.getfloat("stop_position")
                        )
                    elif attack_type == "rear_end":
                        attack_fn(
                            state,
                            aggressive_accel = atk_cfg.getfloat("aggressive_accel"),
                            target_type      = atk_cfg.get("target_type", fallback=None),
                            target_vehicles  = atk_cfg.get("target_vehicles", fallback=None)
                        )
                    elif attack_type == "lane_closure":
                        attack_fn(
                            state,
                            target_type = atk_cfg["target_type"]
                        )
                    elif attack_type == "rsu_spoofing":
                        sched_str = cfg["attack"]["vsl_schedule"]
                        schedule  = []
                        for piece in sched_str.split(","):
                            rng,speed = piece.split(":")
                            start,end = map(float, rng.split("-"))
                            schedule.append((start, end, float(speed)))
                        lane_start = cfg["attack"].getfloat("lane_closure_start")
                        zmin = cfg["attack"].getfloat("zone_min")
                        zmax = cfg["attack"].getfloat("zone_max")
                        attack_fn(state,
                                vsl_sched=schedule,
                                lane_close_t=lane_start,
                                zone=(zmin, zmax))
                    # … add new attack dispatch here …

                # poll dets once/sec (every 10 steps if step_length=0.1)
                if STEP_COUNTER % 10 == 0:
                    for det_id in lane_ids:
                        cnt = traci.lanearea.getLastStepVehicleNumber(det_id)
                        dens_km = (cnt/100.0)*1000.0
                        det_rows.append((t, det_id, cnt, dens_km, mode, seed))

                # -- emergency brakes: extend each row with mode & seed --
                for t, vid, acc in poll_brakes(cfg["attack"].getfloat("ebrake_threshold", -4.5)):
                    brake_rows.append((t, vid, acc, mode, seed))

                # -- collisions: extend each row with mode & seed --
                for t, col, vic, cs, vs, lane, pos in poll_collisions():
                    coll_rows.append((t, col, vic, cs, vs, lane, pos, mode, seed))



            traci.close()
            os.remove(mod_e1)

            # ─ write CSVs ───────────────────────────────────────
            data_dir      = scen_dir / "data"
            ebrake_dir    = scen_dir / "emergency"
            collision_dir = scen_dir / "collision"

            data_dir.mkdir(parents=True, exist_ok=True)
            ebrake_dir.mkdir(parents=True, exist_ok=True)
            collision_dir.mkdir(parents=True, exist_ok=True)

            pd.DataFrame(det_rows,
                        columns=["time","det_id","veh_cnt","dens_veh_km","mode","seed"]
                        ).to_csv(data_dir / f"data_{mode}_{seed}.csv", index=False)

            pd.DataFrame(brake_rows,
                        columns=["time","veh_id","acc_m_s2","mode","seed"]
                        ).to_csv(ebrake_dir / f"ebrake_{mode}_{seed}.csv", index=False)

            pd.DataFrame(coll_rows,
                        columns=["time","collider","victim",
                                "col_speed","vic_speed","lane","pos","mode","seed"]
                        ).to_csv(collision_dir / f"coll_{mode}_{seed}.csv", index=False)


            print(f"Completed mode='{mode}', seed={seed}")

if __name__ == "__main__":
    main()
