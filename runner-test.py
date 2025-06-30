#!/usr/bin/env python3
"""
sysimpactcv/runner.py
Run:  python -m sysimpactcv.runner --scenario emergency_brake
"""
import argparse, os, traci
import pandas as pd
import shutil
import xml.etree.ElementTree as ET
from configparser import ConfigParser
from pathlib import Path
from .attacks  import emergency_brake, rear_end_collision, lane_closure, rsu_spoofing
from .sensors  import load_detectors
from .metrics  import poll_detectors, poll_brakes, poll_collisions

ATTACK_FN = {
    "emergency_brake": emergency_brake,
    "rear_end":        rear_end_collision,
    "lane_closure":    lane_closure,
    "scenario3": rsu_spoofing
}

def read_cfg(path: Path) -> ConfigParser:
    cp = ConfigParser()
    cp.optionxform = str
    cp.read(path)
    return cp

def absolute(path: str, base: Path) -> str:
    """Return absolute path if `path` is relative."""
    p = Path(path)
    return str(p if p.is_absolute() else base / p)

def build_cmd(cfg: ConfigParser, scen_dir: Path, seed: int):
    sim = cfg["simulation"]
    det = cfg["detectors"]

    sumo_bin = "sumo-gui" if sim.getboolean("GUI", False) else "sumo"
    sumocfg  = absolute(sim["sumoConfig_file"], scen_dir)

    add_files = ",".join(
        absolute(f.strip(), scen_dir)
        for f in [det["additional_xml"], det["xml_template"]]
    )

    return [
        sumo_bin, "-c", sumocfg,
        "--seed", str(seed),
        "--additional-files", add_files
    ]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", "-s", required=True,
                    help="Folder name inside ./scenarios/")
    args = ap.parse_args()

    scen_dir = Path(__file__).resolve().parent.parent / "scenarios" / args.scenario
    cfg_path = scen_dir / "scenario.ini"
    if not cfg_path.exists():
        raise FileNotFoundError(cfg_path)

    cfg      = read_cfg(cfg_path)
    attack   = cfg["attack"]["type"]
    attackfn = ATTACK_FN[attack]
    det_xml  = absolute(cfg["detectors"]["additional_xml"], scen_dir)
    lane_detectors  = load_detectors(det_xml)

    seeds = range(1, cfg["simulation"].getint("seed_count", 1)+1)
    for seed in seeds:

        # ─── prepare E1 detector file ─────────────────────────────
        original = scen_dir / "e1detectors.add.xml"
        modified = scen_dir / f"e1detectors_{scenario}_{seed}.add.xml"
        shutil.copy(original, modified)

        # update the <file> attributes inside it
        tree = ET.parse(modified)
        for e in tree.getroot().findall("e1Detector"):
            e.set("file", f"e1/e1detectors_{scenario}_{seed}.xml")
        tree.write(modified)
        
        cmd = build_cmd(cfg, scen_dir, seed)
        print("Launching:", " ".join(cmd))
        traci.start(cmd)
        state = {}
        det_rows, brake_rows, coll_rows = [], [], []
        detector_entry_log = {detector: set() for detector in lane_detectors}

        end_time = cfg["simulation"].getfloat("end_time", 9999)
        STEP_COUNTER = 0
        while traci.simulation.getTime() < end_time:
            traci.simulationStep()
            simtime      = traci.simulation.getTime()
            STEP_COUNTER += 1

            if attack == "emergency_brake":
                attackfn(state,
                         vehicle_id   = cfg["attack"]["vehicle_id"],
                         stop_position= cfg["attack"].getfloat("stop_position"))
                
            elif attack == "rear_end":
                attackfn(state,
                         aggressive_accel = cfg["attack"].getfloat("aggressive_accel"),
                         target_type      = cfg["attack"].get("target_type", fallback=None),
                         target_vehicles  = cfg["attack"].get("target_vehicles", fallback=None))
                
            elif attack == "lane_closure":
                attackfn(state,
                         target_type   = cfg["attack"]["target_type"])
                
            elif attack == "rsu_spoofing":
                sched_str = cfg["attack"]["vsl_schedule"]
                schedule  = []
                for piece in sched_str.split(","):
                    rng,speed = piece.split(":")
                    start,end = map(float, rng.split("-"))
                    schedule.append((start, end, float(speed)))
                lane_start = cfg["attack"].getfloat("lane_closure_start")
                zmin = cfg["attack"].getfloat("zone_min")
                zmax = cfg["attack"].getfloat("zone_max")
                attackfn(state,
                        vsl_sched=schedule,
                        lane_close_t=lane_start,
                        zone=(zmin, zmax))


        # ─── metrics every timestep ────────────────────────
            # poll lane-area detectors once a second (10 steps × 0.1 s)
            if STEP_COUNTER % 10 == 0:
                for det_id, cnt, dens in (
                    (d, *traci.lanearea.getLastStepVehicleNumber(d),)
                    for d in lane_detectors
                ):
                    dens_km = (cnt / 100.0) * 1000.0
                    det_rows.append((simtime, det_id, cnt, dens_km))

            brake_rows += poll_brakes(cfg["attack"].getfloat("ebrake_threshold", -4.5))
            coll_rows  += poll_collisions()


        traci.close()

        out_dir = scen_dir / "results" / f"seed_{seed}"
        out_dir.mkdir(parents=True, exist_ok=True)

        pd.DataFrame(det_rows,
                    columns=["time","det_id","veh_cnt","dens_veh_km"]
                ).to_csv(out_dir/"detectors.csv", index=False)

        pd.DataFrame(brake_rows,
                    columns=["time","veh_id","acc_mps2"]
                ).to_csv(out_dir/"ebrakes.csv", index=False)

        pd.DataFrame(coll_rows,
                    columns=["time","collider","victim","col_speed",
                            "vic_speed","lane","pos_m"]
                ).to_csv(out_dir/"collisions.csv", index=False)


        print("Seed", seed, "complete")

if __name__ == "__main__":
    main()
