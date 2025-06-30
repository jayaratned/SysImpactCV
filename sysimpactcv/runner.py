#!/usr/bin/env python3
"""
sysimpactcv/runner.py
Run:  python -m sysimpactcv.runner --scenario emergency_brake
"""
import argparse, os, traci
from configparser import ConfigParser
from pathlib import Path
from attacks  import emergency_brake, aggressive_collision, lane_closure, vsl_laneclosure

ATTACK_FN = {
    "emergency_brake": emergency_brake,
    "rear_end":        aggressive_collision,
    "lane_closure":    lane_closure,
    "vsl_laneclosure": vsl_laneclosure,
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

    seeds = range(1, cfg["simulation"].getint("seed_count", 1)+1)
    for seed in seeds:
        cmd = build_cmd(cfg, scen_dir, seed)
        print("Launching:", " ".join(cmd))
        traci.start(cmd)
        state = {}

        end_time = cfg["simulation"].getfloat("end_time", 9999)
        while traci.simulation.getTime() < end_time:
            traci.simulationStep()

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
                
            elif attack == "vsl_laneclosure":
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


        traci.close()
        print("Seed", seed, "complete")

if __name__ == "__main__":
    main()
