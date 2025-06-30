# SysImpactCV

**SysImpactCV** is a Python framework for simulating and assessing the systemic impacts of cyber-threats on connected and autonomous vehicles (CAVs) using the [SUMO](https://www.eclipse.org/sumo/) traffic simulator and its TraCI API.  It provides:

- A common runner (`sysimpactcv/runner.py`) that launches baseline and attack scenarios  
- Modular attack scripts (`sysimpactcv/attacks.py`) for emergency braking, rear-end collision, lane-closure, RSU spoofing, etc.  
- Live polling of lane-area detectors, emergency-brake events, and collisions via TraCI, with results saved to CSV  
- A configurability layer via simple `scenario.ini` files and per-scenario folders  

---

## Quickstart

1. **Clone the repo**  
   ```bash
   git clone https://github.com/yourusername/SysImpactCV.git
   cd SysImpactCV
   ```

2. **Install Python dependencies**  
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    ```
3. **Install SUMO**  
    SysImpactCV uses the SUMO traffic simulator and its TraCI Python API.
    Download & install SUMO from eclipse.org/sumo or via your OS package manager.
    Make sure sumo, sumo-gui, and the TraCI Python module are on your PATH or in your PYTHONPATH.
    Example (Ubuntu):

    ```bash
    sudo apt install sumo sumo-tools sumo-doc
    export SUMO_HOME=/usr/share/sumo
    export PYTHONPATH=$SUMO_HOME/tools:$PYTHONPATH
    ```

4. **Prepare a scenario**  
    Each scenario lives in scenarios/<name>/, e.g.

    ```bash
    scenarios/
    └─ scenario3-rsu-spoofing/
        ├─ scenario.ini
        ├─ RSU.sumocfg
        ├─ RSU.net.xml
        ├─ RSU.rou.xml
        ├─ gui-settings.xml
        ├─ lanedetectors.add.xml
        └─ e1detectors.add.xml
    ```
    Edit scenario.ini to set:

    [simulation]: sumoConfig_file, seed_count, scenario_list (e.g. base,emergency_brake), end_time, etc.

    [detectors]: additional_xml, xml_template

    [attack]: type (must match one in sysimpactcv/attacks.py) and its parameters

5. **Run**  
    ```bash
    python -m sysimpactcv.runner --scenario cascading_braking  
    ```
    This will automatically run the base and attack modes for each random seed, collect detector and event metrics, and save:
    ```bash
    scenarios/scenario3-rsu-spoofing/
    ├─ data/data_base_1.csv
    ├─ data/data_emergency_brake_1.csv
    ├─ emergency/ebrake_base_1.csv
    ├─ emergency/ebrake_emergency_brake_1.csv
    └─ collision/coll_base_1.csv
    └─ collision/coll_emergency_brake_1.csv

## 📂 Repository Layout

    SysImpactCV/
    ├─ README.md
    ├─ requirements.txt
    ├─ scenarios/              # one folder per scenario
    │   └─ <scenario_name>/
    │       ├─ scenario.ini
    │       ├─ *.sumocfg, *.net.xml, *.rou.xml, *.add.xml
    │       ├─ data/           # CSVs for detector outputs
    │       ├─ emergency/      # CSVs for emergency brake events
    │       └─ collision/      # CSVs for rear-end collisions
    │       └─ e1/             # auto-generated per-seed detector outputs              
    └─ sysimpactcv/            # Python package
        ├─ __init__.py
        ├─ runner.py           # CLI entry point
        ├─ attacks.py          # attack/control functions
        ├─ sensors.py          # detector-ID loader
        └─ metrics.py          # TraCI polling utilities

## ⚙️ Configuration
Edit a scenario’s scenario.ini. Example:


    

    [simulation]
    sumoConfig_file     = RSU.sumocfg
    GUI                 = False
    step_length         = 0.1
    begin_time          = 0
    end_time            = 1200
    seed_count          = 2
    scenario_list       = base,attack

    [detectors]
    additional_xml      = lanedetectors.add.xml
    xml_template        = e1detectors.add.xml

    [attack]
    type                 = scenario3-rsu-spoofing
    vsl_schedule         = 600-2400:50,2400-4200:40,4200-7800:30
    lane_closure_start   = 6000
    zone_min             = 3000
    zone_max             = 4000
    ebrake_threshold     = -4.5

## 📖 Extending
- New attack: add a function in attacks.py, register it in ATTACK_FN in runner.py, and define its parameters under [attack].

- New metrics: add polling helpers in metrics.py and call them in the loop.

- New detectors: list them in your .add.xml and they’ll be discovered by sensors.py.