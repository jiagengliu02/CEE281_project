# CEE281 Traffic Simulation Project

Two traffic environments in one project: a **freeway with an entry ramp** and a **signalized intersection**. The root contains one command-line entry point and one visual launch page. Source code, configurations, and generated results are clearly separated.

Python 3.10+; standard library only. No package installation or server is needed for the saved visualizations.

## Start here

```bash
cd /Users/liujiageng/Desktop/CEE281/project
open view.html
```

The launch page opens either saved viewer:

| Environment | Viewer | What it shows |
| --- | --- | --- |
| Freeway + ramp | `results/freeway/viewer.html` | Vehicle motion, ramp entry, work-zone merges, trajectories, queues, and detector measurements |
| Signalized intersection | `results/intersection/viewer.html` | Fixed-time versus responsive signals, phase timing, queue estimates, faults, and delay |

Press Play, drag the time slider, or change the scenario. Viewer controls replay saved results; they do not rerun the simulation.

## Run and verify

All commands use the same root entry point:

```bash
python3 run.py freeway               # Generate the three freeway cases
python3 run.py intersection          # Generate the intersection experiments
python3 run.py check freeway         # Verify the freeway implementation
python3 run.py check intersection    # Verify the signal implementation
python3 run.py check all             # Verify both environments
```

After running, refresh or reopen `view.html`. A run replaces matching files in its output directory. Checks create their output directory if needed and save `verification.json` for that environment. Run checks without Python optimization (`-O`) because physical audits use assertions.

For a short intersection experiment or a higher-demand freeway case:

```bash
python3 run.py intersection --scenario directional_shift --seed 11 --output results/intersection_demo
python3 run.py freeway --config freeway/configs/heavy_demand.json --output results/freeway_heavy
```

Custom outputs have their own `viewer.html`; `view.html` links to the default output directories. Use `python3 run.py --help` or `python3 run.py intersection --help` for options. Defaults are resolved from the project location, so an absolute path to `run.py` also works from another working directory. User-supplied relative `--config` and `--output` paths are relative to the calling directory.

## File structure

```text
project/
├── README.md                  # Overview and commands
├── run.py                     # The only command-line entry point
├── view.html                  # Open either saved visualization
├── .gitignore                 # Exclude generated results and caches from Git
│
├── freeway/
│   ├── README.md              # Freeway model, assumptions, metrics, and sources
│   ├── __init__.py            # Python package marker
│   ├── behavior.py            # IDM car following, lane changes, merge courtesy
│   ├── model.py               # Simulation loop: arrivals, ramp entry, detectors, metrics
│   ├── run.py                 # Experiments and result export
│   ├── verify.py              # Independent physical checks
│   ├── viewer_template.html   # Freeway viewer source
│   └── configs/
│       ├── default.json
│       └── heavy_demand.json
│
├── intersection/
│   ├── README.md              # Signal model, interfaces, experiments, and sources
│   ├── __init__.py            # Python package marker
│   ├── model.py               # Arrivals, road/external queues, and stop-line service
│   ├── detector.py            # Interval counts and injected detector faults
│   ├── observation.py         # Queue estimates and forecasts from detector reports
│   ├── control.py             # Phase state machine and signal policies
│   ├── run.py                 # Paired experiments and result export
│   ├── verify.py              # Signal, accounting, and causality checks
│   ├── viewer_template.html   # Intersection viewer source
│   └── configs/
│       └── default.json
│
└── results/                   # Generated artifacts; not source code
    ├── freeway/
    │   ├── open/
    │   ├── closure/
    │   ├── fixed/
    │   ├── data.json
    │   ├── verification.json
    │   └── viewer.html
    └── intersection/
        ├── balanced/
        ├── directional_shift/
        ├── detector_fault/
        ├── heavy/
        ├── config.json
        ├── summary.csv
        ├── experiment_report.md
        ├── verification.json
        ├── data.json
        └── viewer.html
```

Edit configurations to change an experiment, `model.py` to change traffic behavior, and `viewer_template.html` to change the visualization. Generated HTML files are overwritten on rerun; do not edit them as source. Do not run package files directly; use the root `run.py` commands so relative imports and paths resolve consistently.

## How the two environments relate

**Freeway:** the shared project foundation. It models individual vehicles with IDM, an entry ramp, and a work-zone closure. Compare `closure` and `fixed` to isolate ramp metering; `open` is the no-work-zone reference. Fixed ramp release is not yet an adaptive TMC.

**Intersection:** the selected final signal-control direction. It uses four finite-storage point queues and explicit green/yellow/all-red phases. It compares fixed-time, queue-responsive, and zero-threshold ablation controllers. Blocks in its viewer are queue-count illustrations, not spatial vehicle trajectories.

The environments share a launcher, units, observation conventions, logging conventions, and output organization. Their traffic models remain distinct; this reorganization does not claim full sharing of simulation or estimation implementations. The intersection currently uses a project-built prototype, not the instructor-supplied scaffold.

Read [freeway/README.md](freeway/README.md) and [intersection/README.md](intersection/README.md) for model details and remaining course work. Verification scripts and the observation/control modules are retained because they are part of the scientific implementation and evaluation, not disposable scaffolding.

## What was cleaned up

- Moved all freeway source and configuration into `freeway/`.
- Moved the signal runner and the former `signals/` package into `intersection/`.
- Replaced four competing root run/check scripts with one `run.py` entry point.
- Separated freeway dynamics from CSV/HTML export without changing the model.
- Moved outputs into matching `results/freeway/` and `results/intersection/` directories.
- Moved detailed documentation into the environment it describes.
- Retired unused historical snapshots in `archive/` and removed disposable bytecode and Finder metadata.

A verified full pre-reorganization backup, including the archived versions and previous results, is saved as `project-before-restructure.zip` in the Codex chat outputs. No model features or numerical results were intentionally removed or changed.

## Reproducibility and disclosure

Both environments use synthetic, uncalibrated models. Delay accounting includes unfinished vehicles and boundary/external queues; consult each branch for its exact definition. Operational improvements do not establish crash reduction.

Codex assisted implementation, translation, organization, and checks. Students must understand the code, verify assumptions, disclose AI use, and document their own contributions. The shared freeway estimation/forecasting/adaptive TMC, course scaffold integration, video exercise, component exchange, and course-specific reports remain separate requirements; this structure change does not complete them automatically.
