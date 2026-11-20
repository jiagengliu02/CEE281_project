# CEE C181/C281 Design Project: Detector-Based Traffic Control

Simulation and detector-based control experiments for a freeway work zone with ramp metering and an isolated signalized intersection (Pathway B), built on a shared Intelligent Driver Model (IDM) car-following framework.

**Authors:** Jiageng Liu (CEE C281) and Nasser Alsowyan (CEE C181)

---

## Repository Structure

- `freeway/`: IDM freeway simulation with a left-lane work-zone closure, on-ramp merging, loop and boundary detection, count-conservation state estimation, forecasting, and ramp metering. See `freeway/README.md`.
- `intersection/`: Isolated signalized intersection simulation (Pathway B) reusing the shared IDM vehicle model with advance and stop-line loop detectors, queue estimation, arrival forecasting, and four control modes (Webster fixed-time, actuated, adaptive queue + forecast pressure, and a no-forecast ablation) across four scenarios. See `intersection/README.md`.
- `run.py`: Single command-line entry point for running simulations and automated verification checks.
- `tools/make_figures.py`: Generates static figures from existing simulation outputs into `results/figures/` (requires `matplotlib`).
- `results/`: Reproducible experiment data, metrics, logs, summary reports (including `results/intersection/experiment_report.md`), and offline HTML viewers.
- `docs/`: Course project documentation and intermediate design records (`docs/design_record_review3.md` and `docs/design_record_review3_zh.md`).
- `view.html`, `freeway.html`, `intersection.html`: Standalone offline HTML viewers with embedded simulation data (open directly in any browser; no local server required). Note: `.html.in` files are HTML templates and cannot be opened directly.

---

## Requirements

- Python 3.9+
- `matplotlib` (required for `tools/make_figures.py`)

---

## Reproducing Checks, Experiments, Tables, and Figures

Run all commands from the repository root directory:

### 1. Run Verification Suite
Runs conservation checks, trajectory safety assertions (zero vehicle overlaps, negative speeds, closed-lane entries, red-light runs, or simultaneous conflict-box occupancies), detector-count consistency checks, single-vehicle IDM tests, and time-step checks across both environments:

    python3 run.py check all

*(You can also run intersection-specific safety and consistency checks via `python3 run.py check intersection`.)*

### 2. Run Experiments & Reproduce Summary Tables
Executes the default freeway and intersection experiment suites, writes metrics, logs, and reports to `results/` (including `results/intersection/experiment_report.md`), and refreshes the matching offline HTML viewers in the root directory:

    python3 run.py freeway
    python3 run.py intersection

- **Custom output directories:** Pass `--output <dir>` to write custom experiment outputs to a specified directory without modifying the default viewer files.
- **Shared arrival tables & safety guarantees:** For intersection runs, the four controllers in each scenario share one arrival table, with zero red-light runs and zero simultaneous conflict-box occupancies enforced by verification checks.

### 3. Reproduce Report Figures
Builds static figures from existing simulation results into `results/figures/`:

    python3 tools/make_figures.py

---

## Interactive Offline Viewers

Double-click `view.html` to choose a simulation, or open `freeway.html` or `intersection.html` directly in a browser. Each page embeds its own data and requires no local server. In `intersection.html`, you can select a scenario and controller, play back the simulation, and inspect the phase timeline, true vs. estimated queues, controller decision traces at any time step, and the controller comparison table.
