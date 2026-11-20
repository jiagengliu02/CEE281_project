# CEE C181/C281 Design Project: Detector-Based Traffic Control

Simulation and detector-based control experiments for a freeway work zone with ramp metering and an isolated signalized intersection (Pathway B), built on a shared Intelligent Driver Model (IDM) car-following framework.

**Authors:** Jiageng Liu (CEE C281) and Nasser Alsowyan (CEE C181)

---

## Repository Structure

- `freeway/`: IDM freeway simulation with a left-lane work-zone closure, on-ramp merging, loop/boundary detection, count-conservation estimation, forecasting, and ramp metering (`freeway/README.md`).
- `intersection/`: Isolated signalized intersection simulation (Pathway B) reusing the shared IDM model with advance and stop-line loop detectors, queue estimation, arrival forecasting, and four signal control modes (Webster fixed-time, actuated, adaptive queue + forecast pressure, and a no-forecast ablation) across four demand/fault scenarios (`intersection/README.md`).
- `run.py`: Single command-line entry point for running simulations and automated verification checks.
- `tools/make_figures.py`: Generates all static report figures from saved simulation outputs into `results/figures/`.
- `results/`: Reproducible experiment outputs, metrics, logs, summary reports (including `results/intersection/experiment_report.md`), and HTML viewers.
- `docs/`: Course project documentation and intermediate design records (`docs/design_record_review3.md` and `docs/design_record_review3_zh.md`).
- `view.html`, `freeway.html`, `intersection.html`: Standalone offline HTML viewers with embedded simulation data (open directly in any browser; no local server required). Note: `.html.in` files are HTML templates and cannot be opened directly.

---

## Requirements

- Python 3.9+
- `matplotlib` (required for `tools/make_figures.py`)

---

## Reproducing Verification Checks, Tables, and Figures

Run all commands from the repository root directory:

### 1. Run Verification Suite
Runs independent conservation, trajectory safety (zero vehicle overlaps, negative speeds, closed-lane entries, red-light runs, or simultaneous conflict-box occupancies), detector-count consistency, single-vehicle IDM tests, and time-step sensitivity checks across both environments:
```bash
python3 run.py check all
