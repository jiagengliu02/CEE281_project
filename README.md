# CEE281

## Important docs for Nasser

> - `intersection.html`: offline replay of the signal-control prototype. Pick a scenario and a controller, play the simulation, and see the phase timeline, true vs estimated queues, the decision trace at any moment, and the controller comparison table.
> - `docs/design_record_review3.md`: the design record for intermediate review 3 (English; `_zh` version in Chinese). It states our questions and predictions, the model, interfaces, verification, every experiment with its interpretation, and the current status of final pathway B.

Double-click `view.html` to choose a simulation, or open `freeway.html` / `intersection.html` directly. The pages embed their own data; no server is needed.

- `freeway/`: IDM freeway with an on-ramp, including detection, estimation, forecasting, feedback control and execution. See `freeway/README.md`.
- `intersection/`: isolated signalised intersection (final pathway B). Reuses the freeway IDM vehicle model; advance/stop-line loop detectors → queue estimate → arrival forecast → three controller types (Webster fixed-time, actuated, queue + forecast pressure) plus a no-forecast ablation; four scenarios. See `intersection/README.md`.
- `docs/`: course project requirements; `docs/design_record_review3_zh.md` (Chinese) and `docs/design_record_review3.md` (English) are the design records for intermediate review 3.
- `results/`: reproducible experiment data, metrics, logs, reports and HTML viewers.
- `run.py`: single command-line entry point.
- `tools/make_figures.py`: builds static figures from existing results into `results/figures/` (requires matplotlib).

Run from this directory:

```bash
python3 run.py freeway
python3 run.py intersection
python3 run.py check all
```

A default run also refreshes the matching viewer page in this directory. Custom experiments with `--output` update only the given directory. `.html.in` files are templates and cannot be opened directly.

For the intersection, the four controllers of each scenario share one arrival table; red-light runs and simultaneous conflict-box occupancy must be zero and are asserted by `run.py check intersection`. Comparison results, including the negative result that adaptive control is worse than fixed-time under heavy demand, are in `results/intersection/experiment_report.md`.
