# CEE281 Signal-Control Branch: Implementation and Experiments

## Run and visualize

Python 3.10+; standard library only. The freeway remains independently runnable.

```bash
cd /Users/liujiageng/Desktop/CEE281/project
python3 run.py intersection
python3 run.py check intersection
open results/intersection/viewer.html
```

Alternatively open `view.html` and choose Signalized intersection. The viewer is self-contained and works offline in Safari or Chrome. It compares fixed-time and responsive control at the same replay time, with four scenarios, phase timelines, queue truth versus estimates, and a multi-seed comparison table. Vehicle blocks illustrate queue counts; they are not IDM spatial trajectories.

For a smaller run:

```bash
python3 run.py intersection --scenario directional_shift --seed 11 --output results/intersection_demo
```

Copy `intersection/configs/default.json` before making custom changes and pass the copy using `--config`. A run replaces matching files in its output directory. The default experiment has four scenarios, three controllers, and three seeds: 36 runs.

## Research question and scope

For an isolated intersection with four single-lane through approaches, can signal timing based on estimated queues reduce waiting delay relative to a justified fixed-time plan? What happens under directional demand shifts, detector faults, and full approach storage?

This is a project-built intersection prototype. No instructor-supplied intersection scaffold was present in the directory, so this implementation does not claim to use it. Once the scaffold is available, first adapt arrivals, admission, and discharge in `intersection/model.py`, preserving observation and control interfaces where possible. Recheck geometry, lane layout, and phase compatibility before repeating validation.

The original `freeway/model.py` is a multilane IDM work-zone model. This branch uses the simpler queue model permitted by the guide, independently of the freeway dynamics. It follows the same seconds-based timing, interval observations, command timestamps, CSV/JSON outputs, and offline HTML approach. Completing this prototype does not complete all shared-foundation course requirements.

## Modules and reading order

Paths are relative to the project root. `run.py` is the only command-line entry point; branch modules are imported through it.

| File | Responsibility | Start with |
| --- | --- | --- |
| `intersection/configs/default.json` | Demand, timing, storage, control, and observation parameters | `scenarios`, `fixed_green_s` |
| `intersection/model.py` | Arrivals, finite queues, stop-line service, truth, and metrics | `make_arrivals()`, `IntersectionSimulation.run()` |
| `intersection/detector.py` | Interval entry/stop-line counts and injected missing/delayed faults | `StopLineDetectors.report()` |
| `intersection/observation.py` | Queue estimation and arrival forecasting from detector reports | `receive()`, `estimates()`, `predict()` |
| `intersection/control.py` | Bounded phase state machine and three policies | `SignalController.update()` |
| `intersection/run.py` | Paired experiments, export, and aggregation | `run_experiments()` |
| `intersection/verify.py` | Independent count, analytic, timing, and causality checks | `audit()`, saturated-queue test |
| `intersection/viewer_template.html` | Offline replay and plots | Controls and drawing functions |

Data flow: demand events → road/external queues → entry and stop-line counts → queue estimate → control decision → green/yellow/all-red state → discharge. Future arrivals, true queues, and vehicle histories belong to the simulator/evaluation layer. The controller receives estimates and quality flags only.

## Traffic model

Each approach has storage for 35 vehicles. When full, additional demand remains in an unlimited external FIFO queue and continues accumulating delay. External queues represent unmet admission demand; their spatial propagation into an upstream network is not modeled.

Each vehicle records demand arrival, admission, and stop-line departure times. Admission places it directly in the approach queue, with no free-flow travel segment. Speed and spacing are not modeled.

Each green begins with a 2 s startup loss. Under continuous queuing, each served approach releases one vehicle every 2 s, equivalent to a saturation flow of 1800 veh/h. Unused service opportunities do not accumulate for an instantaneous burst later. Yellow and all-red permit no new stop-line departures. These numerical values are teaching assumptions, not empirically calibrated values or recommendations for a real intersection.

Demand follows piecewise Poisson processes with independent random streams by approach. All controllers use the same pre-generated arrival table and export the same `demand_id` hash for auditing. Processing rounds arrivals up to the simulation grid, while delay accounting retains the original demand time. The default timestep is 0.5 s; verification also uses 0.25 s.

## Phase plan and controllers

The sequence is `NS_green → NS_yellow → NS_all_red → EW_green → EW_yellow → EW_all_red → NS_green`.

- Compatible movements are north/south through or east/west through. Turns and pedestrians are excluded.
- Yellow lasts 3 s and all-red 1 s. Minimum green is 10 s; maximum green is 55 s.
- `fixed`: 30 s green in each direction group, giving a 68 s cycle. Equal green matches the symmetric development demand of 450 veh/h per approach. With startup loss, sustained capacity is about 741 veh/h per approach. This is not a globally optimal plan and is not retuned to test demand. A stronger final study could optimize cycle and green split on separate development scenarios.
- `adaptive`: after minimum green, request a switch if the opposing estimated queue exceeds the current estimated queue by more than 3 vehicles, or if the current estimated queue is empty and opposing demand exists. Maximum green forces rotation. Every switch still passes through yellow and all-red.
- `adaptive_no_hysteresis`: change only the switching queue-difference threshold from 3 to 0, isolating that design choice for the CEE281 ablation.
- If any approach observation is unavailable, missing, or older than 12 s, responsive control falls back to fixed green durations without bypassing clearance. Normal response resumes when observations recover. Startup before the first report also uses fallback.

This is a project-defined teaching policy, not a complete implementation of a standard commercial actuated controller. Maximum-green rotation prevents an approach from being denied service indefinitely under normal timing.

## Observations, estimation, and forecasting

Every 5 s, each approach produces an entry/exit report. `count` means vehicles admitted to road storage; `departure_count` means vehicles crossing the stop line. Speed is not measurable and remains explicitly missing.

Two cumulative counters accompany the interval counts. Starting from empty approaches, estimated road queue = cumulative admissions − cumulative departures. The estimator holds its last value between reports, so even healthy data have sampling lag. External demand waiting for admission is hidden from the controller.

Missing reports have null counts. Cumulative counters allow later healthy reports to recover after a missing interval. Delayed reports cannot be used before `available_time_s`, and older reports never overwrite newer ones. Freshness is measured from the observation interval end, not the receipt time.

The forecast target is **admitted arrivals over the next 30 s**, not hidden external demand; they differ when storage is full. Methods are a moving average over six reports and EMA with alpha = 0.35. Parameters are fixed in advance and not fitted on evaluation results. The first 300 s establish history/development context. Later evaluation compares identical issue times and horizons using MAE in vehicles; horizons extending beyond the run are not scored. Missing/stale data suppress new forecasts; rates spanning a report gap average over that full gap.

Forecasts are advisory in this version and do not drive control. CSV outputs retain issue time, target time, eventual truth, and error. The implementation does not assume that EMA must outperform the moving-average baseline.

## Scenarios and outputs

1. `balanced`: 450 veh/h per approach throughout.
2. `directional_shift`: after 300 s, north/south each receive 900 veh/h; east/west each receive 240 veh/h.
3. `detector_fault`: the same directional shift; N reports are missing from 420–540 s; E reports are delayed by 20 s from 600–720 s. Fault selection uses the report interval end, including the start but excluding the end of the fault window.
4. `heavy`: 1100 veh/h per approach throughout, exposing storage limits, external accumulation, and residual queues.

Each `results/intersection/<scenario>/<seed>/<controller>/` contains:

- `observations.csv`: detector records and availability times.
- `estimates.csv`, `forecasts.csv`: estimates, quality, and equal-horizon forecast scoring.
- `control_log.csv`: issue/apply times, phase, reason, fallback state, and estimated queues used in the decision.
- `events.csv`, `vehicles.csv`: actual arrival/admission/departure events and individual accrued waits for independent audits.
- `metrics.json`: versions, seed, demand hash, delay, throughput, approach queues, service imbalance, and forecast errors.

The top-level output includes the configuration copy, `summary.csv`, `experiment_report.md`, `verification.json`, `data.json`, and `viewer.html`. Replay shows the first seed; the table averages all seeds, so their numbers can differ.

## Metrics and verification

Accrued delay for each vehicle is `min(departure time, simulation end) - original demand arrival time`. The total includes departed vehicles, vehicles still on the approach, and vehicles waiting outside. It is point-queue waiting time, not complete HCM control delay including deceleration and acceleration.

Approach mean delay divides by all generated vehicles and measures waiting accrued by the analysis endpoint, rather than eventual completed-trip delay. Service imbalance is the range of these approach means. There is no extra clearance tail; residual queues are reported separately. Throughput is departures divided by the full analysis duration.

The 11 test methods cover:

- Conservation, FIFO ordering, and independent event-to-detector count audits across all four scenarios and three controllers.
- Phase sequence, minimum/maximum green, clearance duration, green-only departures, and saturation headways.
- An independent saturated-queue example: 50 vehicles on one approach, 14 departures within 40 s, and 1650 veh·s total delay.
- Zero demand and complete clearing of a finite arrival pulse.
- No early access to delayed observations; cumulative-count recovery and out-of-order report protection.
- Correct forecast horizons and equal samples for both forecast methods.
- Control using estimates only; changing future demand cannot change past decisions.
- Timestep halving and rejection of invalid configurations.

## Sources, AI disclosure, and remaining course work

| Design basis | Source | Use and limitations |
| --- | --- | --- |
| Minimum/maximum green, yellow, all-red, detector response | FHWA Traffic Signal Timing Manual, Chapter 5: https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter5.htm | Parameter meanings and state constraints; numerical settings are assumptions and the queue-difference policy is project-defined |
| Detectors and actuated-control timing | FHWA Traffic Detector Handbook, Chapter 4: https://www.fhwa.dot.gov/publications/research/operations/its/06108/04.cfm | Observation/control background; no hardware simulation or copied source code |
| Queue conservation and event accounting | Vehicle conservation and independent analytic tests | Numerical verification, not empirical calibration |
| Python, controller, experiments, and viewer | Newly implemented with Codex assistance | No third-party implementation copied; actual checks are recorded |

Students must understand and verify the model, code, parameters, and conclusions and disclose AI assistance. This document does not invent names or individual contribution records.

The current branch provides a runnable signal prototype, comparisons, and a design-choice ablation. Before course submission: check/integrate the instructor scaffold; add individual contribution records and pre-experiment predictions; update the design record in response to feedback; complete the video observation exercise and component-exchange evidence; complete shared freeway estimation, prediction, and adaptive TMC work; and supplement literature and independent validation as needed. This prototype is not completion of every course requirement.
