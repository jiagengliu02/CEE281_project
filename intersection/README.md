# Signalized Intersection Environment (Final Pathway B)

This branch puts the freeway's IDM vehicles on four single-lane approaches of an isolated intersection and closes the loop with physical loop detectors, a queue estimator, an arrival forecast and three signal controllers. It reuses `freeway/behavior.py` (IDM, ballistic motion, vehicle state) and `freeway/drivers.py` (reproducible heterogeneous drivers) unchanged; everything intersection-specific lives here.

Run from the project root (Python 3.10+, standard library only):

```bash
python3 run.py intersection                      # 4 scenarios x 4 controllers -> results/intersection/
python3 run.py check intersection                # unit tests + independent audits -> verification.json
python3 run.py intersection --scenario heavy     # one scenario, same output folder
open results/intersection/viewer.html            # or intersection.html / view.html in the project root
```

The viewer replays any scenario/controller pair: top-down intersection with signal heads, advance loops and committed vehicles; phase timeline; true vs estimated zone queue and boundary queue; forecast vs actual; a six-stage decision trace at the current time; and the controller comparison table for the scenario.

## Design question and scope

*Can a detector-based actuated or queue-responsive controller reduce total delay relative to a justified fixed-time plan when demand changes, and what happens when detectors fail or storage fills?*

Scope follows the project guide: a small intersection, two through-only phases (N–S and E–W), no turns, no pedestrians (so no pedestrian benefit is claimed), no network. The instructor scaffold had not been supplied when this was written; the geometry and intersection mechanics are therefore project-defined and documented below so they can be swapped for the scaffold's.

## Geometry, vehicles and the stop line

| Element | Value | Note |
| --- | --- | --- |
| Approach | 300 m single lane per direction, entry at x = 0, stop line at x = 300 m | Finite storage ≈ 40 vehicles at jam; excess demand waits at the boundary and is counted as unmet demand |
| Conflict box | 20 m square | Vehicles of both groups inside the box at the same time is a logged and asserted violation |
| Exit leg | 60 m | Vehicles leave the model 380 m from the entry |
| Advance loop | 100 m upstream of the stop line | Reports counts and crossing speeds every 5 s |
| Stop-line loop | at the stop line | Same report format |
| Vehicles | freeway IDM; 5 m long; per-driver desired speed 14 ± 1.5 m/s (10–18), T 1.5 s, a 1.2 m/s², b 2 m/s², s0 2 m | Same driver population as the freeway except the urban desired speed |

Car following is the unmodified freeway IDM with the ballistic update. The stop line is handled by a kinematic rule rather than an IDM obstacle: a driver who must stop applies the constant deceleration that ends `min_gap` before the line, and the applied acceleration is `min(IDM, stopping)`, bounded to [−6, a] m/s². The IDM reacts far too strongly to a standing obstacle that appears suddenly (it is designed for gradual approach), so the kinematic rule is a declared modelling choice, not a calibrated driver model.

Yellow uses the ITE dilemma-zone rule: a driver stops if the required deceleration is at most 3.05 m/s² (10 ft/s²), otherwise the vehicle is *committed* and clears during yellow/all-red. Commitment is re-evaluated every step, so a committed vehicle slowed by its leader stops if it still can (up to emergency braking on red). Every stop-line crossing is logged with the phase seen by that approach: crossings while the opposing group is green or yellow are **red-light runs and must be zero**; crossings during all-red are reported separately.

Clearance intervals are derived from the model's speeds and checked in `verify.py`: yellow 3.5 s ≈ 1 + v85/(2·3.05) with v85 = 15.6 m/s (drivers here have no reaction time, so the 1 s term is a margin); all-red 2.5 s ≥ (box + vehicle length)/v15 = 25/12.45 = 2.0 s.

## Arrivals

Each scenario has piecewise-constant demand per approach. Arrival headways are exponential draws from a stream seeded by `arrival_seed` and the approach, so the same arrival table serves every controller of a scenario (the `demand_id` hash is exported with each run). Uniform headways were avoided because they lock onto the signal cycle and create artificial platoons. A vehicle enters at x = 0 at `min(desired speed, speed of the last vehicle)` when the gap allows (2 m + 1 s headway); otherwise it waits at the boundary, where its delay keeps accruing.

## Observation, estimation and prediction

1. **Detect** (`observation.py`, `ApproachDetectors`): two loops per approach report `sensor_id, interval_start_s, interval_end_s, count, cumulative_count, mean_speed_mps, quality_flag` every 5 s. Reports are delivered causally; a `missing` fault drops reports, a `delay` fault postpones `available_time_s`. The controller never receives positions, true queues or future arrivals.
2. **Estimate** (`SignalObservation.estimate`): `estimated_queue = cumulative advance crossings − cumulative stop-line crossings`, i.e. vehicles between the advance loop and the stop line, moving or stopped. It assumes an initially empty approach and recovers automatically after missing intervals because the counters are cumulative. Vehicles queued upstream of the advance loop are invisible; the zone saturates at about 13 vehicles. Quality is `unavailable` (no data yet), `missing`, `stale` (> 12 s old) or `ok`.
3. **Predict** (`SignalObservation.forecast`): advance-loop arrivals over the next 30 s per approach, EMA (α = 0.35) against a 6-report moving-average baseline. Both methods are issued at identical times and scored against the withheld counters from 120 s onward, only when the full horizon lies inside the run. Missing or stale data suppress forecasts.

Estimate MAE against the true zone count and forecast MAE per method are in each `metrics.json` and in `summary.csv`.

## Controllers (`control.py`)

All controllers share one two-phase state machine: green → yellow (3.5 s) → all-red (2.5 s) → the other group's green. Minimum green 8 s is never cut short; yellow and all-red are never skipped, including during fallback. Maximum green 60 s applies whenever an opposing call exists (estimated queue or recent arrivals on the other group); without a call the signal rests in green, as a real actuated controller does, so an approach with demand can never be starved.

| Mode | Rule | Fallback |
| --- | --- | --- |
| `fixed` | Webster plan computed once from the declared design demand (450 veh/h per approach), the measured saturation flow and lost time: C0 = (1.5 L + 5)/(1 − Y), g_i ∝ y_i. Default: 44.7 s green per group, 101.4 s cycle. Never retuned to a test scenario. | none needed |
| `actuated` | After minimum green, extend while the served approaches' advance loops reported at least one arrival in the last 5 s report (a 5 s unit extension); otherwise gap out. Max out at 60 s with an opposing call. | fixed plan while any approach report is unavailable, missing or stale |
| `adaptive` | Pressure = estimated zone queue + forecast arrivals over 30 s, summed per group. Switch when the served group is empty and quiet, or when opposing pressure exceeds served pressure by more than 2 vehicles. | same |
| `adaptive_no_forecast` | Identical rule with every forecast term set to zero (CEE281 ablation). | same |

Every command is logged in the guide's format (`issue_time_s, apply_time_s, actuator_id, action, value, units`) plus the reason, bounds, fallback flag and the estimates and forecasts the decision used, so any decision can be traced.

### Why the fixed plan looks long

The IDM population discharges slowly: the standing-queue check measures a 2.5 s saturation headway (1440 veh/h) and a 7.2 s start-up lost time, because vehicles accelerate at ≈ 1.2 m/s² and each one waits for its leader's gap to open. With 11 s lost per phase, Webster's optimum for 450 veh/h per approach is a 101 s cycle. The configured `saturation_flow_vph` and `startup_lost_s` must stay within 10 % / 1 s of the measured values or `check intersection` fails. This is a property of the driver model, not of real intersections (typical measured start-up lost time is about 2 s).

## Scenarios (`configs/default.json`)

| Scenario | Demand (veh/h per approach) | Faults |
| --- | --- | --- |
| `balanced` | 450 on all approaches | – |
| `directional_shift` | 450 → 700 N/S and 300 E/W from 300 to 660 s → 450 | – |
| `detector_fault` | as directional shift | N reports missing 360–480 s; E reports delayed 20 s 540–660 s |
| `heavy` | 800 on all approaches for 600 s, then 300 | – |

All runs last 900 s with dt 0.1 s. Delay counts every vehicle from its demand arrival, including boundary waiting and unfinished trips, as total time minus distance covered at the driver's desired speed.

## Results (regenerated by `python3 run.py intersection`)

From `results/intersection/experiment_report.md`, delay in veh·min, saving relative to fixed:

| Scenario | fixed | actuated | adaptive | adaptive_no_forecast |
| --- | ---: | ---: | ---: | ---: |
| balanced | 260.5 | 330.2 (−69.7) | 246.3 (+14.2) | 551.2 (−290.7) |
| directional_shift | 469.0 | 358.3 (+110.6) | 381.8 (+87.2) | 695.6 (−226.6) |
| detector_fault | 469.0 | 421.4 (+47.5) | 351.6 (+117.4) | 592.9 (−123.9) |
| heavy | 1458.5 | 1426.0 (+32.5) | 1969.5 (−511.0) | 2376.9 (−918.4) |

What the numbers say about this model:

- **Balanced demand**: the Webster plan is hard to beat; actuation loses because gap-outs with a 5 s unit extension produce short greens, and every extra phase costs 11 s of lost time. The pressure rule gains 5 %.
- **Directional shift**: both responsive controllers beat the fixed plan, which cannot move green to N–S (fixed capacity ≈ 564 veh/h per approach against 700 veh/h demand). The actuated controller is slightly better than the pressure rule here.
- **Detector faults**: 238 s of fallback. The pressure rule still gains because the healthy approaches keep it adaptive outside the fault windows; the estimate MAE rises from 0.40 to 0.55 vehicles and recovers after the faults.
- **Heavy demand**: a supported negative result. Both detector zones saturate at ≈ 13 vehicles, so the pressure comparison sees two equal, full zones and switches shortly after minimum green on forecast noise (28 greens vs 18), wasting lost time while hundreds of vehicles wait beyond the loops. The actuated controller, which only asks "are vehicles still arriving", behaves like a long fixed plan and does slightly better than Webster. A queue estimator that only sees 100 m is the wrong sensor for an oversaturated intersection; this is the design revision the final report should test (longer min green under saturation, or a saturation-aware rule).
- **Ablation**: removing the forecast makes the pressure rule much worse everywhere (41–42 greens). The mechanism is not prediction accuracy (MAE ≈ 2.2 vehicles per 30 s for both methods, with the moving average slightly better than EMA): during green, vehicles that were queued *beyond* the advance loop start moving and cross it, so the served group's forecast rises and holds the green until the hidden queue has passed. The forecast term works as a proxy for the invisible upstream queue; without it the rule switches as soon as the visible zone drains.

Red-light runs and conflict-box violations are zero in every default run; all-red crossings by committed vehicles are also zero. These are model-consistency checks, not safety claims.

## Verification (`python3 run.py check intersection`)

`test_signal.py` (12 small examples): queue = advance − stop-line counts; missing/recovery/staleness; delayed reports cannot be used early and never overwrite newer data; arrival tables are controller-independent; the Webster plan by hand; minimum green and clearance sequencing; actuated gap-out, max-out and rest; forecast changes the adaptive decision and the ablation does not see it; fallback runs the fixed plan without skipping clearance; a complete loop with faults scores forecasts on matched horizons; future demand cannot change past commands; zero demand.

`verify.py` audits every default run from the raw logs: conservation, no overlap, no backward motion, loop counts recomputed from trajectories, stop-line crossings versus the independent phase timeline (no red runs), conflict-box occupancy never contains both groups, yellow/all-red durations exact, every green ≥ minimum, greens beyond maximum only while resting or in fallback, fixed greens equal to the plan. It also measures saturation flow and start-up lost time, checks the ITE clearance intervals, confirms fallback during and recovery after the declared faults, clears a finite pulse under every controller, keeps unmet demand at the boundary under a 2400 veh/h overload while the zone estimate saturates, and reports dt/2 sensitivity for the directional shift. Fixed control is insensitive to dt; the pressure rule is not (its thresholded decisions depend on report timing), which is reported rather than hidden.

## Files

| File | Responsibility |
| --- | --- |
| `intersection/model.py` | `IntersectionSimulation`: arrivals, boundary entry, stop-line rule, IDM motion, loops, crossings, metrics, replay frames |
| `intersection/observation.py` | `ApproachDetectors` (reports, faults, causal delivery), `SignalObservation` (estimates, forecasts) |
| `intersection/control.py` | `webster_plan`, `SignalState`, `SignalController` (fixed / actuated / adaptive / ablation) |
| `intersection/run.py` | Scenario × controller experiments, CSV/JSON export, summary, report, viewer |
| `intersection/test_signal.py` | Small independent examples for the pipeline and controllers |
| `intersection/verify.py` | Physical and log audits; writes `results/intersection/verification.json` |
| `intersection/viewer_template.html.in` | Offline replay and plots |
| `results/intersection/<scenario>/<controller>/` | `traces.csv` (0.5 s), `crossings.csv`, `observations.csv`, `estimates.csv`, `forecasts.csv`, `control_log.csv`, `phases.csv`, `arrivals.csv`, `drivers.csv`, `metrics.json` |

## Sources and AI disclosure

| Design choice | Basis |
| --- | --- |
| Car following, parameters, ballistic update | Treiber, Hennecke & Helbing (2000); reused freeway code |
| Webster fixed-time plan | Webster, F. V. (1958). *Traffic Signal Settings*, Road Research Technical Paper 39; FHWA Traffic Signal Timing Manual (2008), ch. 5 |
| Yellow and all-red intervals, 3.05 m/s² stopping deceleration | ITE Traffic Engineering Handbook clearance-interval formulas; FHWA Signal Timing Manual, ch. 5 |
| Actuated minimum green, unit extension, gap-out/max-out, rest in green | FHWA Signal Timing Manual, ch. 5 (basic actuated controller operation) |
| Cumulative-count queue estimate | Input–output (cumulative curve) method, Daganzo (1997) ch. 2 |
| Pressure rule, thresholds, detector placement, geometry, demand, all numerical values | Explicit, uncalibrated project assumptions |
| Simulation, detectors, accounting, visualization | Project implementation; no third-party code copied |

Claude assisted the redesign of this branch (replacing an earlier point-queue prototype), the verification scripts and this README; all numbers come from the reproducible runs in this repository and were checked with `python3 run.py check intersection`. Students must understand, verify and extend the work and disclose their actual contributions.
