# Freeway and Ramp Environment

This branch owns the existing three-lane IDM model, ramp entry, work-zone merging, and detector outputs. It includes a five-stage forecast-assisted ramp TMC alongside fixed and unmetered baselines.

Run from the project root:

```bash
python3 run.py freeway
python3 run.py check freeway
open results/freeway/viewer.html
```

The viewer has five modes: forecast-assisted metering, adaptive metering with detector faults, and the three original baselines: Open road, Closure with an unmetered ramp, and Closure with fixed ramp metering. Compare the two closure cases to isolate ramp-control effects. Use Play, the time slider, and Watch a merge to inspect motion. Charts include trajectories, queues, detector flow, and speed. The intersection uses a different queue model and is documented in `intersection/README.md`.

## Freeway environment

The current freeway has three lanes. The left lane tapers from 500 to 650 m and is fully closed from 650 to 850 m, creating a three-to-two-lane merging bottleneck. There is no desired-speed reduction zone: slowing emerges from car following, merging, and queuing.

Three paired runs use identical arrivals, driver identities, desired speeds, and initially empty roads:

- `open`: no work zone; all three lanes open; ramp vehicles enter whenever the gap rules permit.
- `closure`: left-lane closure; unmetered ramp.
- `fixed`: the same closure; at least 12 seconds between successful ramp releases.
- `adaptive`: the same closure; measured boundary counters and EMA forecasts set a 3–20 s release interval.
- `adaptive_fault`: identical adaptive policy and demand, with missing reports at 300–360 s and 45 s delivery delays at 420–480 s (fault windows use interval end times).

Vehicle identity is assigned at arrival, and desired speed follows from that identity. Differences in insertion order therefore do not change driver parameters.

Default demand is total mainline demand across all lanes: 1800 mainline + 300 ramp veh/h from 0–240 s; 3000 + 450 from 240–480 s; 1800 + 300 from 480–600 s. Mainline arrivals rotate across lane-entry queues.

## Vehicle motion: IDM

Position is the front-bumper location in meters; speed is in m/s. Net gap is the distance from the preceding vehicle rear to the following vehicle front. Vehicle length is 5 m. Desired speed is one of 25, 26.5, 28, 29.5, or 31 m/s, determined by ID. Desired time headway is 1.5 s, standstill gap 2 m, acceleration parameter 1.2 m/s², and comfortable deceleration 2 m/s².

```text
s* = s0 + max(0, v T + v (v - v_leader) / (2 sqrt(a b)))
acceleration = a [1 - (v / v0)^4 - (s* / s)^2]
```

Vehicles follow the leader in their current lane. All longitudinal accelerations are calculated from the same time snapshot before motion is updated. Ballistic integration handles stopping within a timestep to prevent negative speeds. Acceleration is additionally bounded to [-6, 1.2] m/s², an explicit assumption of this prototype.

## How the work zone creates a bottleneck

Lane IDs are 0 = left, 1 = center, and 2 = right. Calling the left lane a passing lane does not imply a complete model of traffic law or passing-lane behavior.

1. After 350 m, drivers in lane 0 seek a merge into lane 1. Urgency grows linearly with distance toward the usable lane end; a quadratic urgency bonus (up to 2 m/s²) increases the lane-change incentive. At urgency 0.8, a safe merge can bypass the incentive threshold.
2. The work-zone boundary slopes across lane 0 from 500 to 650 m. With a 3.6 m lane, 1.8 m vehicle width and 0.2 m margin on each side, the remaining corridor becomes too narrow at 558.33 m. This conservative usable lane end is the stationary IDM obstacle; unmerged vehicles stop before it, rather than entering the narrowing area without adequate room.
3. Courtesy requests begin at urgency 0.35. Courtesy braking increases with urgency up to the existing 2 m/s² bound. A follower already alongside the merger is allowed to pass instead of being held in an unusable gap. Actual leader braking still takes precedence, and front/rear gap and unclipped IDM braking checks remain mandatory at every urgency.
4. Lane 0 is unavailable from the usable lane end through the work zone. A vehicle can return only when its entire body is beyond the closure.

This remains a discrete-lane model. The viewer draws a triangular taper and places closing-lane vehicles within the remaining corridor. Animated departing vehicles are kept clear of the taper visually. These lateral positions are display-only, not continuous steering or a two-dimensional collision model. Geometry, urgency thresholds, and courtesy strength are uncalibrated teaching assumptions. Configurations without `taper_start_m` retain the original abrupt lane-end behavior.

Low demand may produce little slowing. At higher demand, merging disturbances reduce effective discharge and queues extend upstream. Compare the open-road and closure cases under identical demand to identify this effect.

## Lane changing: a simplified MOBIL-inspired rule

Lane changes are assessed every 0.2 s. Voluntary changes consider the driver acceleration gain and changes to affected followers, with a politeness weight of 0.2, an incentive threshold of 0.25 m/s², and a 0.15 m/s² right-lane bias. A driver waits at least 5 s between lane changes.

High-urgency work-zone merges can bypass the incentive threshold but not the safety checks. Front and rear net gaps must be at least 1.5 m. The raw IDM accelerations of both the merging driver and the target-lane follower must be no lower than -3 m/s². Safety checks use unclipped accelerations so a braking cap cannot conceal an unsafe gap.

Vehicles approaching the closure are considered first, then from downstream to upstream. Lane occupancy updates after every accepted move, preventing several vehicles from claiming the same old gap. Only adjacent-lane changes are allowed.

The rule borrows from MOBIL but adds project-specific work-zone priorities, exclusion areas, and keep-right assumptions. It is not a calibrated implementation of the complete original MOBIL model. There is no continuous lateral dynamics, reaction delay, communication, or CAV control.

A simplified alternating-yield rule lets the foremost closing-lane vehicle request space from a target-lane follower. The requested courtesy braking is scaled by urgency and limited to 2 m/s² with a 4 m extra gap buffer; stronger braking required by the actual leader is retained. After yielding to one merging vehicle, that follower receives its own passage opportunity. This is an uncalibrated behavioral assumption, not part of the original MOBIL specification.

A lane change instantly changes `lane_id`. Smooth lateral movement in the viewer is a display effect and cannot support collision-risk analysis.

## Ramp entry, observations, and metrics

The ramp is a 200 m single-lane IDM road with a desired speed of 12 m/s, entering lane 2 at 250 m. Vehicles enter the ramp at rest, follow their leader and slow at the downstream metering/merge point. The first vehicle transfers within 1 m of the endpoint only with meter permission and safe mainline gaps, keeping its identity. Entry speed follows the original mainline rule: the lesser of desired speed and the current leader speed (desired speed if no leader). The low ramp stopping speed is not carried onto the freeway; this is an explicit discrete handoff assumption, not a simulated acceleration lane. After entry, ordinary mainline IDM controls acceleration and braking. Ramp vehicles stay in the right lane for the first 45 m and respect the lane-change cooldown. Excess demand waits upstream of the ramp. All on-ramp and upstream vehicles count toward system time. Boundary storage is unlimited in this freeway version; there is no upstream spillback model.

Detectors at 200, 550, and 900 m report crossing counts and crossing-mean speeds for each lane every 30 s. Mean speed remains missing if no vehicle crosses. Combined-lane mean speed is weighted by crossing counts.

- Conservation is checked every timestep: generated = exited + on-road + boundary waiting.
- Total system time integrates all on-road and boundary-waiting vehicles over time.
- Total delay subtracts mainline traveled distance divided by each vehicle desired speed and ramp traveled distance divided by the 12 m/s ramp desired speed from system time. It includes unfinished trips and boundary waits, in veh·s.
- Throughput is the exit count divided by the analysis duration, converted to veh/h.
- The road queue is the count of vehicles below 5 m/s.
- Outputs include residual queues, lane changes, mandatory merges, minimum same-lane gap, strongest braking, and numerical guard interventions.

A numerical guard is recorded if a motion update would overlap a leader or cross a closed-lane boundary. No guards trigger in the default runs and verification scenarios. This does not establish improved real-world safety.

Watch a merge selects the next recorded work-zone merge and replays from 3 s before to 4 s after it at 1× speed. Yellow highlights the tracked vehicle; blue identifies yielding vehicles. Positions interpolate between snapshots; lateral interpolation does not affect physics.

Road replay and traffic-performance plots use simulator truth. The new estimate plots distinguish measured estimates from truth. Human drivers see neighboring vehicles and the closure; ramp TMC receives only boundary detector reports. Fixed control uses the clock; adaptive control uses estimates and forecasts. Physical insertion safety is enforced after the meter issues a permit.

## Code and outputs

Paths below are relative to the project root. Run commands from that root.

| File | Responsibility |
| --- | --- |
| `freeway/behavior.py` | Vehicle state, IDM car following, MOBIL-style lane changes, merge courtesy |
| `freeway/model.py` | Simulation loop (`FreewaySimulation`): arrivals, road entry, ramp control, detectors, metrics |
| `freeway/run.py` | The three paired experiments and CSV/HTML export |
| `freeway/verify.py` | Independent physical checks and timestep sensitivity |
| `freeway/configs/default.json` | Default road, ramp, work-zone, and demand settings |
| `freeway/configs/heavy_demand.json` | Higher-demand configuration |
| `freeway/viewer_template.html.in` | Offline replay and plots |
| `results/freeway/` | Generated results; separate from source code |

The output contains `open/`, `closure/`, `fixed/`, `adaptive/`, and `adaptive_fault/` run folders, plus `data.json`, `viewer.html`, and `verification.json` after checks. Each run folder contains traces, observations, control logs, lane changes, yield logs, and metrics.

Useful parameters include `merge_anticipation_m`, `closure_start_m`, `closure_end_m`, `lane_change_safe_braking_mps2`, `politeness`, and `demand`. The prototype supports a left-lane closure; changes to road layout or lane count also require corresponding viewer changes.

## Default freeway results and checks

Default demand was reduced to make merging easier to inspect. To run the earlier higher-demand configuration:

```bash
python3 run.py freeway --config freeway/configs/heavy_demand.json --output results/freeway_heavy
```

The default 10-minute results are regenerated in `results/freeway/experiment_report.md`; they include all five control modes and can change substantially with taper geometry and merge behavior. Do not reuse the previous abrupt-closure result numbers for this version.

Independent checks cover IDM equilibrium, free motion, stopping, unsafe merge rejection at every urgency, taper usable-width geometry, increasing merge incentive, yielding and subsequent passage, lane-specific detector counts, overlaps, zero demand, and boundary overload. A finite arrival pulse must fully clear. Timestep sensitivity is reported in `results/freeway/verification.json`; it is not a convergence proof or empirical validation.

## Sources and AI disclosure

- IDM: Treiber, Hennecke & Helbing (2000), *Congested traffic states in empirical observations and microscopic simulations*, Physical Review E 62, 1805–1824. https://doi.org/10.1103/PhysRevE.62.1805
- MOBIL: Kesting, Treiber & Helbing (2007), *General Lane-Changing Model MOBIL for Car-Following Models*, Transportation Research Record 1999, 86–94. https://doi.org/10.3141/1999-10
- Author documentation: https://mtreiber.de/MicroApplet/IDM.html and https://mtreiber.de/MicroApplet/MOBIL.html

| Design choice | Basis |
| --- | --- |
| Longitudinal following and parameter meanings | IDM paper and author documentation |
| Ballistic integration and within-step stopping | Author IDM documentation |
| Lane-change incentives and follower braking checks | MOBIL paper and author documentation |
| Work-zone geometry, mandatory-merge priorities, demand, boundaries, and numerical values | Explicit, uncalibrated prototype assumptions |
| Simulation, detectors, accounting, and visualization | Project implementation; no third-party source code copied |

Codex assisted implementation, verification, English translation, and browser checks. Students must understand, verify, and extend the work and disclose their actual contributions. The freeway now includes boundary observation, conservation estimates, forecasts, adaptive ramp control and fault fallback. This is an uncalibrated teaching controller; video observations, component-exchange evidence, external validation, individual contributions and the final pathway requirements remain separate work.


## Five-stage ramp loop

1. **Detect** (`observation.py`, `BoundaryDetectors`): virtual event counters measure mainline demand arrivals, actual mainline entries, ramp demand arrivals, actual ramp entries and freeway exits. Reports are delivered every 30 s. Existing lane-specific crossing/speed detectors remain available in `observations.csv`. The boundary measurement bundle is in `boundary_observations.csv`; speed is explicitly unavailable there. Detector faults affect this bundle, not the independent loop-speed plots.
2. **Estimate** (`RampObservation.estimate`): road inventory = measured mainline entries + ramp entries - exits; ramp queue = ramp arrivals - ramp entries; mainline boundary queue = mainline arrivals - mainline entries. Counts are cumulative and recover across missed reports. Estimates are held between reports and compared with withheld simulator truth. They assume empty initial conditions and complete boundary detection. Road inventory is not a count of slow vehicles.
3. **Predict** (`RampObservation.forecast`): predict the next 30 s of each counter using EMA (alpha 0.35) and a 3-report moving-average baseline. Both use the same issue times and horizons. Forecast MAE is scored in vehicles from 120 s onward, only when the complete target horizon lies inside the simulation. Prediction truth is attached after the run. Missing/stale reports suppress forecasts. Old delayed reports never overwrite newer data.
4. **Decide** (`control.py`): estimate the road inventory after one horizon without further ramp entries, `N0 = max(0, N_est + main_entry_forecast - exit_forecast)`. Request `r = (N_target - N0) / H` vehicles/s, bounded by intervals of 3–20 s. The fixed target is 45 vehicles. If the predicted ramp queue exceeds a soft 15-vehicle threshold, request faster service within the same bounds. These are declared teaching parameters, not calibrated capacities. This is a project-defined conservation rule, not ALINEA or an optimized MPC implementation.
5. **Execute** (`model.py`, `control_ramp`): apply the interval since the last successful entry, then check the physical merge gap and IDM follower braking. A permit may remain unexecuted if no safe gap or no vehicle exists. Successful entry resets the meter clock. Once on the freeway, ramp vehicles follow the same IDM as mainline vehicles. The ramp uses IDM movement with finite road space and unlimited upstream FIFO storage; its 15-vehicle control threshold is not physical storage. The estimated ramp inventory includes moving vehicles as well as stopped and upstream vehicles.

The signal pathway remains separate. Predictions here **actively affect** the meter. Healthy information drives the adaptive rule; unavailable, missing or older-than-45-s information triggers the 12-s fixed fallback. Startup also uses fallback. The full loop runs each simulation timestep, with sensor information changing only on delivery.

### Outputs and verification

- `boundary_observations.csv`: counts, cumulative counts, interval boundaries, delivery time and quality.
- `estimates.csv`: causal road inventory and boundary queue estimates.
- `forecasts.csv`: method, issue/target times, predicted counts, evaluation truth and absolute error.
- `control_log.csv`: permit/hold command, interval, reason, estimates/forecasts used, fallback, actual release/hold response and last successful release.
- `metrics.json`: total delay including all waits, residual queues, estimate MAE, matched forecast MAE, fallback duration, physical audits and version IDs.
- The replay includes a five-stage trace, truth-versus-estimate plots, forecast comparison and meter interval history.

Run `python3 run.py check freeway`. Checks include independent trajectory/detector audits for all five modes, analytic estimator accounting, fault recovery, delivery causality, forecast-to-action sensitivity, queue relief and bounds, actual release spacing, equal-horizon forecast scoring, zero demand and future-demand noninterference. The default demand shift is built in. A separate stress run is `python3 run.py freeway --config freeway/configs/heavy_demand.json --output results/freeway_heavy`.

This first controller can perform worse than fixed metering. Inventory predictions extrapolate past measured rates and can react poorly to bottleneck changes; queue relief may trade mainline congestion for ramp waiting. Preserve negative results and evaluate parameter choices on separate development scenarios before claiming improvement. See `results/freeway/experiment_report.md` for the measured comparison.

Design reference: [FHWA Ramp Management and Control Handbook, Chapter 5](https://ops.fhwa.dot.gov/publications/ramp_mgmt_handbook/manual/manual/5_1.htm), for traffic-responsive metering and ramp queue management concepts. The equations above are explicitly project-defined. No external controller source code was copied.

## Ramp movement and replay

Orange vehicles travel along the ramp curve with recorded IDM positions and speeds, queue behind their leader, and enter the mainline with the same ID. Replay interpolates through the recorded entry timestamp, so a vehicle crosses from the ramp endpoint to the mainline without disappearing or appearing twice. The ramp waiting path ends one displayed lane width outside the mainline. After a successful entry, the vehicle blends toward the right-lane centre over the next 45 m; this lateral movement is display-only. The meter is drawn beside the merge point. The former queue icons and release flash are removed. `ramp_vehicles` and `ramp_upstream` are exported in replay frames; mainline vehicle arrays also include the entry timestamp and entry speed. The curve is a display mapping of longitudinal ramp distance, not a lateral steering model.

The default replay begins at 60 seconds. “Watch ramp cars” finds a frame with moving ramp vehicles. Ramp motion changes travel time and merge speeds, so experiment results have been regenerated and should not be compared with the previous point-queue results as though only styling changed.

Optional replay regression check: `node freeway/check_replay.cjs`. This checks orange vehicle polygon overlaps near the merge at quarter-second samples in all five generated runs (1200 px canvas); it complements, rather than replaces, simulation safety audits.
