# Freeway and Ramp Environment

This branch owns the existing three-lane IDM model, ramp entry, work-zone merging, and detector outputs. It is the common freeway foundation; fixed metering is not yet an adaptive TMC.

Run from the project root:

```bash
python3 run.py freeway
python3 run.py check freeway
open results/freeway/viewer.html
```

The viewer has three modes: Open road, Closure with an unmetered ramp, and Closure with fixed ramp metering. Compare the two closure cases to isolate ramp-control effects. Use Play, the time slider, and Watch a merge to inspect motion. Charts include trajectories, queues, detector flow, and speed. The intersection uses a different queue model and is documented in `intersection/README.md`.

## Freeway environment

The current freeway has three lanes. The left lane closes from 650 to 850 m, creating a three-to-two-lane merging bottleneck. There is no desired-speed reduction zone: slowing emerges from car following, merging, and queuing.

Three paired runs use identical arrivals, driver identities, desired speeds, and initially empty roads:

- `open`: no work zone; all three lanes open; ramp vehicles enter whenever the gap rules permit.
- `closure`: left-lane closure; unmetered ramp.
- `fixed`: the same closure; at least 12 seconds between successful ramp releases.

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

1. After 350 m, drivers in lane 0 try to merge into lane 1.
2. At 650 m, the closure acts as a stationary obstacle. Drivers who cannot merge slow and stop, creating a following queue.
3. Merging vehicles occupy gaps in lane 1 and alter following speeds and spacing. Vehicles can also change between adjacent available lanes.
4. Lane 0 is unavailable between 650 and 850 m. A vehicle can return only when its entire body is beyond the closure.

Cones in the advance-merge area are illustrative; the longitudinal stopping obstacle is at 650 m. This is not a geometric simulation of a continuous lane taper or collision geometry.

Low demand may produce little slowing. At higher demand, merging disturbances reduce effective discharge and queues extend upstream. Compare the open-road and closure cases under identical demand to identify this effect.

## Lane changing: a simplified MOBIL-inspired rule

Lane changes are assessed every 0.2 s. Voluntary changes consider the driver acceleration gain and changes to affected followers, with a politeness weight of 0.2, an incentive threshold of 0.25 m/s², and a 0.15 m/s² right-lane bias. A driver waits at least 5 s between lane changes.

Mandatory work-zone merges can bypass the incentive threshold but not the safety checks. Front and rear net gaps must be at least 1.5 m. The raw IDM accelerations of both the merging driver and the target-lane follower must be no lower than -3 m/s². Safety checks use unclipped accelerations so a braking cap cannot conceal an unsafe gap.

Vehicles approaching the closure are considered first, then from downstream to upstream. Lane occupancy updates after every accepted move, preventing several vehicles from claiming the same old gap. Only adjacent-lane changes are allowed.

The rule borrows from MOBIL but adds project-specific work-zone priorities, exclusion areas, and keep-right assumptions. It is not a calibrated implementation of the complete original MOBIL model. There is no continuous lateral dynamics, reaction delay, communication, or CAV control.

A simplified alternating-yield rule lets the foremost closing-lane vehicle request space from a target-lane follower. The requested courtesy braking is limited to 2 m/s² with a 4 m extra gap buffer; stronger braking required by the actual leader is retained. After yielding to one merging vehicle, that follower receives its own passage opportunity. This is an uncalibrated behavioral assumption, not part of the original MOBIL specification.

A lane change instantly changes `lane_id`. Smooth lateral movement in the viewer is a display effect and cannot support collision-risk analysis.

## Ramp entry, observations, and metrics

The ramp enters lane 2 at 250 m. Vehicles wait until front/rear gap and braking checks permit insertion. Mainline and ramp vehicles unable to enter remain in boundary queues and count toward delay. Boundary storage is unlimited in this freeway version; there is no upstream spillback model.

Detectors at 200, 550, and 900 m report crossing counts and crossing-mean speeds for each lane every 30 s. Mean speed remains missing if no vehicle crosses. Combined-lane mean speed is weighted by crossing counts.

- Conservation is checked every timestep: generated = exited + on-road + boundary waiting.
- Total system time integrates all on-road and boundary-waiting vehicles over time.
- Total delay subtracts actual traveled distance divided by each vehicle desired speed from system time. It includes unfinished trips and boundary waits, in veh·s.
- Throughput is the exit count divided by the analysis duration, converted to veh/h.
- The road queue is the count of vehicles below 5 m/s.
- Outputs include residual queues, lane changes, mandatory merges, minimum same-lane gap, strongest braking, and numerical guard interventions.

A numerical guard is recorded if a motion update would overlap a leader or cross a closed-lane boundary. No guards trigger in the default runs and verification scenarios. This does not establish improved real-world safety.

Watch a merge selects the next recorded work-zone merge and replays from 3 s before to 4 s after it at 1× speed. Yellow highlights the tracked vehicle; blue identifies yielding vehicles. Positions interpolate between snapshots; lateral interpolation does not affect physics.

Plots and replay use simulator truth. The human-driver model sees neighboring vehicles and the closure. Fixed ramp control uses only the clock for release eligibility; insertion safety is enforced by the simulator. Future estimation modules should consume `observations.csv`, not complete vehicle trajectories.

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
| `freeway/viewer_template.html` | Offline replay and plots |
| `results/freeway/` | Generated results; separate from source code |

The output contains `open/`, `closure/`, and `fixed/` run folders, plus `data.json`, `viewer.html`, and `verification.json` after checks. Each run folder contains traces, observations, control logs, lane changes, yield logs, and metrics.

Useful parameters include `merge_anticipation_m`, `closure_start_m`, `closure_end_m`, `lane_change_safe_braking_mps2`, `politeness`, and `demand`. The prototype supports a left-lane closure; changes to road layout or lane count also require corresponding viewer changes.

## Default freeway results and checks

Default demand was reduced to make merging easier to inspect. To run the earlier higher-demand configuration:

```bash
python3 run.py freeway --config freeway/configs/heavy_demand.json --output results/freeway_heavy
```

In the default 10-minute run, 417 vehicles exit with no closure, compared with 385 under the closure. The closure run records 114 mandatory merges, excluding voluntary merges before the mandatory-merge area. There are no closed-lane entries, same-lane overlaps, or guard interventions.

Independent checks cover IDM equilibrium, free motion, stopping, unsafe merge rejection, yielding and subsequent passage, lane-specific detector counts, overlaps, zero demand, and boundary overload. A finite arrival pulse generates 70 vehicles; all eventually pass the closure and exit, ruling out permanent trapping in that test.

Reducing the timestep from 0.05 to 0.025 s changes closure exits from 385 to 387, delay from 7902.25 to 7675.65 veh·s (about -2.87%), and mandatory merges from 114 to 121. Discrete lane-change timing remains sensitive. This is not a convergence proof or empirical validation; full results are in `results/freeway/verification.json`.

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

Codex assisted implementation, verification, English translation, and browser checks. Students must understand, verify, and extend the work and disclose their actual contributions. The original freeway still lacks state estimation, forecasting, and an adaptive TMC. The separate signal prototype implements these observation tasks and signal control within its own environment; see `intersection/README.md` for remaining course requirements.
