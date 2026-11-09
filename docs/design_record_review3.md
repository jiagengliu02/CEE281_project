# CEE281 Design Record (Intermediate Review 3): Freeway Simulation, Observation and Forecasting, and Basic Ramp Control

Jiageng Liu · November 9, 2026 · Repository: `CEE281_project`

## Summary

This project implements a microscopic simulation of a 1 km, three-lane freeway. The left lane is closed by a work zone from 500 to 850 m, and an on-ramp joins at 250 m. The ramp meter sees only 30-second counts from boundary detectors; from these it estimates the number of vehicles on the road, forecasts entries and exits over the next 30 s, and sets a release interval.

Main findings:
1. The discharge capacity of the lane closure is about 2,600–2,700 veh/h regardless of whether traffic upstream is queued; there is no capacity drop.
2. Ramp metering therefore cannot raise throughput; it only moves the queue from the mainline to the ramp. Adaptive metering yields 13% less delay than fixed 12 s metering but 2.5% more than no metering. The conclusion is unchanged under different demand and a different ramp location.
3. Forecasts contribute about 1% to the control result; driver heterogeneity increases delay by about 10%.

Reproduction (from the project root; Python 3; the simulation has no third-party dependencies; plotting needs matplotlib):

```bash
python3 run.py freeway                       # 5 control modes → results/freeway/
python3 run.py check freeway                 # independent checks → results/freeway/verification.json
python3 run.py freeway --config freeway/configs/<name>.json --output results/<dir>   # other configurations
/usr/bin/python3 -s tools/make_figures.py    # Figures 1–5 → results/figures/
```

## 1. Questions and predictions

| Question | Prediction before running | Outcome |
|---|---|---|
| Does the left-lane closure create a persistent queue? | Peak demand of 3,450 veh/h is close to two-lane capacity; a queue forms and clears after demand falls | Confirmed. Queue starts near 300 s; 65 vehicles still on the road at 600 s |
| Does ramp metering reduce total delay? | Adaptive beats fixed, fixed beats no metering | Partly. Adaptive beats fixed, but both are worse than no metering |
| How much does the forecast matter? | Without it, adaptive degrades to fixed control | Not confirmed. Only a 1% difference |
| Does driver heterogeneity worsen queuing? | Yes | Confirmed. Homogeneous drivers give 10% less delay |
| Does the system stay stable under detector faults? | Falls back to fixed control, delay rises slightly | Confirmed. 135 s of fallback, +3% delay, estimate error does not diverge |

## 2. Model

### 2.1 Longitudinal motion: IDM

```
s* = s0 + max(0, v·T + v·(v − v_lead) / (2·√(a·b)))
dv/dt = a·[1 − (v/v0)^4 − (s*/s)^2]
```

Position is the front bumper; net gap s is from the leader's rear to the follower's front; vehicle length is 5 m. Acceleration is bounded to [−6, a] m/s²; ballistic integration prevents negative speeds when stopping within a step. Each driver draws five truncated-Gaussian parameters on arrival (seed 281, out-of-range draws resampled) and keeps them for the whole trip:

| Parameter | Mean | SD | Bounds | Basis |
|---|---:|---:|---|---|
| v0 desired speed (m/s) | 28 | 2 | 22–34 | textbook value |
| T time headway (s) | 1.5 | 0.2 | 0.9–2.1 | IDM paper 1.6 |
| a max acceleration (m/s²) | 1.2 | 0.2 | 0.6–1.8 | IDM paper 0.73, raised |
| b comfortable deceleration (m/s²) | 2.0 | 0.3 | 1.1–2.9 | IDM paper 1.67 |
| s0 minimum gap (m) | 2.0 | 0.3 | 1.1–2.9 | IDM paper 2 |

### 2.2 Lane changing: simplified MOBIL

Evaluated every 0.2 s. Politeness 0.2, incentive threshold 0.25 m/s², keep-right bias 0.15 m/s², at least 5 s between changes. Safety: front and rear net gaps ≥ 1.5 m, and unclipped IDM accelerations of the changing vehicle and the new follower ≥ −3 m/s². From 350 m before the closure, left-lane vehicles build an "urgency" that grows linearly with distance; at urgency ≥ 0.8 a merge is forced whenever it is safe, and from 0.35 the vehicle may ask the target-lane follower to yield (at most 2 m/s²). The taper geometry, urgency and courtesy rules are project assumptions and are not calibrated.

### 2.3 Road, ramp, detectors and demand

- Mainline 1,000 m, 3 lanes. The left lane tapers from 500 m and is fully closed from 650 to 850 m; with a 3.6 m lane, 1.8 m vehicle width and 0.2 m clearance it becomes impassable at 558 m.
- Ramp: 200 m single lane, desired speed 12 m/s, joins the right lane at 250 m; unlimited FIFO storage upstream.
- Detectors at 200, 550 and 900 m report per-lane counts and mean speeds every 30 s; boundary counters report mainline arrivals/entries, ramp arrivals/entries and exits.
- Demand (all lanes + ramp): 0–240 s 1,800 + 300 veh/h; 240–480 s 3,000 + 450; 480–600 s 1,800 + 300.
- Time step 0.05 s, duration 600 s.

## 3. Interfaces and outputs

| Type | File | Fields |
|---|---|---|
| Vehicle state | `traces.csv` | time_s, vehicle_id, origin, lane_id, position_m, speed_mps, acceleration_mps2 |
| Observation | `observations.csv`, `boundary_observations.csv` | sensor_id, interval_start_s, interval_end_s, count, mean_speed_mps (blank when unmeasurable), quality_flag |
| Estimate / forecast | `estimates.csv`, `forecasts.csv` | issue_time_s, target_time_s, segment_id, variable_name, predicted_value, units, quality_flag, actual_value, absolute_error |
| Control command | `control_log.csv` | issue_time_s, apply_time_s, actuator_id, action, value, units, bounds, fallback, reason, executed action |
| Evaluation record | `metrics.json` | mode, model_version, controller_version, delay, throughput, residual queues (mainline boundary / ramp / road), estimate and forecast MAE |

Seconds and metres are used internally. Each module has a small standalone example: `freeway/test_drivers.py`, `test_ramp.py`, `test_tmc.py`.

## 4. Verification

`python3 run.py check freeway`; all checks pass:

| Check | Method | Result |
|---|---|---|
| Conservation | every step: generated = exited + on road + waiting at boundary | passes in all 5 modes |
| Equilibrium, free motion, stopping | one- and two-vehicle analytic comparison | pass |
| Gaps | same-lane overlap, negative speed, closed-lane intrusion | 0 occurrences |
| Boundaries | zero demand; overload (440 vehicles into 1 km) | unmet demand recorded as 226 waiting vehicles, none dropped |
| Detectors | independent recount from trajectories vs detector counts | match per lane |
| Time step | dt 0.05 vs 0.025 (closure mode) | exits 372 = 372; delay 12,202 vs 12,197 veh·s; minimum gap 1.70 vs 1.64 m |
| Finite pulse | 70 vehicles must all exit | pass |

## 5. Experiment record

All experiments use identical arrival times, identical driver parameters, an initially empty road and 600 s. Delay includes residual queues and boundary waiting; units are veh·min. Five control modes: no work zone (open), closure without metering (closure), closure with fixed 12 s metering (fixed), closure with adaptive metering (adaptive), adaptive with detector faults (adaptive_fault).

| # | Change | Prediction | Result | Interpretation |
|---|---|---|---|---|
| E1 | No work zone → left-lane closure (unmetered) | queue at peak | delay 31 → 203; discharge at 900 m 3,450 → 2,670 veh/h | two-lane bottleneck discharges only 75% of the IDM equilibrium capacity of 3,544 |
| E2 | Closure + fixed 12 s | ramp queue in exchange for a smoother mainline | delay 203 → 240; ramp residual 3 → 16 veh | ramp is 13% of demand; holding it does not shorten the mainline queue |
| E3 | Closure + adaptive | better than fixed | delay 208: −13% vs fixed, +2.5% vs unmetered | target of 45 vehicles is conservative; the meter sits at the 20 s maximum through most of the peak |
| E4 | Adaptive with forecasts set to zero | degrades to fixed | delay 211 (+1%) | entry and exit forecasts are similar and nearly cancel |
| E5 | Reports missing 300–360 s, delayed 45 s at 420–480 s | fallback, slightly more delay | 135 s fallback; delay 215 (+3%); estimate MAE 2.2 → 4.3 veh | cumulative counts self-correct after recovery |
| E6 | Demand × 1.4 | queue does not clear | delay 695 / 717 / 681 (unmetered / fixed / adaptive) | adaptive is best but by only 2%; at saturation metering cannot change discharge |
| E7 | Driver parameter SD → 0 | less delay | closure delay 203 → 182 (−10%); lane changes in open mode 178 → 8 | speed heterogeneity is the main source of discretionary lane changes and disturbances |
| E8 | dt 0.05 → 0.025 | same conclusion | see Section 4 | time step does not drive the conclusions |
| E9 | Peak demand 2,400 mainline + 900 ramp; ramp moved from 250 to 500 m; both combined | ramp becomes the bottleneck, metering helps | see table below; metering still not better than unmetered | see Section 8 |

E9 details (delay in veh·min; peak discharge at 900 m in veh/h in parentheses):

| Configuration | Unmetered | Fixed 12 s | Adaptive |
|---|---:|---:|---:|
| Ramp at 250 m, 2,400 + 900 veh/h | 164 (2,674) | 257 (2,417) | 165 (2,674) |
| Ramp at 500 m, default demand | 195 (2,743) | 233 (2,623) | 219 (2,606) |
| Ramp at 500 m, 2,400 + 900 veh/h | 173 (2,657) | 211 (2,589) | 173 (2,657) |

![Figure 1](../results/figures/fig1_trajectories.png)
![Figure 2](../results/figures/fig2_delay_by_mode.png)

## 6. Observation, estimation and forecasting

The controller receives only cumulative boundary counts every 30 s; it sees no speeds and no vehicles on the road.

- Estimate: vehicles on road = mainline entries + ramp entries − exits; ramp queue = ramp arrivals − ramp entries. Held constant between reports.
- Forecast: next-30 s count for each channel. EMA (α = 0.35) against a 3-interval moving-average baseline. Evaluation from 120 s onward at identical issue times and horizons; error in veh per 30 s. α and window length were fixed in advance.
- Fault test: one window of missing reports and one of 45 s delays (E5).

| Metric | Normal | Faults |
|---|---:|---:|
| On-road count estimate MAE (veh) | 2.2 | 4.3 |
| Ramp queue estimate MAE (veh) | 0.8 | — |
| Mainline arrivals forecast MAE, MA / EMA | 2.50 / 3.15 | |
| Ramp arrivals forecast MAE, MA / EMA | 0.71 / 0.67 | |
| Exits forecast MAE, MA / EMA | 1.71 / 2.50 | |

EMA does not beat the moving average. Demand steps at 240 s and 480 s; both methods lag by one interval, and EMA's heavier weight on the latest value amplifies the random variation in 30 s counts.

![Figure 3](../results/figures/fig3_estimate_vs_truth.png)
![Figure 4](../results/figures/fig4_forecast_mae.png)

## 7. Basic TMC loop

Every simulation step runs observe → estimate → predict → decide → act → evaluate and logs each stage in `control_log.csv`.

Rule (project-defined):

```
N0 = max(0, N_est + forecast mainline entries − forecast exits)   # on-road count after 30 s with no ramp releases
r  = (45 − N0) / 30                                                # desired release rate, veh/s
interval = 1/r, bounded to [3, 20] s; release faster if forecast ramp queue > 15 veh
```

A permit still has to pass the physical merge-gap and follower-braking checks. Missing, stale (> 45 s) or start-up information falls back to fixed 12 s. Forecasts directly affect the decision.

Decision trace examples (adaptive):
- t = 320 s: estimated 41 on road, ramp queue 3; forecast 43.7 on road after 30 s without releases, near the target of 45 → interval 20 s → hold.
- t = 427 s: estimated 55 on road, ramp queue 13; forecast queue reaches 15 → queue relief, interval 18.5 s → one vehicle released.

## 8. Why metering is not better than no metering

Three pieces of evidence:

1. **Bottleneck discharge does not depend on upstream state.** With mainline demand only and no ramp, discharge at the lane closure is 2,263 / 2,469 / 2,537 / 2,589 veh/h for demand of 2,400 / 2,700 / 3,000 / 3,300 veh/h, saturating around 2,600. Free-flowing traffic does not pass more vehicles than queued traffic, i.e. there is no capacity drop. The benefit of ramp metering in real networks comes from avoiding a capacity drop (Cassidy & Bertini 1999 report 5–20%); none exists here.
2. **Same departure curve, same total delay.** Cumulative crossings at 900 m almost coincide for unmetered / fixed / adaptive (375 / 370 / 375 vehicles at 600 s). By the cumulative-curve argument, total delay is the area between arrival and departure curves; metering only relocates the queue to the ramp, and adds waiting when the ramp queue is still released at ≥ 12 s after demand has dropped.
3. **The ramp and the bottleneck are on opposite sides of the road.** The closed lane is the left lane, whose vehicles merge into the centre lane; the ramp enters the right lane. Right-lane speed at 550 m stays at 18–23 m/s in every mode, so the ramp merge never congests. Moving the ramp to 500 m (E9) does not change this; metering then wastes usable right-lane gaps and discharge falls from 2,743 to 2,606 veh/h.

Analytic comparison: with mean driver parameters the IDM equilibrium capacity is 1,772 veh/h per lane (at v ≈ 16 m/s), i.e. 3,544 veh/h for two lanes. The simulated bottleneck discharges about 2,670 veh/h, 25% lower. The gap comes from left-lane vehicles decelerating and waiting to merge before 558 m.

![Figure 5](../results/figures/fig5_discharge_900m.png)

## 9. Transition to final pathway B (signal control)

The guide specifies an instructor-supplied isolated-intersection scaffold (geometry and basic intersection mechanics) for the signal pathway. As of this record the scaffold has not been released, so `intersection/` defines its own geometry and mechanics and keeps them together in `intersection/model.py`; once the scaffold arrives only the arrival, admission and stop-line discharge parts need to be swapped, while the observation, control and evaluation interfaces stay unchanged.

Current prototype: four 300 m single-lane approaches; vehicles reuse the freeway IDM and driver parameters unchanged (desired speed lowered to 14 m/s); two through phases (N–S, E–W) with 3.5 s yellow, 2.5 s all-red, 8 s minimum and 60 s maximum green; an advance loop 100 m upstream and a stop-line loop on each approach, reporting every 5 s. Four controllers: Webster fixed-time (computed once from the 450 veh/h design demand, never retuned to a test scenario), actuated (gap extension), adaptive (pressure rule from the queue estimate plus a 30 s arrival forecast), and adaptive without forecasts (CEE281 ablation). Four scenarios: balanced, directional shift, directional shift with detector faults, heavy demand.

| Scenario | Actuated vs fixed | Adaptive vs fixed | No-forecast vs fixed |
|---|---:|---:|---:|
| Balanced | +27% | −5% | +112% |
| Directional shift | −24% | −19% | +48% |
| Shift + detector faults | −10% | −25% | +26% |
| Heavy demand | −2% | +35% | +63% |

(Change in total delay relative to the fixed-time plan; negative is better.) Red-light runs and simultaneous conflict-box occupancy are zero in every run. Unlike the freeway, the forecast matters a lot here: without it the pressure rule switches phases far too often and lost time grows. Adaptive control is also worse than fixed under heavy demand because queues beyond the detector zone are invisible to the controller; this is the main issue to address for the final submission. See `intersection/README.md` and `results/intersection/experiment_report.md`.

## 10. Limitations and next steps

Limitations:
1. All parameters are teaching assumptions, not calibrated against data; conclusions describe causality inside the model only.
2. Boundary storage is unlimited and there is no upstream spillback; the 15-vehicle ramp threshold is not a physical capacity.
3. Lane changes are discrete jumps and cannot support collision-risk analysis.
4. Standard IDM parameters produce no capacity drop, which limits what ramp control can show in this model.

Next steps:
1. Introduce reaction time or an IDM variant at the lane closure, test whether a capacity drop appears, and repeat E3.
2. Second car-following model: repeat E1 with Newell's model in the same framework.
3. Lead-vehicle braking experiment: inject one −3 m/s² braking event in open mode and observe whether the disturbance grows or decays as a function of T.
4. Video observation with a manually labelled count audit; run this estimator on a peer's detector data.

## 11. References

1. Treiber, M., Hennecke, A., Helbing, D. (2000). Congested traffic states in empirical observations and microscopic simulations. *Physical Review E*, 62, 1805–1824.
2. Kesting, A., Treiber, M., Helbing, D. (2007). General lane-changing model MOBIL for car-following models. *Transportation Research Record*, 1999, 86–94.
3. Treiber, M., Kesting, A. (2013). *Traffic Flow Dynamics*. Springer. Chapters 2–4, 10, 11.
4. Federal Highway Administration (2006). *Ramp Management and Control Handbook*, Chapter 5.
5. Cassidy, M. J., Bertini, R. L. (1999). Some traffic features at freeway bottlenecks. *Transportation Research Part B*, 33, 25–42.

## 12. Source-to-design table, AI use and individual contribution

Source → design:

| Design element | Source |
|---|---|
| IDM equations, parameter meanings, ballistic integration | refs 1, 3 |
| Lane-change incentive and safety criteria | refs 2, 3 |
| Cumulative-count conservation estimate, bottleneck discharge analysis | refs 3, 5 |
| Ramp control concepts | ref 4 |
| Work-zone geometry, urgency, courtesy, demand, all thresholds | project assumptions, uncalibrated |
| Simulation, detector, accounting and visualisation code | written for this project; no third-party code copied |

AI use: Codex and Claude assisted with code implementation, verification scripts, plotting scripts and document editing. All AI-generated code was verified by running `run.py check freeway`; all numbers come from reproducible runs in this repository.

Individual contribution:

| Component | Code | Tests |
|---|---|---|
| Vehicle dynamics, lane changing, numerical verification | `freeway/behavior.py`, `drivers.py`, `verify.py` | equilibrium, stopping, time step, gap audit |
| Arrivals, bottleneck, ramp, detectors | `freeway/model.py`, `ramp.py`, `observation.py` | conservation, detector count audit, `test_ramp.py` |
| Estimation, forecasting, ramp control | `freeway/observation.py`, `control.py` | `test_tmc.py`, five-stage loop checks |
| Experiments and figures | `freeway/run.py`, `freeway/configs/`, `tools/make_figures.py` | E1–E9 |
| Signalised intersection prototype | `intersection/` | `intersection/verify.py` |
