# CEE281 Design Record (Intermediate Review 3): Freeway Common Foundation + Signal Intersection Prototype

Jiageng Liu · November 9, 2026 · Repository `CEE281_project` · Covers Design Project Guide pages 1–4 (common foundation) and page 6 (final pathway B)

## Summary

We built two environments on one IDM vehicle model:

- **Freeway**: 1 km, three lanes, left lane closed by a work zone from 500 to 850 m, on-ramp at 250 m. The ramp meter sees only 30 s boundary counts, estimates the number of vehicles on the road, forecasts entries and exits 30 s ahead, and sets a release interval.
- **Signalized intersection** (final pathway B prototype): four 300 m single-lane approaches, two phases. The signal sees only 5 s counts from two loops per approach (100 m upstream and at the stop line), estimates queues, forecasts arrivals, and decides when to switch.

Four main findings:

1. The lane-closure discharge is about 2,600–2,700 veh/h whether or not traffic upstream is queued (no capacity drop). Ramp metering therefore cannot raise throughput; it only moves the queue to the ramp: adaptive metering gives 13% less delay than fixed 12 s metering but 2.5% more than no metering.
2. On the freeway the forecast contributes about 1%; at the intersection it is essential — without it the pressure rule switches phases far too often and delay doubles.
3. Under a directional demand shift both actuated and adaptive signal control save about 20% delay against the Webster fixed-time plan; during detector faults they fall back to the fixed plan and recover afterwards.
4. **A case where the design needs revision**: under heavy demand adaptive control is 35% worse than fixed, because queues longer than the 100 m detector zone are invisible to the controller.

Reproduction (project root, Python 3, no third-party dependencies; plotting needs matplotlib):

```bash
python3 run.py freeway            # 5 ramp-control modes → results/freeway/
python3 run.py intersection       # 4 scenarios × 4 signal controllers → results/intersection/
python3 run.py check all          # independent checks for both → results/*/verification.json
/usr/bin/python3 -s tools/make_figures.py   # Figures 1–5 → results/figures/
open view.html                    # offline replay
```

## 1. Questions and predictions (written before running)

| Environment | Question | Prediction | Outcome |
|---|---|---|---|
| Freeway | Does the left-lane closure create a persistent queue? | Peak 3,450 veh/h is near two-lane capacity; yes | ✅ Queue from ≈300 s; 65 vehicles still on road at 600 s |
| Freeway | Does ramp metering cut total delay? | adaptive < fixed < unmetered | ⚠️ Adaptive beats fixed, but both are worse than unmetered |
| Freeway | How much does the forecast matter? | Without it, adaptive degrades to fixed | ❌ Only 1% difference |
| Freeway | Does driver heterogeneity worsen queuing? | Yes | ✅ Homogeneous drivers: −10% delay |
| Freeway | Stable under detector faults? | Falls back to fixed, slightly more delay | ✅ 135 s fallback, +3% delay |
| Intersection | Can responsive control beat fixed time under balanced demand? | Slightly | ⚠️ Adaptive −5%, actuated +27% (switches too often) |
| Intersection | Under a directional shift? | Clearly | ✅ Actuated −24%, adaptive −19% |
| Intersection | When detectors fail? | Falls back to fixed, slightly worse than normal | ✅ 238 s fallback; adaptive still 25% better than fixed |
| Intersection | Under heavy demand? | Everyone oversaturated, small differences | ❌ Adaptive +35% vs fixed — the detector zone saturates and the controller goes blind |

## 2. Model

### 2.1 Vehicles (shared by both environments): IDM

```
s* = s0 + max(0, v·T + v·(v − v_lead) / (2·√(a·b)))
dv/dt = a·[1 − (v/v0)^4 − (s*/s)^2]
```

Position is the front bumper; net gap s is leader rear to follower front; vehicle length 5 m. Acceleration bounded to [−6, a] m/s²; ballistic integration prevents negative speeds when stopping within a step. Each driver draws five truncated-Gaussian parameters on arrival (seed 281, out-of-range draws resampled) and keeps them:

| Parameter | Mean | SD | Bounds | Basis |
|---|---:|---:|---|---|
| v0 desired speed (m/s) | freeway 28 / intersection 14 | 2 / 1.5 | 22–34 / 10–18 | textbook value; 50 km/h urban street at the intersection |
| T time headway (s) | 1.5 | 0.2 | 0.9–2.1 | IDM paper 1.6 |
| a max acceleration (m/s²) | 1.2 | 0.2 | 0.6–1.8 | IDM paper 0.73, raised |
| b comfortable deceleration (m/s²) | 2.0 | 0.3 | 1.1–2.9 | IDM paper 1.67 |
| s0 minimum gap (m) | 2.0 | 0.3 | 1.1–2.9 | IDM paper 2 |

### 2.2 Freeway

- Mainline 1,000 m, 3 lanes. Left lane tapers from 500 m, fully closed 650–850 m; with a 3.6 m lane, 1.8 m vehicle and 0.2 m clearance it is impassable from 558 m.
- Lane changing: simplified MOBIL every 0.2 s; politeness 0.2, incentive threshold 0.25 m/s²; safety requires front/rear net gaps ≥ 1.5 m and unclipped IDM accelerations ≥ −3 m/s² for both vehicles. From 350 m before the closure, left-lane vehicles build an "urgency"; at ≥ 0.8 a merge is forced whenever safe, from 0.35 the follower may be asked to yield (≤ 2 m/s²). Geometry, urgency and courtesy are uncalibrated project assumptions.
- Ramp: 200 m single lane, desired speed 12 m/s, joins the right lane at 250 m; unlimited upstream storage.
- Detectors at 200 / 550 / 900 m report per-lane counts and mean speeds every 30 s; boundary counters report mainline arrivals/entries, ramp arrivals/entries, exits.
- Demand: 0–240 s 1,800 mainline + 300 ramp veh/h; 240–480 s 3,000 + 450; 480–600 s 1,800 + 300. dt 0.05 s, 600 s.

### 2.3 Signalized intersection

- **Geometry**: four 300 m single through lanes N/S/E/W (≈40 vehicles at jam), 20 m conflict box, 60 m exit leg. No turns, no pedestrians (scope stated; no pedestrian benefit claimed).
- **Stop line**: car following stays IDM; on red a driver brakes at the constant deceleration that stops s0 before the line, and the applied acceleration is min(IDM, stopping). Treating the stop line as an IDM obstacle produced −6 m/s² braking (IDM over-reacts to a suddenly appearing standing obstacle), so this is a declared modelling choice.
- **Yellow**: ITE dilemma-zone rule — stop if the required deceleration is ≤ 3.05 m/s², otherwise the vehicle is committed. Re-evaluated every step, so a committed vehicle slowed by its leader stops if it can.
- **Clearance intervals** derived from model speeds and checked in verification: yellow 3.5 s ≈ 1 + v85/(2·3.05); all-red 2.5 s ≥ (box + vehicle length)/v15 = 2.0 s.
- **Arrivals**: piecewise-constant demand per approach; exponential headways from a seeded stream, so all four controllers of a scenario share one arrival table (exported as a `demand_id` hash). Uniform headways were avoided because they lock onto the signal cycle.
- **Loops**: an advance loop 100 m upstream of the stop line and a stop-line loop per approach, reporting counts, mean speeds and cumulative counts every 5 s; missing and delayed reports supported. dt 0.1 s, 900 s.

## 3. Interfaces (the five formats of guide page 1)

| Type | File | Fields |
|---|---|---|
| Vehicle state | `traces.csv` | time_s, vehicle_id, origin/approach, lane_id, position_m, speed_mps, acceleration_mps2 |
| Observation | `observations.csv`, `boundary_observations.csv` | sensor_id, interval_start_s, interval_end_s, available_time_s, count, cumulative_count, mean_speed_mps (blank when unmeasurable), quality_flag |
| Estimate / forecast | `estimates.csv`, `forecasts.csv` | issue_time_s, target_time_s, segment_id, variable_name, estimated/predicted_value, units, quality_flag, actual_value, absolute_error |
| Control command | `control_log.csv` | issue_time_s, apply_time_s, actuator_id, action, value, units, bounds, fallback, reason, the estimates and forecasts used |
| Evaluation record | `metrics.json` | scenario_id, demand_id, seed, model_version, controller_version, delay (total / per approach), throughput, max/residual queues, boundary queues, service imbalance, estimate and forecast MAE, red-light runs, conflict-box occupancy |
| Phases | `phases.csv`, `crossings.csv` (intersection) | start/end/group/stage of every phase; time, phase and commitment of every stop-line crossing |

Seconds and metres internally. Small standalone examples: `freeway/test_drivers.py`, `test_ramp.py`, `test_tmc.py`; `intersection/test_signal.py` (12 tests).

## 4. Observation, estimation and forecasting

**Estimator (same idea in both)**: cumulative-count conservation. Freeway: vehicles on road = mainline entries + ramp entries − exits. Intersection: queue = cumulative advance-loop crossings − cumulative stop-line crossings (vehicles inside the detector zone; anything queued beyond 100 m is invisible). Missing reports hold the last value; cumulative counts self-correct on recovery; stale data (> 45 s freeway, > 12 s intersection) are flagged.

**Forecast**: next-30 s count, EMA (α = 0.35) against a moving-average baseline (3 intervals freeway, 6 intervals intersection). Both methods are scored at identical issue times and horizons from 120 s onward; α and window were fixed in advance.

| Metric | Freeway normal | Freeway faults | Intersection normal | Intersection faults |
|---|---:|---:|---:|---:|
| State estimate MAE (veh) | road 2.2 / ramp 0.8 | 4.3 | queue 0.40 | 0.55 |
| Arrival forecast MAE, MA / EMA (veh per 30 s) | mainline 2.50 / 3.15; ramp 0.71 / 0.67 | — | 2.2 / 2.2 | 2.4 / 2.4 |

In neither environment does EMA beat the moving average: demand changes in steps, both methods lag one interval, and EMA's weight on the latest value amplifies count noise. A kept negative result.

![Figure 3](../results/figures/fig3_estimate_vs_truth.png)
![Figure 4](../results/figures/fig4_forecast_mae.png)

## 5. Control

### 5.1 Freeway: basic TMC loop

Every step runs observe → estimate → predict → decide → act → evaluate and logs each stage. Rule (project-defined, not ALINEA):

```
N0 = max(0, N_est + forecast mainline entries − forecast exits)   # on-road count after 30 s with no ramp releases
r  = (45 − N0) / 30                                                # desired release rate, veh/s
interval = 1/r, bounded to [3, 20] s; faster if the forecast ramp queue exceeds 15
```

A permit must still pass the merge-gap and follower-braking checks. Missing, stale or start-up information falls back to fixed 12 s.

Decision traces: t = 320 s, estimated 41 on road, ramp queue 3, forecast 43.7 after 30 s → near the target of 45 → 20 s interval → hold. t = 427 s, estimated 55, ramp queue 13, forecast queue reaches 15 → queue relief, 18.5 s → one release.

### 5.2 Intersection: four signal controllers

One two-phase state machine: green → yellow 3.5 s → all-red 2.5 s → the other group's green. Minimum green 8 s is never cut short; yellow and all-red are never skipped (also in fallback); maximum green 60 s applies with an opposing call, otherwise the signal rests in green — as a real actuated controller does, so an approach with demand is never starved.

| Controller | Rule | Basis |
|---|---|---|
| `fixed` | Webster plan from the design demand (450 veh/h on all approaches), the **measured** saturation flow of 1,440 veh/h and 11 s lost time per phase: 44.7 s green, 101 s cycle; never retuned to a test scenario | Webster 1958; FHWA Signal Timing Manual ch. 5 |
| `actuated` | After minimum green, extend while the served approaches' advance loops counted a vehicle in the last 5 s report (5 s unit extension); otherwise gap out; max out at 60 s | FHWA manual ch. 5 |
| `adaptive` | Pressure = zone queue estimate + 30 s arrival forecast, summed per group; switch when the served group is empty and quiet, or when the opposing pressure exceeds the served pressure by more than 2 vehicles | project-defined |
| `adaptive_no_forecast` | Same rule with every forecast term set to zero | CEE281 ablation |

If any approach report is unavailable, missing or stale, responsive controllers run the fixed plan.

**Why the fixed cycle is long**: this IDM population starts slowly (a = 1.2 m/s²). Releasing a standing queue of 25 vehicles measures a 2.5 s saturation headway and a 7.2 s start-up lost time (real intersections ≈ 2 s). The configured saturation flow and lost time must stay within 10% / 1 s of the measurement or `check intersection` fails.

**Decision traces** (directional_shift / adaptive, N–S demand 700, E–W 300):
- t = 381 s: NS green reached 60 s with an opposing call → `max_out` → yellow. NS estimated queue 5, forecast 16.7 arrivals in 30 s; EW queue 7, forecast 1.3.
- t = 395 s: after 8 s of EW green, EW queue 5 + forecast 3.8 = 8.8 vs NS 5 + 8.1 = 13.1, difference > 2 → `opposing_pressure_larger` → back to NS. Over the shift NS received 511 s of green, EW 240 s.
- Fault run, t = 365.7 s: N loop missing (quality `missing/ok/ok/ok`) → `fallback_fixed`, still passing through 3.5 s yellow and all-red.

## 6. Experiment record (prediction → change → result → interpretation)

**Freeway**: identical arrivals and drivers, empty road, 600 s; delay in veh·min including residual queues and boundary waits.

| # | Change | Prediction | Result | Interpretation |
|---|---|---|---|---|
| E1 | No work zone → left-lane closure | queue at peak | delay 31 → 203; discharge at 900 m 3,450 → 2,670 veh/h | the two-lane bottleneck passes only 75% of the IDM equilibrium capacity of 3,544 |
| E2 | Closure + fixed 12 s | ramp queue buys a smoother mainline | 203 → 240; ramp residual 3 → 16 | ramp is 13% of demand; holding it does not shorten the mainline queue |
| E3 | Closure + adaptive | better than fixed | 208: −13% vs fixed, +2.5% vs unmetered | target of 45 is conservative; the meter sits at 20 s through the peak |
| E4 | Adaptive, forecasts zeroed | degrades to fixed | 211 (+1%) | entry and exit forecasts nearly cancel |
| E5 | Reports missing 300–360 s, delayed 45 s at 420–480 s | fallback, slightly worse | 135 s fallback; 215 (+3%); estimate MAE 2.2 → 4.3 | cumulative counts self-correct |
| E6 | Demand × 1.4 | queue does not clear | 695 / 717 / 681 (unmetered / fixed / adaptive) | at saturation metering cannot change discharge |
| E7 | Driver SD → 0 | less delay | 203 → 182 (−10%); lane changes in open mode 178 → 8 | speed heterogeneity drives discretionary lane changes |
| E8 | dt 0.05 → 0.025 | same conclusion | exits 372 = 372; delay 12,202 vs 12,197 veh·s | time step does not drive conclusions |
| E9 | Peak 2,400 + 900; ramp moved to 500 m; both | ramp becomes the bottleneck, metering helps | metering still not better than unmetered (table) | Section 7 |

E9 (delay veh·min; peak discharge at 900 m in veh/h in parentheses):

| Configuration | Unmetered | Fixed 12 s | Adaptive |
|---|---:|---:|---:|
| Ramp at 250 m, 2,400 + 900 | 164 (2,674) | 257 (2,417) | 165 (2,674) |
| Ramp at 500 m, default demand | 195 (2,743) | 233 (2,623) | 219 (2,606) |
| Ramp at 500 m, 2,400 + 900 | 173 (2,657) | 211 (2,589) | 173 (2,657) |

**Intersection**: one arrival table per scenario for all four controllers, 900 s; delay in veh·min including boundary waits and unfinished trips. Parentheses: change relative to fixed.

| # | Scenario | Prediction | fixed | actuated | adaptive | no_forecast | Interpretation |
|---|---|---|---:|---:|---:|---:|---|
| S1 | Balanced 450 × 4 | responsive slightly ahead | 260.5 | 330.2 (+27%) | 246.3 (−5%) | 551.2 (+112%) | the fixed plan was designed for this demand and is hard to beat; every switch costs 11 s, and actuation switches too often |
| S2 | Shift 700/300 (300–660 s) | responsive clearly ahead | 469.0 | 358.3 (−24%) | 381.8 (−19%) | 695.6 (+48%) | fixed N–S capacity 564 veh/h cannot serve 700; responsive control moves the green. Per-approach delay on S: fixed 18,002 veh·s, adaptive 6,643; imbalance ratio 3.8 → 2.7 |
| S3 | S2 + N missing 360–480 s, E delayed 20 s 540–660 s | fallback, slightly worse than S2 | 469.0 | 421.4 (−10%) | 351.6 (−25%) | 592.9 (+26%) | 238 s of fallback; adaptive outside the windows; estimate MAE 0.40 → 0.55 without accumulating |
| S4 | Heavy 800 × 4 for 600 s, then 300 | all oversaturated, small differences | 1,458.5 | 1,426.0 (−2%) | 1,969.5 (+35%) | 2,376.9 (+63%) | both detector zones saturate at ≈13 vehicles; the pressure comparison fails and adaptive switches on noise (28 greens vs 18). Boundary queues up to 53 vehicles are reported as unmet demand, not dropped |
| S5 | Ablation: forecast removed (all scenarios) | slightly worse | — | — | — | 41–42 greens | not about accuracy (MAE 2.2 for both methods): during green, vehicles queued beyond the advance loop start moving and cross it, so the served group's forecast rises and acts as a proxy for the hidden queue. Without it the rule switches as soon as the visible zone drains |

All 16 runs: red-light runs 0, both groups in the conflict box 0, all-red crossings 0.

![Figure 1](../results/figures/fig1_trajectories.png)
![Figure 2](../results/figures/fig2_delay_by_mode.png)

## 7. Why freeway metering is not better than no metering

1. **Bottleneck discharge does not depend on the upstream state.** With mainline demand only, discharge at the closure is 2,263 / 2,469 / 2,537 / 2,589 veh/h for 2,400 / 2,700 / 3,000 / 3,300 veh/h demand, saturating near 2,600. Free flow does not pass more vehicles, i.e. there is no capacity drop — and avoiding a capacity drop is where real ramp metering earns its benefit (Cassidy & Bertini 1999: 5–20%).
2. **Same departure curve, same total delay.** Cumulative crossings at 900 m nearly coincide for the three modes (375 / 370 / 375 at 600 s). By the cumulative-curve argument total delay is the area between arrival and departure curves; metering only relocates the queue.
3. **Ramp and bottleneck are on opposite sides.** The closed lane is the left one; the ramp enters the right lane, where speed at 550 m stays 18–23 m/s in every mode. Moving the ramp to 500 m (E9) does not change this.

Analytic comparison: with mean parameters the IDM equilibrium capacity is 1,772 veh/h per lane, 3,544 for two lanes; the simulated bottleneck discharges 2,670, 25% lower, because left-lane vehicles slow and wait to merge before 558 m.

![Figure 5](../results/figures/fig5_discharge_900m.png)

## 8. Verification (`python3 run.py check all`, all passing)

| Check | Freeway | Intersection |
|---|---|---|
| Conservation (every step: generated = exited + on road + waiting) | 5 modes | 16 runs |
| One/two-vehicle analytic comparison | equilibrium, free motion, stopping | same + measured saturation flow 1,440 veh/h and start-up lost time 7.2 s |
| Gaps and safety | overlaps 0, negative speeds 0, closed-lane intrusions 0 | overlaps 0, backward motion 0, **red-light runs 0, both groups in the conflict box 0** (recomputed from trajectories) |
| Detectors | trajectory recount = detector counts per lane | trajectory recount = loop counts per approach and loop |
| Signal constraints | — | yellow/all-red durations exact; every green ≥ minimum; greens beyond maximum only while resting or in fallback; fixed greens equal the Webster plan; ITE clearance formulas |
| Boundaries | zero demand; overload 440 vehicles → 226 waiting, none dropped | zero demand (actuated rests in green); overload 2,400 × 4 → 511 waiting; zone estimate saturates at 13 |
| Faults | fallback and recovery | fallback only inside the fault windows, none after 700 s |
| Finite pulse clears | all 70 vehicles exit | all 76 vehicles exit under every controller |
| Time step dt/2 | exits 372 = 372, delay within 0.1% | fixed: insensitive; **adaptive: delay 22,905 vs 17,145 veh·s** — thresholded decisions are sensitive to report timing; reported, not hidden |

## 9. Status of final pathway B (against guide page 6)

**Confirmed design**: detector-based actuated / queue-responsive control against a justified fixed-time baseline, asking whether it reduces total delay when demand changes. Geometry, phase plan, clearance, service constraints, detector interface, decision logging, fallback, the four tests, per-approach metrics, conflict checks, phase timeline and ablation — everything listed on page 6 exists in the prototype (details in `intersection/README.md`).

**Relation to the scaffold**: the guide says the intersection geometry and basic mechanics come from an instructor scaffold, which had not been released on this date. Its wording ("approach arrivals and queues, stop-line discharge") suggests a queue model rather than a car-following model. The prototype is built so that only `model.py` would be replaced: `observation.py` consumes reports only, `control.py` consumes estimates and forecasts only, evaluation and replay consume logs. If the scaffold is a queue model, its arrival and discharge events plug into `ApproachDetectors.cross()`; if it has turns or several lanes, the hard-coded NS/EW groups in `control.py` must become a movement-compatibility matrix.

**Case needing revision**: S4, heavy demand. The detector zone is only 100 m; once queues exceed it the pressure rule degenerates into noise-driven switching. Candidate fixes: a longer minimum green under saturation, or falling back to the fixed plan when both zones read as full. This is the first experiment for the final submission.

## 10. Limitations and next steps

**Limitations**

1. All parameters are teaching assumptions, not calibrated; conclusions describe causality inside the model.
2. Freeway boundary storage is unlimited with no spillback; the intersection's 7 s IDM start-up lost time is far above the real ≈ 2 s and lengthens the fixed cycle.
3. Lane changes are discrete jumps; yellow decisions have no reaction time. Neither supports crash-risk analysis.
4. The intersection uses one arrival seed; adaptive control is dt-sensitive, so single-seed conclusions are fragile.

**Next steps (by priority)**

1. Intersection: 2–3 arrival seeds; a zero-demand tail on every scenario so queues clear; a saturation-aware revision for S4.
2. Intersection: adapt `model.py` to the scaffold when it arrives, rerun S1–S5, and compare the car-following and queue-model conclusions (a CEE281 controller/model comparison).
3. Freeway: reaction time or an IDM variant at the closure to test for a capacity drop, then repeat E3; Newell's model as a second car-following model for E1.
4. Common: video observation with a manually labelled count audit; run this estimator on a peer's detector data (component exchange).

## 11. References

1. Treiber, M., Hennecke, A., Helbing, D. (2000). Congested traffic states in empirical observations and microscopic simulations. *Physical Review E*, 62, 1805–1824.
2. Kesting, A., Treiber, M., Helbing, D. (2007). General lane-changing model MOBIL for car-following models. *Transportation Research Record*, 1999, 86–94.
3. Treiber, M., Kesting, A. (2013). *Traffic Flow Dynamics*. Springer. Chapters 2–4, 10, 11.
4. FHWA (2006). *Ramp Management and Control Handbook*, ch. 5.
5. Cassidy, M. J., Bertini, R. L. (1999). Some traffic features at freeway bottlenecks. *Transportation Research Part B*, 33, 25–42.
6. Webster, F. V. (1958). *Traffic Signal Settings*. Road Research Technical Paper 39.
7. FHWA (2008). *Traffic Signal Timing Manual*, ch. 5 (fixed-time, actuated control, clearance intervals).
8. Daganzo, C. F. (1997). *Fundamentals of Transportation and Traffic Operations*, ch. 2 (cumulative curves).

## 12. Source-to-design table, AI use and individual contribution

| Design element | Source |
|---|---|
| IDM equations, parameters, ballistic integration | refs 1, 3; the intersection reuses `freeway/behavior.py` and `drivers.py` unchanged |
| Lane-change incentive and safety criteria | refs 2, 3 |
| Cumulative-count estimate, cumulative-curve delay analysis | refs 3, 5, 8 |
| Ramp control concepts | ref 4 |
| Webster fixed-time plan | refs 6, 7 |
| Yellow / all-red formulas, 3.05 m/s² stopping deceleration, actuated minimum green / unit extension / gap-out / max-out | ref 7 (ITE clearance formulas) |
| Work-zone geometry, urgency, courtesy; intersection geometry, loop positions, pressure rule, all thresholds and demand | project assumptions, uncalibrated |
| Simulation, detector, accounting and visualisation code | written for this project; no third-party code copied |

**AI use**: Codex and Claude assisted with code, verification scripts, plotting scripts and document editing; Claude assisted the rewrite of the intersection branch from a queue prototype to the car-following version. All AI-generated code was verified by running `python3 run.py check all`; all numbers come from reproducible runs in this repository.

**Individual contribution** (owners to be filled in after both partners confirm):

| Component | Code | Tests | Owner |
|---|---|---|---|
| Vehicle dynamics, lane changing, numerical verification | `freeway/behavior.py`, `drivers.py`, `verify.py` | equilibrium, stopping, time step, gap audit | |
| Arrivals, bottleneck, ramp, detectors | `freeway/model.py`, `ramp.py`, `observation.py` | conservation, detector audit, `test_ramp.py` | |
| Estimation, forecasting, ramp control | `freeway/observation.py`, `control.py` | `test_tmc.py` | |
| Freeway experiments and figures | `freeway/run.py`, `configs/`, `tools/make_figures.py` | E1–E9 | |
| Intersection mechanics and stop-line rule | `intersection/model.py` | saturation discharge, clearance formulas, trajectory audits | |
| Intersection observation and signal control | `intersection/observation.py`, `control.py` | `test_signal.py` | |
| Intersection experiments and replay | `intersection/run.py`, `viewer_template.html.in` | S1–S5 | |

Changes made in response to feedback: heavy-demand results rerun with the current model; E4, E7, E9 control experiments and static figures added; the intersection branch rewritten from a point-queue prototype to a car-following version that reuses the freeway IDM, with measured saturation flow, a Webster baseline, and red-light / conflict-box checks.
