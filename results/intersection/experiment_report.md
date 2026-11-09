# Signal-control experiments

Fixed-time baseline: Webster plan on the declared design demand, displayed green NS 44.7 s / EW 44.7 s, cycle 101.4 s (saturation flow 1440 veh/h measured from the IDM discharge check). The plan is never retuned to a test scenario.

All controllers of a scenario use the same arrival table and driver parameters. Delay includes boundary waits and unfinished trips (veh.s). Positive saving = less delay than fixed.

## Balanced demand (`balanced`, demand bd0273491b)

| Controller | Exits | Delay (veh.min) | Saving vs fixed (veh.min) | Residual road / boundary | Max boundary | Greens | Fallback (s) | Estimate MAE (veh) | Red runs | Box conflicts |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 411 | 260.51 | 0.00 | 25 / 0 | 7 | 18 | 0.0 | 0.393 | 0 | 0 |
| actuated | 401 | 330.17 | -69.66 | 31 / 4 | 4 | 24 | 5.0 | 0.4 | 0 | 0 |
| adaptive | 403 | 246.27 | 14.23 | 33 / 0 | 6 | 22 | 5.0 | 0.378 | 0 | 0 |
| adaptive_no_forecast | 373 | 551.19 | -290.69 | 60 / 3 | 4 | 42 | 5.0 | 0.406 | 0 | 0 |

## Directional demand shift (`directional_shift`, demand f9e9566a88)

| Controller | Exits | Delay (veh.min) | Saving vs fixed (veh.min) | Residual road / boundary | Max boundary | Greens | Fallback (s) | Estimate MAE (veh) | Red runs | Box conflicts |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 413 | 468.95 | 0.00 | 51 / 6 | 14 | 18 | 0.0 | 0.367 | 0 | 0 |
| actuated | 431 | 358.31 | 110.64 | 39 / 0 | 6 | 24 | 5.0 | 0.399 | 0 | 0 |
| adaptive | 432 | 381.75 | 87.20 | 36 / 2 | 6 | 25 | 5.0 | 0.397 | 0 | 0 |
| adaptive_no_forecast | 391 | 695.57 | -226.63 | 72 / 7 | 19 | 41 | 5.0 | 0.404 | 0 | 0 |

## Demand shift with detector faults (`detector_fault`, demand f9e9566a88)

| Controller | Exits | Delay (veh.min) | Saving vs fixed (veh.min) | Residual road / boundary | Max boundary | Greens | Fallback (s) | Estimate MAE (veh) | Red runs | Box conflicts |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 413 | 468.95 | 0.00 | 51 / 6 | 14 | 18 | 0.0 | 0.43 | 0 | 0 |
| actuated | 414 | 421.43 | 47.52 | 54 / 2 | 9 | 23 | 237.9 | 0.465 | 0 | 0 |
| adaptive | 445 | 351.59 | 117.35 | 17 / 8 | 7 | 19 | 237.9 | 0.555 | 0 | 0 |
| adaptive_no_forecast | 405 | 592.90 | -123.96 | 64 / 1 | 20 | 35 | 237.9 | 0.629 | 0 | 0 |

## Heavy demand and finite storage (`heavy`, demand e6662d7e90)

| Controller | Exits | Delay (veh.min) | Saving vs fixed (veh.min) | Residual road / boundary | Max boundary | Greens | Fallback (s) | Estimate MAE (veh) | Red runs | Box conflicts |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 545 | 1458.51 | 0.00 | 66 / 26 | 52 | 18 | 0.0 | 0.444 | 0 | 0 |
| actuated | 531 | 1426.02 | 32.49 | 72 / 34 | 40 | 17 | 5.0 | 0.382 | 0 | 0 |
| adaptive | 484 | 1969.55 | -511.03 | 101 / 52 | 50 | 28 | 5.0 | 0.376 | 0 | 0 |
| adaptive_no_forecast | 415 | 2376.85 | -918.34 | 117 / 105 | 70 | 38 | 5.0 | 0.374 | 0 | 0 |

Notes

- `adaptive_no_forecast` is the CEE281 ablation: the identical pressure rule with every forecast term set to zero.
- The queue estimate counts vehicles between the advance loop and the stop line; queues longer than the detector zone are invisible to the controller, which matters in the heavy case.
- Fallback time counts seconds in which a responsive controller ran the fixed plan because a report was unavailable, missing or stale (startup included).
- Red runs must be zero; all-red crossings are committed vehicles still clearing the line during all-red. Neither is a crash measure.
- Synthetic, uncalibrated teaching model; one arrival seed. Results describe this model, not a real intersection.
