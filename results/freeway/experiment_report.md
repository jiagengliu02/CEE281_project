# Five-stage ramp-control experiment

Same deterministic demand and per-vehicle driver parameters in all modes. Only open has no lane closure; compare closure/fixed/adaptive for control effects.

| Mode | Departures | Total delay (veh.min) | Residual main / ramp / road (veh) | Fallback (s) |
|---|---:|---:|---:|---:|
| open | 416 | 31.01 | 0 / 3 / 21 | 0.0 |
| closure | 372 | 203.37 | 0 / 3 / 65 | 0.0 |
| fixed | 367 | 239.74 | 0 / 16 / 57 | 0.0 |
| adaptive | 372 | 208.39 | 0 / 16 / 52 | 30.0 |
| adaptive_fault | 369 | 215.45 | 0 / 16 / 55 | 134.9 |

Adaptive delay minus fixed delay: -31.35 veh.min (positive means worse).

Forecasts actively change adaptive release intervals. Conservation estimates use measured cumulative boundary counters; the controller never sees true queues or future demand. Forecasts are evaluated on matched issue times and complete 30-second horizons, after the declared evaluation start.

The initial adaptive rule is not guaranteed to outperform fixed control. Inventory extrapolation, queue-relief overrides and merge timing can produce adverse feedback. A fault run performing better is not evidence that missing data are beneficial: fallback changes the timing policy.

Ramp vehicles use IDM along a finite ramp with unlimited upstream FIFO storage. The queue threshold is soft, all residual queues and their accrued waiting are included. No physical ramp-storage or spatial spillback claim is made.

Parameters were declared for a teaching prototype. This is neither empirical validation nor a safety-effectiveness claim. Preserve these results when developing and testing another controller.
