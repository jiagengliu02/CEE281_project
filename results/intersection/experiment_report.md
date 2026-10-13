# Signal experiments

Seeds: [11, 22, 33]. Paired identical arrival schedules; positive saving means lower delay than fixed.

| Scenario | Controller | Mean delay (veh.s) | Mean exits | Mean external residual | Paired saving (veh.s) |
|---|---|---:|---:|---:|---:|
| balanced | fixed | 7444.8 | 440.3 | 0.0 | 0.0 |
| balanced | adaptive | 5301.6 | 440.7 | 0.0 | 2143.2 |
| balanced | adaptive_no_hysteresis | 6662.9 | 441.3 | 0.0 | 781.8 |
| directional_shift | fixed | 30295.2 | 462.0 | 17.0 | 0.0 |
| directional_shift | adaptive | 7417.6 | 533.0 | 0.0 | 22877.7 |
| directional_shift | adaptive_no_hysteresis | 8394.7 | 531.3 | 0.0 | 21900.5 |
| detector_fault | fixed | 30295.2 | 462.0 | 17.0 | 0.0 |
| detector_fault | adaptive | 12773.9 | 528.3 | 0.0 | 17521.3 |
| detector_fault | adaptive_no_hysteresis | 13413.1 | 528.0 | 0.0 | 16882.2 |
| heavy | fixed | 175215.2 | 729.0 | 245.3 | 0.0 |
| heavy | adaptive | 176615.0 | 759.3 | 215.0 | -1399.8 |
| heavy | adaptive_no_hysteresis | 186444.0 | 746.7 | 227.7 | -11228.8 |

The no-hysteresis ablation sets only the switching queue difference threshold to zero.
These are synthetic, uncalibrated point-queue experiments, not evidence of crash reduction.
Delay includes accrued waits of unfinished vehicles and external queues. No clearance tail is used.
Forecast MAE compares identical issue times/horizons within each run; forecasts are advisory.
Three seeds provide an exploratory sensitivity check, not a statistical significance claim.
