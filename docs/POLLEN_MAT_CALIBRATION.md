# Competition-mat calibration

This file explains the drive models. The measurements themselves, what has and
has not run on the robot, and the log for new trials are in
[POLLEN_ROBOT_BEHAVIOR.md](POLLEN_ROBOT_BEHAVIOR.md); add new data there first.

This branch preserves the tested baseline and adds a selectable **MAT_TRIAL**
model. The installed settings used for the three shoot-mode measurements were
not confirmed. MAT_TRIAL is therefore a candidate for controlled tests, not a
validated replacement. Neither profile measures position or corrects heading.

## Forward-only measurements (known power and time)

| Power | Seconds | Actual cm | cm / (power × seconds) |
| ---: | ---: | ---: | ---: |
| 0.25 | 1 | 32.6 | 130.40 |
| 0.25 | 2 | 68 | 136.00 |
| 0.25 | 3 | 105 | 140.00 |
| 0.50 | 1.5 | 113 | 150.67 |

A least-squares line through zero for the three 0.25-power trials gives
`distance_cm ≈ 138.1714 × power × seconds`. Predicted distances are 34.54,
69.09 and 103.63 cm; residuals are -1.94, -1.09 and +1.37 cm. The increasing
coefficient with duration is consistent with startup/transient effects, but
these four measurements do not establish a universal acceleration model.

The 0.50-power point gives 150.6667. A's display interpolates the coefficient
between powers 0.25 and 0.50; outside that range it holds the nearest coefficient.
Interpolation, extrapolation and the single half-power point need repeat tests.
The old 150 estimate is retained here only as the prior comparison, not the new
mat estimate. At 0.20 for 2 seconds the new display estimates 55.27 cm, with an
explicit extrapolation warning. **A still commands exactly its selected power
and seconds; its tested 0.20 / 2.00-second defaults are unchanged.** A has no
requested distance to recalibrate automatically. For a desired distance, begin
with `seconds = desired_cm / (power × coefficient)`, then measure again.

## Shoot-mode observations

| Requested path | Actual displacement | Actual distance | Actual angle |
| --- | --- | ---: | ---: |
| 100 cm forward | 107 cm forward | 107 cm | 0 degrees |
| 100 cm sideways | 80 cm sideways | 80 cm | 90 degrees |
| 75 cm at 45 degrees | 54 cm forward, 31 cm sideways | 62.27 cm | 29.86 degrees |

The 45-degree observation is not a 75-cm diagonal: the ideal components would
be 53.03 cm each. The forward component was close, while sideways travel was
short by 22.03 cm. More drive time alone preserves the wrong direction.

Do not combine these observations with the known-power forward trials as if
they were one confirmed experimental setup. The shoot tests' drive power,
installed code/coefficient, battery, strafe direction and load were not recorded.

## Selectable profiles in B and C

- **0 / TESTED_BASELINE (default):** exactly the existing 129 timing coefficient,
  direct sine/cosine vector, and unchanged B defaults (+25.87 degrees, 60.96 cm,
  power 0.20). This makes the previously tested behavior available after reset.
- **1 / MAT_TRIAL:** assumes those shoot tests used the unchanged baseline model
  (`seconds = requested_cm / (129 × power)`) at one comparable power. It fits the
  measured output-angle/distance-ratio knots `(0°, 1.07)`, `(29.859°, 0.830207)`
  and `(90°, 0.80)` to command angles `0°, 45°, 90°`, respectively. Piecewise
  linear interpolation of output angle and distance ratio is inverted to choose
  a command angle and time. Left and right are assumed symmetric because the
  measured strafe side was not supplied. That assumption requires testing.

MAT_TRIAL keeps the selected vector-power magnitude; it changes direction and
seconds, not motor mapping. At supported powers the mecanum mixer does not
saturate. The selected angle means the **desired physical travel direction** in
this trial model. Telemetry also shows the different motor-command angle.
The field geometry is independent of these empirical motor corrections.

Conditional examples at power 0.20:

| Intended travel | MAT_TRIAL command angle | Estimated seconds |
| --- | ---: | ---: |
| 100 cm forward | 0 degrees | 3.622 |
| 100 cm sideways | +/-90 degrees | 4.845 |
| 75 cm at +45 degrees | +56.329 degrees | 3.534 |

The last row is an interpolated prediction, not an observed result. The same
129 baseline would request 2.907 seconds for the last row. Changing coefficient
alone cannot fix its direction. Changing profile or angle revalidates the full
time budget, as does changing power, distance, delay or feed duration.

## Validation procedure

1. Record installed commit/build, profile, power, requested distance/angle,
   battery, load and surface. Photograph INIT settings.
2. With feed set to zero, repeat short pure-forward and left/right strafe trials
   at the same power. Include a repeat of the exact original settings if known.
3. Measure both displacement components at 30, 45 and 60 degrees; record heading
   drift. Repeat each at least three times. Do not fit a second correction onto
   results obtained with an unidentified first correction.
4. Run the selected shooting sequence, then repeat departure with actual load.
5. Validate the complete field route, full footprint and partner timing. A
   geometrically clear centerline is not sufficient, and a passing computer
   test does not establish physical travel accuracy.

See [field route analysis](POLLEN_FIELD_ROUTES.md) for nominal field geometry.
