# Parameter

## Visible trajectory and motion parameters

| UI field | Configuration | Default / initial behaviour | Units and restrictions | Purpose |
|---|---|---|---|---|
| Preset Type | Path `kind` | Freehand on launch | Line, Circle, Spiral, Freehand | Selects generation or stroke processing. |
| Path Length | `path.length` | Calculated by preview | m; read-only | Processed path length. |
| Smoothing Samples | `smoothingWindow` | 11 | GUI 1–101; rounded to an integer; Freehand only | Starting smoothing strength, which can be strengthened within the shape budget. |
| Sampling Interval | `pathSpacing` | 0.03 | m; GUI 0.005–1 | Spatial path-sampling target; not controller sampling time. |
| Desired Trajectory Speed | `vr` | 0.5 | m/s; GUI 0.01–10 | Sets reference progress, duration and curvature-based reference angular rate. |
| Robot speed | `v0` | 0.5 | m/s; GUI 0.01–10 | Constant physical forward speed for every controller. |
| Sample Period | `Ts` | 0.1 | s; GUI 0.01–1 | Discrete controller and plant step. |
| Desired Duration | Calculated `path.length/vr` | Calculated | s; read-only | Run stop time; not an independently editable parameter. |
| Max Omega | `wMax`, with `wMin=-wMax` | 1.2 | rad/s; positive | Symmetric physical angular-velocity limit. Script configuration supports separate negative/positive limits. |
| Initial X / Y | `q0(1:2)` | Base `[-1;-1]`, then randomised on path confirmation | m; GUI ±1,000,000 | Robot initial position. |
| Initial Heading | `q0(3)` | Base 0.3 rad, then randomised on confirmation | GUI degrees, ±3600; stored radians | Robot initial orientation. |

## Visible controller parameters

| UI field | Configuration | Default / selection | Units and restrictions | Effect |
|---|---|---|---|---|
| Control Model | `controller` | `openloop` | GUI: `openloop`, `sf`, `lqr`, `mpc` | Chooses the controller. The scripted `external` option is separate from the four-item GUI. |
| Nominal Omega R | `design.wrNom`; optional `nominalOverride` | Automatically calculated after preview | UI deg/s, stored rad/s; GUI range ±100000, but zero is rejected by SF/LQR design | Fixed local SF/LQR design rate which does not replace the instantaneous reference rate. |
| SF Pole 1–3 | `design.poles` | `[-1 -2 -4]` | Negative inverse-time pole locations; GUI -10000 to -0.00001, distinct for SF | Desired continuous-time pole placement. |
| MPC Preview Steps | `N` | 20 | GUI 1–1000, rounded integer | Number of predicted control intervals. Prediction duration is `N*Ts`. |

The automatic nominal rate uses `vr*kappa` at equal-arc samples, excludes rates below the configured magnitude threshold, averages retained absolute magnitudes and uses the majority turning sign. A tie selects positive. The manual override is retained when speeds change but cleared by reprocessing the path. `design.wrNom=pi/4` in the base design is an initial design value; it is replaced by the path-derived or manually selected value for a normal SF/LQR GUI run.

## Noise dialog

| UI setting | Configuration | Default | Interpretation |
|---|---|---|---|
| Control Noise switch | `controlNoiseEnabled` | `true` | Enables independent angular-input disturbance sampling. |
| Control Omega Max | `controlStd` | 0.03 rad/s | Symmetric maximum, not sigma, despite the field name. Parent sigma is 0.01 rad/s. |
| Measurement Noise switch | `stateNoiseEnabled` | `true` | Enables the common tracking-error measurement disturbance. |
| X Error Max | `stateStd(1)` | 0.05 m | Longitudinal error-coordinate maximum |
| Y Error Max | `stateStd(2)` | 0.05 m | Lateral error-coordinate maximum |
| Orientation Max | `stateStd(3)` | 0.02 rad | Heading-error maximum |

Noise magnitudes must be nonnegative; native fields allow `[0, Inf]`. 

## Hidden parameters

These fields are in `SimulationConfig.default_config` unless another location is stated. Change defaults in code and relaunch for GUI use. Use copies of the full default structure for scripted experiments.

| Setting | Default | Purpose / important behaviour |
|---|---|---|
| `seed` | 42 | Measurement RNG seed. Control uses `seed+2`. The random-spawn stream uses `seed+10000` and advances with each pose request. |
| `maxPathDeviation` | 0.20 m | Maximum checked displacement of a smoothed stroke from its original arc-parameterised polyline. It is not a robot tracking tolerance. |
| `spawnFraction` | 0.10 | Uniform initial X/Y offset limits are ±10% of each processed path coordinate span. A zero-span axis borrows the other span. |
| `uiStride` | 3 | Requests a UI update every three controller samples; not a wall-clock frame-rate setting. |
| `endpointHold` | `true` | Reference preview holds the terminal pose after the endpoint. A preview interval crossing the endpoint uses the remaining-distance fraction. It does not add simulation time. |
| `endpointTolerance` | 0.25 m | Internal endpoint-arrival metric threshold; not a stop command or controller tolerance. |
| `trackingTolerance` | 0.3 m | Position-only tolerance used by internal `tracking_metrics` success/convergence calculations. It is not the separate 0.1 m / 0.1 rad analysis criterion discussed for report comparisons. |
| `successHoldTime` | 2 s | Internal tracking-success hold period. Internal convergence also requires the remaining recorded position errors to stay within tolerance. These criteria do not tune a controller. |
| `lookahead` | 0.8 m | Auxiliary target offset; also affects path-mode metric neighbourhood selection. It is not MPC's prediction horizon. |
| `referenceMode` | Base `'time'`; GUI forces `'time'` | Time-indexed progress uses `s=vr*t`. Scripted `'path'` mode uses local geometric projection and changes the evaluation interpretation. |

### Preset geometry

| Field | Default | Meaning |
|---|---|---|
| `preset.line.length` | 30 m | Line length along the positive X direction. |
| `preset.circle.radius` | 10 m | One complete circle. |
| `preset.spiral.initialRadius` | 8 m | Initial radius of the spiral. |
| `preset.spiral.growthRate` | 0.15 m/rad | Radial growth in `r=initialRadius+growthRate*angle`. |
| `preset.spiral.turns` | 5 | Number of complete spiral turns. |

Preset dimensions are code-only. Actual path length is calculated from the processed geometry and is displayed/exported.

### Feedback design and MPC

| Setting | Default and location | Purpose |
|---|---|---|
| SF/LQR `design.Q` | `eye(3)`; base in `ControlDesign.design_feedback_gains` | LQR state-error weights. Must be positive definite in this implementation. |
| SF/LQR `design.R` | 1; same base design | LQR correction-input penalty; positive scalar. |
| `nominalTurn.thresholdDeg` | 1 deg/s | Screening threshold for automatic nominal-rate selection. |
| `nominalTurn.fallbackDeg` | 1 deg/s | Positive nonzero nominal design rate if no screened turns remain. This preserves the local model's controllability requirement. |
| `nominalOverride` | `[]` | Empty uses automatic selection; a nonzero manual value is stored in radians/s. |
| `algorithm.mode` | `'nonlinear'` | Only the nonlinear MPC predictor is supported; the GUI also explicitly sets this mode. |
| `algorithm.Q` | Initially copied from `design.Q` | MPC state-error cost, positive semidefinite. Set separately if different MPC weights are wanted. |
| `algorithm.R` | Initially copied from `design.R` | MPC correction-input cost, positive scalar. |
| `algorithm.terminalWeight` | `zeros(3)` | Terminal-state cost; zero does not supply a terminal stability guarantee. |
| `algorithm.maxIterations` | 100 | SQP iteration limit. |
| `algorithm.tolerance` | `1e-6` | Passed to SQP constraint/optimality tolerances. Solver acceptance also checks bound residual against ten times this tolerance. |
| `algorithm.predictionSubsteps` | 4 | Number of RK4 substeps per predicted control interval. |

MPC uses SQP through `fmincon`, finite-difference gradients and a shifted previous input plan as its warm start when available.
SF and LQR gains remain fixed for each run. Their design uses the selected nominal reference rate and reference speed. The nonlinear plant and MPC prediction retain distinct robot and reference speeds. The simulated plant uses exact held-input unicycle integration; MPC's RK4 transition predicts error dynamics and is a separate calculation.

## What an export records

The evaluation CSV records speed, initial pose, sample period, angular limits, seed, noise switches/bounds/parent sigmas, reference mode and the applicable controller design or MPC settings. It also records status, duration and evaluation metrics. Curve CSVs retain actual motion, reference samples, errors and timing.
