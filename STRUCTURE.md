# Code Structure

The application separates interface behaviour, reference generation, control, simulation and evaluation. The main entry is `start_app.m`. Utility operations are grouped into static-method classes rather than a directory of forwarding functions.

## Directory map

```text
Group_project/
├── start_app.m
├── setup_paths.m
├── README.md
├── PARAMETERS.md
├── STRUCTURE.md
├── app/
│   ├── TrajectoryAppController.m
│   ├── AppUI.m
│   ├── attach_designer_controller.m
│   └── designer/
│       ├── TrajectoryDesignerApp.mlapp
│       ├── TrajectoryEditorApp.mlapp
│       └── TrajectoryNoiseApp.mlapp
├── trajectory/
│   └── TrajectoryPath.m
├── controller/
│   ├── ControlDesign.m
│   └── mpc/
│       ├── solve_nonlinear_mpc.m
│       ├── mpc_objective.m
│       ├── predict_tracking_error.m
│       └── tracking_mpc_constraints.m
├── simulation/
│   ├── SimulationConfig.m
│   ├── SimulationSession.m
│   ├── RobotDynamics.m
│   ├── run_simulation.m
│   ├── simulation_registry.m
│   ├── build_simulation_model.m
│   ├── sfun_robot_plant.m
│   ├── sfun_trajectory_controller.m
│   └── models/
│       └── trajectory_simulation.slx
├── evaluation/
│   └── ExperimentResults.m
├── docs/
│   └── images/                       Screenshots used by README
└── results/                          Exported experiments
```

This map describes the checked `Group_project` folder. Its MPC routines are currently four separate files. In the merged MPC version, the objective, predictor and constraints are local functions at the end of `solve_nonlinear_mpc.m`; the external entry and algorithm roles are unchanged.

## Module responsibilities

| File or module | Responsibility |
|---|---|
| `start_app.m` | Adds project paths and creates the main native app. Returns the shared application controller when an output is requested. |
| `setup_paths.m` | Adds the current app, simulation, trajectory, controller/MPC and evaluation directories. |
| `app/designer/*.mlapp` | Native, editable App Designer layouts and component-creation code. They are runtime dependencies. |
| `app/attach_designer_controller.m` | Creates editor/noise views, resolves the project root and connects all three views to one controller. |
| `app/AppUI.m` | Binds tagged components, installs callbacks, places independent speed controls and sets initial UI values/colours. It also assembles the saved layouts into the three-step workflow. |
| `app/TrajectoryAppController.m` | Coordinates path editing, confirmation, parameter changes, controller selection, noise, Run/Stop, result display and export. Its properties store shared state across callbacks. |
| `trajectory/TrajectoryPath.m` | Preset generation, freehand smoothing, geometric checks/summary, arc-length lookup, reference preview, nominal-turn selection and random initial-pose sampling. |
| `controller/ControlDesign.m` | SF/LQR design, per-run nominal preparation, feedback correction and the common measured-error/MPC adapter. |
| `controller/mpc/solve_nonlinear_mpc.m` | Constrained SQP optimisation, warm start, candidate acceptance and first-input execution. |
| MPC objective/predictor/constraints | Roll out nonlinear error states, calculate the cost and supply the nonlinear-constraint interface. Physical angular bounds are represented by optimisation bounds. |
| `simulation/SimulationConfig.m` | Complete defaults, configuration validation and explicit reference-speed access. |
| `simulation/SimulationSession.m` | Owns a run's reference/controller memory, independent RNGs, sampled control/noise caches, cancellation state and logs. |
| `simulation/RobotDynamics.m` | Exact held-input robot integration, tracking-coordinate error, angle wrapping and rejection-sampled truncated Gaussian noise. |
| `simulation/run_simulation.m` | Prepares a run, creates the session, sets Simulink parameters, executes the model and returns a result with metrics. Also has an explicitly selected MATLAB verification-loop engine. |
| `simulation/simulation_registry.m` | Gives S-functions numeric IDs for their session objects. |
| `simulation/build_simulation_model.m` | Returns the existing model, or creates the two-S-function harness when the SLX is missing. |
| `simulation/sfun_trajectory_controller.m` | Reads plant pose and invokes the session's control calculation at the explicit sample period. |
| `simulation/sfun_robot_plant.m` | Holds the true pose as discrete state and advances it using the applied input. |
| `evaluation/ExperimentResults.m` | True-motion metrics, export evaluation and six timestamp-matched CSV files. |

## Startup and interface flow

```text
start_app
  → setup_paths
  → TrajectoryDesignerApp
  → attach_designer_controller
      → TrajectoryEditorApp + TrajectoryNoiseApp
      → TrajectoryAppController
          → AppUI.bind_designer_views
          → trajectory setup → controller settings → run review
```

The `.mlapp` files create the components; the external `.m` files implement most operational behaviour. The editor and main layouts are switched between the visible window and a hidden host. Do not remove layout-host or component-handle properties merely because they are not numerical model parameters. The binding code removes unused saved Step 1 components because the native editor supplies that stage.

## Path and reference flow

```text
Preset / freehand stroke / imported XY
  → TrajectoryPath.generate_trajectory
  → preset resampling or freehand smoothing
  → path {xy, s, theta, kappa, length, ...}
  → TrajectoryPath.sample_reference_preview
  → reference {q, w, vr, s, ...}
```

Freehand smoothing uses uniform input resampling, a second-difference penalty with fixed endpoints, cubic splines and a shape-deviation check. The prepared path is sampled by arc length. In GUI time mode the reference advances at `vr*t`, and angular reference is `vr*kappa`. The preview contains N+1 poses and N reference angular rates/speeds, with endpoint-hold handling. GUI confirmation does not guarantee that all turns satisfy physical limits.

## Simulation and feedback flow

```text
TrajectoryAppController.runTest
  → run_simulation(path, configuration, callback, 'simulink')
  → ControlDesign.prepare_controller_design
  → SimulationSession + numeric registry ID
  → Simulink model
      robot pose → controller S-function → session.control
          → reference preview and true tracking error
          → common measured error = true error + measurement noise
          → Open Loop / SF / LQR / MPC
          → command limit → control noise → physical limit
      controller output → plant S-function → session.advance
          → exact RobotDynamics integration of the true pose
  → result and ExperimentResults metrics
```

The actuator disturbance is sampled once per control interval and cached so that plant advancement uses the same realisation as the logged physical rate. Repeated controller requests at the same sample reuse the cached command rather than resampling noise. The controller algorithms do not receive the configured initial pose through their algorithm configuration. The session supplies their current measured feedback and reference.

MPC predicts tracking-error states with RK4, not the plant's exact integration function. The optimiser receives the measured current error and future reference, but not future random-noise samples. Predicted global poses may be constructed for diagnostics; the displayed robot trajectory comes from `qTrue`.
