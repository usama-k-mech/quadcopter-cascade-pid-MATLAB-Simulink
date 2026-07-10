# Quadcopter Cascade PID: MATLAB/Simulink

![MATLAB](https://img.shields.io/badge/MATLAB-R2021b-blue)
![Simulink](https://img.shields.io/badge/Simulink-required-orange)
![License](https://img.shields.io/badge/License-MIT-green)

**Nonlinear 6-DOF quadcopter simulation with a three-loop cascade PID (200 / 100 / 50 Hz) flying a takeoff, figure-8, landing mission, with the plant dynamics implemented as a 12-state Level-2 MATLAB S-Function and every controller built from user-defined function blocks. No preconfigured library blocks; every equation is visible and editable.**

This is the original implementation of the controller later ported to Python and **published as an official use-case example in the [c4dynamics framework documentation](https://c4dynamics.github.io/c4dynamics/programs/pid_cascade/quadcopter_pid.html)**: the same architecture validated on two independent toolchains.

Python port: [quadcopter-cascade-pid-python](https://github.com/usama-k-mech/quadcopter-cascade-pid-python)

![Simulation results](src/figures/simulation_results.png)

---

## Requirements

- MATLAB R2021b or later (the .slx will not open in older releases)
- Simulink
- No additional toolboxes required

## How to Run

From the `src/` folder, in the MATLAB Command Window:

```matlab
run('Quad_Params.m')            % 1. load parameters and generate the reference trajectory
open('Quad_Cascade_PID.slx')    % 2. open the model, then Run (Ctrl+T)
run('plot_results_quad.m')      % 3. results dashboard + RMSE metrics
```

Step 1 loads the vehicle parameters and PID gains, generates the full 3-phase reference trajectory as `timeseries` objects for the model's From Workspace blocks, and shows a trajectory preview so you can verify the mission before flying it. Step 3 produces a 6-panel results dashboard (3D trajectory, XY plane, horizontal and altitude tracking, position error, attitude angles), prints figure-8 phase RMSE metrics to the Command Window, and saves the dashboard to `figures/`. An optional real-time 3D animation is available via `run('animate_quad.m')`.

---

## Mission

Three phases over a 90 s flight, all reference signals generated analytically in `Quad_Params.m`:

| Phase | Time | Description |
|-------|------|-------------|
| Takeoff | 0 to 8 s | Smooth S-curve climb from ground to 3.0 m hover altitude (zero start and end velocity) |
| Figure-8 | 8 to 82 s | x = 4 sin(0.1 t), y = 2 sin(0.2 t): an 8 m x 4 m Lissajous figure-8, period about 63 s |
| Landing | 82 to 90 s | Smooth return to origin and descent to ground |

## Model Structure

| Block | Rate | Role |
|-------|------|------|
| Outer loop: position PID | 50 Hz | position error to desired roll/pitch and total thrust, with velocity feedforward (finite-differenced reference velocity rotated to the body frame) and anti-windup |
| Middle loop: attitude PID | 100 Hz | attitude error to body-rate references, filtered derivatives, anti-windup |
| Inner loop: rate PID | 200 Hz | rate error to torque commands, filtered derivative (N = 50) |
| Control allocation (mixer) | 200 Hz | thrust and torques to individual rotor speeds, with saturation at the physical rotor limit |
| Quadcopter plant (`quad_plant_fcn.m`) | continuous | full nonlinear Newton-Euler 12-state dynamics as a Level-2 MATLAB S-Function: gyroscopic rotor coupling, per-axis aerodynamic drag, ZYX Euler kinematics |

Physical parameters follow the widely used Luukkonen quadcopter model (*Modelling and Control of Quadcopter*, Aalto University, 2011): m = 0.468 kg, L = 0.225 m, kT = 2.98e-6 N/(rad/s)^2, torque-to-thrust ratio 0.0382.

![Simulink top level](src/figures/simulink_top_level.png)

### Why user-defined functions instead of library blocks

Implementing the plant as a hand-written S-Function and every controller as plain MATLAB code means the model contains no opaque pre-tuned blocks: the Newton-Euler equations, the PID law with anti-windup and derivative filtering, and the allocation inverse are all readable line by line. This is also what made the later port to Python on the c4dynamics API a direct translation rather than a re-derivation.

---

## Results

Figure-8 phase metrics (t = 8 to 82 s, 3.0 m altitude):

| Metric | Value |
|--------|-------|
| RMSE X | 0.364 m (9.1% of the 4 m amplitude) |
| RMSE Y | 0.358 m (17.9% of the 2 m amplitude) |
| RMSE Z | 0.004 m (0.14% of altitude) |
| Max altitude deviation | 5.10 cm |

The Python port of this controller records figure-8 RMSE of 0.20 m (X), 0.38 m (Y), and 0.002 m (Z) on its published mission at 1.5 m altitude. The main differences: the Simulink version computes feedforward velocity by finite-differencing the sampled reference, while the Python port uses the analytic derivative (which chiefly explains the X-axis gap), and the missions differ in hover altitude (3 m vs 1.5 m) and altitude-loop gains. Y barely differs between the two toolchains, consistent with the Y axis being limited by its doubled trajectory frequency in both implementations.

---

## Repository Structure

```
+-- src/Quad_Cascade_PID.slx   # Simulink model: cascade loops, mixer, plant
+-- src/Quad_Params.m          # parameters, PID gains, 3-phase trajectory generation, preview plots
+-- src/quad_plant_fcn.m       # nonlinear 12-state Newton-Euler plant (Level-2 S-Function)
+-- src/plot_results_quad.m    # 6-panel results dashboard + RMSE metrics (saved to figures/)
+-- src/animate_quad.m         # optional real-time 3D animation (run locally)
+-- src/figures/               # model screenshot and results dashboard
```

## Related Repositories

- **Python port (published)**: [quadcopter-cascade-pid-python](https://github.com/usama-k-mech/quadcopter-cascade-pid-python), the c4dynamics official use case
- **Closed-loop EKF GNC**: the same plant and controller flown on 12-state EKF estimates ([quadrotor-cascade-pid-ekf](https://github.com/usama-k-mech/quadrotor-cascade-pid-ekf))
- **Feedback linearization + qLPV-MPC**: model-based control reaching millimeter RMSE on the same trajectory class ([quadrotor-feedback-linearization-lpv-mpc](https://github.com/usama-k-mech/quadrotor-feedback-linearization-lpv-mpc))
- **Informed RRT\* + cascade PID**: this controller tracking planned collision-free paths ([quadrotor-rrt-pid](https://github.com/usama-k-mech/quadrotor-rrt-pid))

---

## License

MIT license; see [LICENSE](LICENSE).
