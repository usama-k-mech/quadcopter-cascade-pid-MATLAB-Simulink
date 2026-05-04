# quadcopter-cascade-pid-MATLAB-Simulink

Nonlinear 6-DOF simulation of a quadcopter tracking a figure-8 trajectory using a 3-loop cascade PID controller. Implemented in MATLAB/Simulink.

---

## 🚀 How to Run

### Step 1: Load Parameters

```matlab
run('quad_params.m')
```

This loads:

* Vehicle physical parameters (mass, inertia, drag coefficients)
* PID gains (inner, middle, outer loops)
* Figure-8 trajectory (3-phase: takeoff → figure-8 → landing)
* Simulation settings (sample times, saturation limits)

---

### Step 2: Run Simulink Model

```matlab
open('Quad_Cascade_PID.slx')
```

Then click **Run** or press **Ctrl+T**

The Simulink model contains:

* Outer loop (Position PID) — 50 Hz
* Middle loop (Attitude PID) — 100 Hz
* Inner loop (Rate PID) — 200 Hz
* Control allocation (mixer) — maps thrust/torques to rotor speeds
* Quadcopter plant — custom MATLAB function (`quad_plant_fcn.m`)

---

### Step 3: Visualize Results

```matlab
run('animate_quad.m')
```

This generates:

* 3D trajectory animation of the quadcopter following the figure-8
* Tracking error analysis

---

## 🧠 Implementation Details

The quadcopter model, including its nonlinear 6-DOF dynamics and the complete cascade control structure (outer-, middle-, and inner-loop controllers), is implemented entirely using user-defined MATLAB functions rather than preconfigured Simulink blocks.

---

## 🔧 Why Custom MATLAB Functions?

* **Full transparency**: All equations are explicitly implemented, ensuring clarity and reproducibility
* **Educational value**: Direct insight into Newton–Euler rigid body dynamics
* **High customizability**: Easy to add aerodynamic effects, disturbances, or actuator dynamics
* **Debugging flexibility**: Step-by-step debugging using MATLAB tools
* **Portability**: Reusable across simulations and projects
* **Toolbox independence**: No reliance on specialized toolboxes
* **Research alignment**: Consistent with academic best practices emphasizing transparency

---
