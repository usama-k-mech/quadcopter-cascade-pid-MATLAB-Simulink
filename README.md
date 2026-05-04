# quadcopter-cascade-pid-MATLAB-Simulink
Nonlinear 6-DOF simulation of a quadcopter tracking a figure-8 trajectory using a 3-loop cascade PID controller. Implemented in MATLAB/Simulink.

---

## How to Run

# Step 1: Load Parameters
run('quad_params.m')
This loads:
Vehicle physical parameters (mass, inertia, drag coefficients)
PID gains (inner, middle, outer loops)
Figure-8 trajectory (3-phase: takeoff → figure-8 → landing)
Simulation settings (sample times, saturation limits)

# Step 2: Run Simulink Model
open('Quad_Cascade_PID.slx')
Then click Run or press Ctrl+T
The Simulink model contains:
Outer loop (Position PID) - 50 Hz
Middle loop (Attitude PID) - 100 Hz
Inner loop (Rate PID) - 200 Hz
Control allocation (mixer) - Maps thrust/torques to rotor speeds
Quadcopter Plant - Custom MATLAB function block calling quad_plant_fcn.m

# Step 3: Visualize Results
run('animate_quad.m')
This generates:
3D trajectory animation of the quadcopter following the figure-8
Tracking error analysis

The quadcopter model, including its nonlinear 6-DOF dynamics and the complete cascade control structure (outer-, middle-, and inner-loop controllers), is implemented entirely using user-defined MATLAB functions rather than preconfigured Simulink blocks.
​
# Why Custom MATLAB Functions?
Full transparency: All equations are explicitly implemented, ensuring clarity and reproducibility
Educational value: Provides direct insight into the Newton–Euler formulation of rigid body dynamics
High customizability: Easily extendable to include aerodynamic effects, disturbances, or actuator dynamics
Debugging flexibility: Enables step-by-step debugging using MATLAB tools
Portability: The same function can be reused across simulations, scripts, or different projects
Toolbox independence: Avoids reliance on specialized toolboxes, improving accessibility
Research alignment: Consistent with common academic practice emphasizing model transparency
