# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

### The Model
Student describes their model in detail. This includes the state, actuators and update equations.

The model uses the follow state variables to track the movement of a vehicle
- x - The position of vehicle on x axis. (Vehicle heading pointing forward)
- y - The position of vehicle on y axis. (Perdendicular to vehicle heading)
- psi - Vehicle heading
- speed - Vehicle speed
- cte - Cumulative total error of position
- epsi - Heading error

The model uses the follow actuator variables to stimulate the movement of a vehicle
- delta - Steering wheel angle
- a - Throttle

The model updates vehicle state with the following equations [1]
```python
x[t + 1] = x[t] + v[t] * cos(psi[t]) * dt
y[t + 1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t + 1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t + 1] = v[t] + a[t] * dt
cte[t + 1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t + 1] = psi[t] - psi_dest[t] + v[t] / Lf * delta[t] * dt
```

The C++ implementation is implied in the constraint for ipopt [2]
```cpp
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

```

### Timestep Length and Elapsed Duration (N & dt)
Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Hyperparameters N and dt are chosen with the following criterias:
1. Determine a good range T = N * dt as the model shouldn't predict too far way as a moving vehicle.
2. Then choose a dt that gives enough steps to reduce overall cost.

The following hyperparameters were experimented with the model:

| Hyperparameters | Test Result |
|-------|--------|
| N = 25, dt = 0.100 | 2.5s seems too long it tries to fit some waypoints far way!!! |
| N = 5, dt = 0.100 | It does not have enough steps for proper polynomial fit.|
| N = 5, dt = 0.050 | Pass the test in simulator. |


### Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints.
If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

Polynomial fit is performed after transforming waypoints from global map to vehicle space.
```cpp
// Map to vehicle relative
for (int i = 0; i < ptsx.size(); ++i) {
  double x = ptsx[i] - px;
  double y = ptsy[i] - py;
  xvals[i] = x * cos(psi) + y * sin(psi);
  yvals[i] = -x * sin(psi) + y * cos(psi);
}
```

### Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

To deal with that 100 millisecond latency, vehicle state is extrapolated by 100 ms before being passed to the dynamics model. This assues that the initial state in the model reflects the current state in the simulator.
```cpp
VectorXd state(6);
// Use vehicle as origin
// Account for 100ms latency. Extrapolation after 100ms
const double dt = 0.100;
// vehicle moving along its x-axis
const auto current_px = 0.0 + v * cos(-delta) * dt;
const auto current_py = 0.0 + v * sin(-delta) * dt;
const auto current_psi = 0.0 + v * (-delta) / Lf * dt;
const auto current_v = v + a * dt;
const auto current_cte = cte + v * sin(epsi) * dt;
const auto current_epsi = epsi + v * (-delta) / Lf * dt;

state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;
```

## References
[1] [Udacity Self Driving Cars - Mine The Line](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425)
[2] [ipopt](https://projects.coin-or.org/Ipopt)