# Model Predictive Control
Self-Driving Car Engineer Nanodegree Program

## Project Objectives

In this project a Model Predictive Control algorithm is implemented in order to drive the car around the track in the Udacity simulator. This time the cross track error is not given and needs to be calculated! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

## Project Files

* [Source code](https://github.com/schambon77/CarND-MPC/tree/master/src)
* [ReadMe](https://github.com/schambon77/CarND-MPC/blob/master/README.md)

## Project Rubric Points

### Compilation

The code compiles without errors with cmake and make.

### Implementation

#### The Model

From the data passed by the simulator, we update the following state variables:

* `ptsx`: waypoints x map coordinates
* `ptsy`: waypoints y map coordinates
* `px`: car x map coordinate
* `py`: car y map coordinate
* `psi`: car heading angle in map coordinates
* `v`: car speed
* `steering_angle`: car steering angle
* `throttle`: car acceleration / deceleration value

The state is updated with a 100ms simulation using the motion model in order to handle the latency (cf. section "Model Predictive Control with Latency").

The waypoints map coordinates are then converted into car coordinates using the `convert2carcoordinates` function. The resulting vectors `ptsx_conv` and `ptsy_conv` are then used to fit a polynomial (cf. section "Polynomial Fitting and MPC Preprocessing"). We obtain the coefficients vector `coeffs_conv`.

In car coordinates, the car is located at the origin (0, 0). The polynomial coefficients are then used in a simple manner to compute:

* the cross-track error `cte` with `coeff_conv[0]`
* and the heading error `epsi` with `coeff_conv[1]`

The state in car coordinates is compiled by adding to the origin coordinates (0, 0) a heading angle 0 and the computed `cte` and `epsi`. The state is passed along the polynomial coefficients to our Model Predictive Control class `Solve` method. 

The skeleton of the MPC class studied in the lessons is heavily reused, including:

* initial state updated from the car state input argument 
* lower and upper bounds for all variables
* constraints for non-actuator variables

The solver uses the following cost function:

	// The part of the cost based on the reference state.
	for (t = 0; t < N; t++) {
	  fg[0] += 10*(1 + 3*t/N)*CppAD::pow(vars[cte_start + t], 2);
	  fg[0] += 10*(1 + 3*t/N)*CppAD::pow(vars[epsi_start + t], 2);
	  fg[0] += 1*CppAD::pow(vars[v_start + t] - ref_v, 2);
	}

	// Minimize the use of actuators.
	for (t = 0; t < N - 1; t++) {
	  fg[0] += 10*CppAD::pow(vars[delta_start + t], 2);
	  fg[0] += 10*CppAD::pow(vars[a_start + t], 2);
	}

	// Minimize the value gap between sequential actuations.
	for (t = 0; t < N - 2; t++) {
	  fg[0] += 100000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	  fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	}

The same cost features are used, with noticeable differences on the weight of each feature. Noticeably 2 points:

* a very high weight on the steering variations (100000), in order to keep smooth driving without abrupt steering changes
* a gradually increasing weight for `cte` and `epsi` errors for more distant points: the idea is to emphasize focus on the further horizon than the immediate vicinity of the car

The update equations rename vastly untouched, except for the update of `psi` which requires a change of sign to incorporate the simulator convention on steering angle sign.

	  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	  fg[1 + psi_start + t] = psi1 - (psi0 - v0 * (delta0 / Lf) * dt);  //model equation modified to factor positive angle means right turn
	  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
	  fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
	  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * (delta0 / Lf) * dt);

Note f0 and psides0 require the polynomial coefficients to be calculated.

The resulting immediate next step actuator values for steering angle and acceleration, as well as all x and y coordinates of predicted future states, are returned to the main function, to be passed back to the simulator.

#### Timestep Length and Elapsed Duration (N & dt)

After many trials, the chosen values are:

* `N`: 25
* `dt`: 0.05

This is actually strongly correlated to the chosen reference speed `ref_v` set to 50. Once the car takes some speed, the chosen values allow a reasonnable look-ahead prediction (1.25 second, equivalent to 28 meters), without going too far, which would be unecessary.

Other tried values include a combination of much larger `N` and slower reference speeds, but proved unstable due to my cost weights.

#### Polynomial Fitting and MPC Preprocessing

The following method is used to convert waypoints from map coordinates to car coordinates:

    Eigen::VectorXd convert2carcoordinates(double car_map_pos_x, double car_map_pos_y, double car_map_psi, double point2convert_x, double point2convert_y) {

    double x = point2convert_x - car_map_pos_x;
    double y = point2convert_y - car_map_pos_y;

    Eigen::VectorXd newpoint(2);
    newpoint[0] = x * cos(-car_map_psi) - y * sin(-car_map_psi);
    newpoint[1] = x * sin(-car_map_psi) + y * cos(-car_map_psi);

    return newpoint;
  }

A polynomial of order 3 (as recommended in the lesson) is then fitted on the converted waypoints.

#### Model Predictive Control with Latency

Latency is handled by the following piece of code, which updates the state variables `px`, `py`, `psi` and `v` with the motion model equations: 

          double latency = 0.1;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi -= (v/Lf) * steering_angle * latency;
          v += throttle * latency;

Note this is the first transformation performed (before polynomial fitting).

### Simulation

The vehicle successfully drives a lap safely around the track at an average speed slighty below 50 mph.
