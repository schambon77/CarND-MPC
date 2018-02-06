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

Student describes their model in detail. This includes the state, actuators and update equations.

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
* and the heading error epsi with `coeff_conv[1]`





#### Timestep Length and Elapsed Duration (N & dt)

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

#### Polynomial Fitting and MPC Preprocessing

A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

Eigen::VectorXd convert2carcoordinates(double car_map_pos_x, double car_map_pos_y, double car_map_psi, double point2convert_x, double point2convert_y) {

  double x = point2convert_x - car_map_pos_x;
  double y = point2convert_y - car_map_pos_y;

  Eigen::VectorXd newpoint(2);
  newpoint[0] = x * cos(-car_map_psi) - y * sin(-car_map_psi);
  newpoint[1] = x * sin(-car_map_psi) + y * cos(-car_map_psi);

  return newpoint;
}

polynomial order 3 with converted waypoints



#### Model Predictive Control with Latency

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

          double latency = 0.1;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi -= (v/Lf) * steering_angle * latency;
          v += throttle * latency;


### Simulation

The vehicle successfully drives a lap safely around the track.
