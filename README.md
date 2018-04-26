# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

This project implements Model Predictive Control using c++ to drive the car around the simulator track. The simulator provides the way points - x and y cooridnates, current car position, velocity and orientation as inputs. The cross track error and orientation error are calculated dynamically in the application and used to generate an optimized solution with a steer angle and throttle value that will minimize the cross track error. The simulated car's actuators have a delay of 100 ms that is accounted in the MPC calcualtion.

## Project Steps

* Fitting a line based on given waypoints and evaluating the current state based on that polynomial line.
* Accounting for latency (I used a predicted state 100ms in the future to replace the actual current state in the calculation)
* Implementing the MPC calculation, including setting variables and constraints
* Calculating actuator values from the MPC calc based on current state
* Calculating steering angle & throttle/brake based on the actuator values
* Setting timestep length and duration
* Testing/tuning of above implementations on Udacity simulator

## Model description

The model takes the following inputs from the simulator.

* ptsx (Array) - The global x positions of the waypoints.
* ptsy (Array) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
* psi (float) - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions.
* x (float) - The global x position of the vehicle.
* y (float) - The global y position of the vehicle.
* steer_angle (float) - The current steering angle in radians.
* throttle_value (float) - The current throttle value [-1, 1].
* v (float) - The current velocity in mph.

### Preprocessing & Polyfit

In order to simply calculations the waypoint coordinates are transformed into vehicle coordinates using vector transformation equations (as in line 119-125) > Here the vehicle is assumed to be at x,y position 0,0 with orientation 0.

Using the transformed way points, a polynomial line of third degree is fit in using the polyfit() function. The cross track error and orientation error for the current state are calculated using the polyeval() function at px.

Next, the latency of actuation control is accounted by predicting the future state at given latency which is 100ms and assume it as current state position.

### MPC Solve

In the MPC.cpp, a mathematical optimization model was defined with variables, constraints along with their respective thresholds and the cost function. The model was defined to solve the optimal steer and throttle values which can minimize the cost. I chose the global parameters N = 10 and dt = 0.1 to do the prediction for 10 * 1 seconds. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

