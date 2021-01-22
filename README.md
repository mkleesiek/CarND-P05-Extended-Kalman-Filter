# Extended Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project of the Udacity [Self-Driving Car NanoDegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) program, I implement an Extended Kalman Filter to estimate the position and speed of a moving object with noisy lidar and radar measurements.

The goal of this project is to achieve RMSE (root mean square error) values on state variables that are lower than the tolerance outlined in the project rubric. 


## Resources
* [Self-Driving Car NanoDegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) course description at Udacity
* [Extended Kalman Filter Project Starter Code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) on Github


## Prerequisites
You can find detailed instructions on setting up your system on the starter code [project page](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). It involves installing a Unity-based [simulator tool](https://github.com/udacity/self-driving-car-sim/releases) and a C++ webserver library [uWebSocketIO](https://github.com/uWebSockets/uWebSockets).


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


## Structure

The provided starter code was only modified where necessary in order simplify evaluation of the implemented filter logic within the course tooling framework:
* [FusionEKF.h](src/FusionEKF.h) / [FusionEKF.cpp](src/FusionEKF.cpp): Processing of incoming measurements, initialization of the filter and triggering Kalman prediction / update logic. 
* [kalman_filter.h](src/kalman_filter.h) / [kalman_filter.cpp](src/kalman_filter.cpp): Filter logic for prediction and update functions for lidar and radar, including calculation of the required Jacobian matrix.
* [tools.h](src/tools.h) / [tools.cpp](src/tools.cpp): Helper function to calculate the RMSE.


## Summary

Applying the formulas outlined in the course material within a correctly structured filter framework, the achieved RMSE values where well within the required limits:

| RMSE   	| Required	 | Achieved (Lidar + Radar) | Lidar only | Radar only |
|:---------:|:----------:|:------------------------:|:----------:|:----------:|
| x      	| 0.110      | 0.097                    | 0.184      | 0.232      |
| y     	| 0.110	     | 0.086                    | 0.154      | 0.336      |
| v_x	    | 0.520      | 0.452                    | 0.606      | 0.526      |
| v_y	    | 0.520      | 0.440                    | 0.486      | 0.699      |

As expected, lidar measurements alone provide better estimates on the position (x, y) while radar measurements provide better estimates on the velocity (v_x, v_y).
Both measurements combined yield the most accurate results.

## License
The contents of this repository are covered under the [MIT License](LICENSE).
