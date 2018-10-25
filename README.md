# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Project Rubric Points Addressed
### Compilation: see Dependencies and instructions below

### Implementaion:
#### Model
The Model Predictive Controller (MPC) takes global way points and transforms from map coordinates to vehicles coordinates. Then it uses vehicle "measurements" provided by the simulator and the transformed way points to calculate the cross-track error (CTE) and psi error (EPSI) [main.cpp, lines 122-158]. The measurements and errors are used to create a state vector: [x, y, psi, v, cte, epsi]; where x and y are the position of the car on the track, psi is the orientation of the car, v is the current linear speed. CTE is calcuated as the difference between the center of the lane, the desired position, and the car's position along the y-axis: `cte = y - f(x)`, where y = 0 is the desired position and f(x) is the vehicles y position, according the the fitted polynomials discussed below [Note: Since the simulator measures negative angles as counter-clockwise, the cte equation becomes `f(x) - y`]. The epsi is the difference between desired orientation (a straight line) and the actual car's orientation. 

Way points, received from the simulator, are transformed from global map coordinates to vehicle coordinates, and a 3rd-degree polynomial is fit to the points to yield a desired trajectory. To control for latency, the current state at time t (state provided by the simulator) is used to estimate the next state at timestep t+1. This next state is then passed to the MPC [main.cpp, lines 166-203]. 

The MPC calculates a cost function [MPC.cpp, lines 50-67], based on the state at t+1, and returns the actuator values that minimize the cost function. The MPC uses equations (Figure 1) as constraints on the solver, taking the results from the calculated next timestep and subtracting the update equation at time step t (MPC.cpp, lines 120-127) forcing the result, stored in fg, to be zero. 

#### Figure 1: Update Equation (taken from Lesson 19, section 6)
<img src="https://github.com/bhumphrey0x20/CarND-MPC-Project/blob/master/ModelEqns.png" height="195" width="302" />


Actuators include the steering angle, `delta` and acceleration `a`. The steering angle has a maximum and minimum value set to +/- 0.436332 radians (+/- 25 degrees) and acceleration maximum and minimum of +/- 1 (MPC.cpp, lines 193-203). The MPC finds the acuator values (steering angle and acceleration) that minimize the cost function. These values are then passed to the simulator for vehicle control.


#### Timestep and Elapsed Duration
The Time Horizon, `T = N * dt`, is the how far ahead in the future the vehicle's calculated path is projected. Here N is the number of future time steps and dt is the time between each step, or between each actuation. N is used to set the size of the vector optimized by the MPC: 

`vector size = N * [Number of States] + (N-1) * [Number of Actuators]`. 

Here the number of states is 6 and the number of actuators is 2. For each unit increase in N the size of the vector optimized by the MPC increases by 8. This in turn increases the processing time of the MPC. 

dt is the time changed used in the update equations and determines the resolution of the projected path. A small dt yields a small time change between predicted values and more accurately projects the trajectory/path, while a large dt increases the error between the desired path and the estimated path. 

For this project, values for N and dt were chosen to keep T relatively low ( <= 1.5 seconds). In the final implementaion an N = 7 and a dt = 0.15 was used based on quality of driving and a lower average MPC processing time (approximately 9.05 milliseconds). N values between 5 and 15 were tested using dt values between 0.05 and 0.2. N values of 15 worked well at slower speed (10 and 20 mph) but at faster speed the projected trajectory tended to skew too much resulting in the vehicle frequently drove off the road.
N value of 5 was too short of a projection, causing the car occillated too much and crash. 

dt values between 0.15 and 0.75 tended to work the best with various N values. dt values of 0.2 tended to make the car "hug" the side of the road (too much straightline projection between time steps) especially around curves where the vehicle tended to drive over the line and onto the curb. Values less than 0.075 made the car occilate and crash. 
 

#### Polynomial Fitting and MPC Preprocessing
Prior to MPC processing, way points from the simulator were tranformed from global coordinates to vehilce coordinates using a Homogeneous Transformation (as discussed in the particle filter project, Lesson 14) [main.cpp, lines 141-146]. A 3rd degree polynomial was then fit to the transformed points using  the `polyfit()` function [main.cpp, lines 150]. 

#### Model Predictive Control with Latency
Following the Slack discussion [here]("https://carnd.slack.com/archives/C54DV4BK6/p1538209080000100"), a latency of 100 milliseconds was handled by calculating a new state vector for time = t+1, using the kinematic equations disscussed in Lesson 18 [main.cpp, lines 171-186]. The new state vector was then passed to the MPC. During the first new state vector calculations, the initial values of the control inputs- steering angle (`delta_1`) and acceleration (`a_1`)- were set to 0. Afterwards, the control input values returned from the MPC were stored [main.cpp, lines 210-211] and used in the kinematic equations during the next callback loop, to calculate the next state: basically, solving control inputs for a time, one step into the future. 

Staying consistent with the `dt` value in the MPC, a time step `dt` of 0.15 seconds was used as a "delta t" in the kinematic equations [main.cpp, lines 36, 171-186].  

### Simulation
The video links below show the vehicle controled by the MPC using reference velocities of 20 mph and 30 mph. 

#### MPC with Reference Velocity = 20 mph
<a href="https://youtu.be/NVu9ff7MVhc" target="_blank"><img src="https://i.ytimg.com/vi/NVu9ff7MVhc/2.jpg" alt="Advanced Lane Finding Video" width="240" height="180" border="10" /></a>


#### MPC with Reference Velocity = 30 mph
<a href="https://youtu.be/OTB-UnRqayc" target="_blank"><img src="https://i.ytimg.com/vi/OTB-UnRqayc/2.jpg" alt="Advanced Lane Finding Video" width="240" height="180" border="10" /></a>


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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
