# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Discussion

This is a C++ PID controller implimentation for Udacity's lake driving course simulation. There are two PID controllers used in the car - one to control steering angle and one to control throttle.

![See PID drive][./pid_car.ogv]

### Steering
The PID controlling steering angle ended up with cross track error as its input and the following tunings:

* P = 0.2 (A relatively small positive coeficient on the proportional component correction provides sufficient "quick reaction" when the car deviates from the center of the lane.)
* I = 0.0 (With no wheel misalignment, there is no constant estimation error that needs to be compensated for, so the integral component is zero.)
* D = -1.55 (The relatively large negative coeficient for the derivative component mitigates some of the overshooting when the car is returning towards the center of the lane.)

### Throttle
The PID controlling throttle ended up with the absolute value of steering angle as its input.

* P = 0.135 (A small positive coeficient on the proportional component causes the car to start moving.)
* I = 0.0 (Nothing to correct for using the integral component.)
* D = -0.759905 (A relatively large negative component on the derivative component causes the car to accelerate most sharply when the car is tending towards a steering angle of zero... i.e. straightning out.)

### Tuning process
This project was tuned using a combination of twiddle tuning and manually setting values. Twiddle tuning was helpful but seems severely limited because:

* It requires a suitable scoring formula. In this case, scoring needed to be some middle ground balancing CTE and speed.
* In this specific project it seemed to like finding local minimums. I had my tuner stop a few time on some very odd driving.
* Stopping a simulation round early because the car fell off the track created a challenge in scoring.

Ultimately, I ended up tuning on the formula "fabs(cte)/fmax(speed/100, 0.001)", which rewards driving in the center of the lane at faster speeds and avoids division by zero. I ended up putting in initial values manually a few times between separate twiddle tuning sessions to experiment with escaping local minimum tunings. And for ending rounds earlier, I simply had the simulation score assume worst case for the remainder of the timesteps.

### Quirks
I ended up applying a sigmoid transform on both my PID outputs. I modified the PID init functions to also expect a min and max output range, which the logistic output is then translated and scaled to. I think the output may slightly more fluid outputs.

Otherwise, this project isn't really structured well for reusability, but the code gets it done for the sake of this project. Overall, it's been a good learning experience and PID controllers are simple and fun, albiet maybe not the most easily smooth of drivers.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
