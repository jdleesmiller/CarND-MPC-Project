# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

- re use the constraints rather than rebuilding them every time
- use the previous vars as the initial solution for the next iteration
  - currently doing this quite naively: just replacing the first time step vars with actuals, but I guess that's not such a bad idea, provided that dt is similar to the actual dt
- TODO should write polynomial using Horner's scheme
- TODO smooth the polynomial coefficients: removing waypoints affects the fit and introduces quite a lot of weave
  - would be nice to know dt so we can smooth over time
  - this did not work. I have another idea: use weighted least squares to fit, so if a waypoint isn't reported, it still gets included for a few iterations, so the polynomial changes more gradually. I think this gives the formula for adapting the QR to : https://www.cs.ubc.ca/~rbridson/courses/542g-fall-2008/notes-oct6.pdf
- TODO latency compensation:
  - estimate dt from recent dt?
  - then can make dt variable  
  - but some risk that if dt becomes too large we get numerical blow up
  - at minimum, need to make the first dt variable; could still fix the rest
  - Using the latency as DT did not work; too big. I think the cleanest solution would be to project the state forward by the estimated latency and then put that into the solver.
- getting some tuning done with no latency might at least get it to the point where I can drive it around the track
- can extract the objective functor to its own file
- would still like to try removing the CTE from the vars

- figuring out units: throttle/acceleration relationship
- latency compensation a must on my laptop; tricky: different sign convention for "delta"
- transform the reference points to compensate for latency, too?
  - we transform to vehicle coordinates at the start of the loop, but then the simulator plots them at the end of the loop. I think that's why the reference trajectory seems to move a bit when the vehicle is steering hard toward it.

### Tuning Results

tune_2: crashed

tune_3: apparently it does not work when locked; first iteration is OK
- no added latency, -O3
inputs:
```
initial_weights =
  NArray[-0.0369763, -2.50752, 0.1058, 1.48699, 4.02008, -0.272567]
initial_weights_stddev =
  NArray[1.34766, 1.0355, 1.91644, 1.07627, 2.07593, 1.87217] * 1.5
```
results:
```
# NArray.float(6):
# [ 0.188185, -1.9549, -0.0288748, 0.269565, 4.68395, -0.259999 ]
# NArray.float(6):
# [ 1.40027, 0.703136, 1.29065, 0.882833, 1.7413, 1.47497 ]
```

tune_4: accidentally locked; one iteration OK
```
NArray.float(6):
[ 0.187376, -1.50104, 0.530293, -0.964634, 4.68049, -0.589437 ]
NArray.float(6):
[ 1.51579, 0.630768, 1.32019, 0.516218, 0.989551, 1.02348 ]
```

tune_5: accidentally broken

tune_6: first run that included the 100ms latency

NArray.float(6):
[ -0.566452, -1.28948, 0.960193, -1.10084, 5.95476, -1.09054 ]
NArray.float(6):
[ 1.59474, 0.186256, 1.4063, 0.564281, 0.622535, 1.25145 ]
NArray.float(6):
[ -0.878479, -1.12471, 0.941112, -0.872981, 6.37622, -0.773817 ]
NArray.float(6):
[ 1.61116, 0.347754, 1.45068, 0.328843, 0.349873, 1.06035 ]
NArray.float(6):
[ -1.53904, -1.06667, 1.58644, -1.24987, 6.53842, -0.573538 ]
NArray.float(6):
[ 1.38876, 0.341757, 1.99009, 0.296602, 0.433879, 0.959922 ]
NArray.float(6):
[ -0.613693, -0.966355, 1.57587, -1.4254, 6.72493, -1.45234 ]
NArray.float(6):
[ 1.16105, 0.381916, 2.35653, 0.387587, 0.398655, 0.625377 ]
NArray.float(6):
[ -0.945875, -0.496115, 2.72138, -1.42903, 6.83475, -1.20666 ]
NArray.float(6):
[ 1.17103, 0.457922, 1.52162, 0.454173, 0.473798, 0.899891 ]
NArray.float(6):
[ -0.904017, -0.305483, 1.84108, -1.62267, 6.87089, -1.56572 ]
NArray.float(6):
[ 1.69159, 0.205871, 1.95127, 0.217392, 0.285529, 0.503974 ]



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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
