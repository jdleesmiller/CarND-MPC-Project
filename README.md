# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Assessment

### Implementation

#### The Model

The model is similar to the one presented in the lecture, with a few modifications as described below.

Profiling showed that ~85% of the time (when compiled with `-O3`) was spent in the IPOPT solver, so I spent most of my time on the solver and model, rather than the other tasks, such as waypoint fitting.

1. Instead of including the `cte` (cross track error) and `epsi` (steering angle error) terms in the state of the optimization problem, I put them into the objective function directly. This reduced the number of state variables from `N * 6` to `N * 4`, and it reduced the number of constraints similarly. Tests showed a latency reduction of about 10ms when compiling without `-O3`, but this difference was no longer visible when compiling with `-O3`. It therefore had little performance impact, but it did simplify some of the code.

1. Rather than reinitializing the `vars` to zero on every iteration, I used the `vars` from the previous solve as the initial solution. The rationale is that the variable values (in vehicle coordinates) should not be very different between MPC iterations, so the values from the previous iteration should provide a good initial guess for the current iteration. This also makes it less likely that subsequent iterations will choose drastically different plans, which might lead to oscillations. In my tests this change reduced latency by about 10ms (with `-O3`), which is again not much; apparently IPOPT is pretty smart.

1. Rather than rebuilding the constraints in every iteration, the `MPC` class stores them, since the only constraint that changes between iterations is the initial value constraint.

1. Rather than using the `throttle` value from the simulator, which is a unitless quantity in `[-1, 1]`, directly as the acceleration in the state equations, I estimated acceleration from the throttle and current speed. Empirically, full throttle at low speed seems to give an acceleration of 5/ms^2 (which is quite high!), and this reduces gradually as speed increases. See `data/acceleration_estimate.xslx` for details.

#### Timestep Length and Elapsed Duration (N & dt)

I found that `N = 20` and `dt = 0.05` worked well in initial manual testing. These settings give a total lookahead of 1s, which empirically was enough for the vehicle to plan its way through the corners in the test track.

I tuned the remaining parameters (namely the cost coefficients in the optimization problem) with these values for `N` and `dt`.

In principle, smaller `N` should lead to lower latency due to reduced compute times but also may result in a plan that does not extend far enough into the future. Conversely, larger `N` should lead to larger latency due to increased compute time, but it lets the controller plan further ahead. If `dt` is too large, then the approximations underlying the numerical integration scheme that we use to set the state constraints may break down.

I tested some other `N` and `dt` values, using the same cost coefficients (which were tuned as described below) and a 50mph reference speed:

| N  | dt (s) | latency (s) | Crashed? |
|----|--------|-------------|---------|
| 10 | 0.05   | 0.13        | No |
| 30 | 0.05   | 0.14        | No |
| 20 | 0.02   | 0.14        | No |
| 20 | 0.10   | 0.14        | No |

Qualitatively, the car handled much the same in each of the above trials. So, in practice, the results do not seem very sensitive to the choice of `N` and `dt`.

#### Polynomial Fitting and MPC Preprocessing

I noticed that the fitted polynomial for the reference trajectory often changed suddenly when the simulator changed the set of reference waypoints that it sent in the telemetry packet. To compensate, the controller (in `reference_polynomial.cpp`), keeps track of the waypoints that it knows about, weights them based on how new or old they are, and then uses weighted least squares (rather than ordinary least squares) to fit the polynomial. When a waypoint is seen for the first time, it starts with a low weight that gradually increases; when a waypoint is no longer seen, its weights decrease until they hit zero, at which point the controller forgets about the waypoint.

The modification to the polynomial fitting routine to support weighted least squares is based on [these notes](https://www.cs.ubc.ca/~rbridson/courses/542g-fall-2008/notes-oct6.pdf).

The only preprocessing on the state and actuators was to convert units to SI units as required and to reconcile different sign conventions, e.g. for the steering angle, `delta`, and the orientation of the car, `psi`. (And you could view the translation of the throttle into acceleration, which was described above, and the latency compensation, which is described below, as preprocessing steps.)

#### Model Predictive Control with Latency

To deal with the 100ms of extra latency that we are required to add, the controller keeps an estimate of the latency using an exponential moving average of recent observed latencies. It then predicts the state of the vehicle forward by this estimated latency to produce the initial conditions for the optimization problem. It uses the same kinematic model as the one in the optimization problem.

It's worth noting that, because I removed the `cte` and `epsi` variables from the state, this prediction was a bit simpler than it might otherwise have been.

### Tuning Results

I tuned the coefficients in the cost function using the Cross Entropy Method (CEM), as I did in [the PID project](https://github.com/jdleesmiller/CarND-PID-Control-Project).

The CEM is similar in spirit to a genetic algorithm, but instead of working with 'genes', it works with a probability distribution over the parameters. You start with a prior distribution, and then CEM

1. generates samples based on that distribution,
1. scores them according to the objective function, and
1. uses the lowest-scoring samples (if we want to minimize the objective function, in this case negative distance) to update the parameters of the probability distribution.

This process repeats for each 'generation'. It is relatively resistant to randomness in the objective function, because it averages over a (configurable) number of samples in each generation. I used an implementation of the CEM that I wrote a few years ago: https://github.com/jdleesmiller/cross_entropy

The parameters to be tuned here were `cte_weight`, `epsi_weight`, `v_weight` (speed), `delta_weight` (steering), `throttle_weight`, `delta_gap_weight`, `throttle_gap_weight`. I fixed `cte_weight` at 1 to give the objective an overall magnitude, which left 6 other parameters to tune. I used the negative of distance travelled in a fixed time (90s) as the scoring function to be minimized (that is, the parameters should be tuned to make the car should travel as far as possible, or equivalently to have the highest average speed).

It seemed reasonable to constrain all of the weights to be non-negative, so I used the lognormal distribution as the sampling distribution.

The initial weights were set to zero with a variance of 3 (in `tune.rb`):

```ruby
initial_weights = NMath.log(NArray[1, 1, 1, 1, 1, 1])
initial_weights_stddev = NArray[3, 3, 3, 3, 3, 3]
```

I started tuning quite early in the development process, before all of the changes detailed above were complete, and gradually refined the estimates. The final tuning run `tune_7` started from

```ruby
initial_weights =
  NArray[-0.904017, -0.305483, 1.84108, -1.62267, 6.87089, -1.56572]
initial_weights_stddev =
  NArray[1.69159, 0.205871, 1.95127, 0.217392, 0.285529, 0.503974]
```

As in the PID project, the CEM showed some 'risk seeking' behaviour, particularly in later iterations: if a set of parameters often resulted in fast laps (that is, long distances) but sometimes crashed spectacularly, that set of parameters would often make it into the 'elite' samples that seeded the next generation. It therefore tended to select for dangerous driving.

I didn't have time to run the tuning for very long, so this problem was not so severe. I worked around it by choosing the solutions with minimum cross track error out of those tried (which didn't crash) in the final CEM run. In particular, I chose the most 'timit' solution using the `tune.R` script:

```
> # Timid: smallest error among our solutions
> head(dOK[order(dOK$total_absolute_cte),])
   max_runtime   dt reference_speed cte_weight epsi_weight  v_weight
1           90 0.05              50          1 32.02384590 0.6092036
5           90 0.05              50          1  0.16306562 0.8004959
3           90 0.05              50          1  0.09002779 0.8727496
11          90 0.05              50          1  0.07199585 0.7597734
59          90 0.05              50          1 21.85268738 0.6737644
10          90 0.05              50          1  0.20808638 0.8189340
   delta_weight throttle_weight delta_gap_weight throttle_gap_weight crashed
1    4.46984276       0.2217726         833.4191           0.1247289   FALSE
5    0.06660825       0.2277583        1159.8454           0.2108857   FALSE
3    0.86883336       0.1778646        1065.2655           0.1422173   FALSE
11   0.48275760       0.2048286        1283.0884           0.1609739   FALSE
59  19.16793491       0.2018953        1094.3281           0.4070019   FALSE
10  18.99809940       0.2100364        1247.8385           0.2185488   FALSE
   runtime distance total_absolute_cte
1  90.0369  1910.06            14.1555
5  90.0877  1911.28            15.3508
3  90.0200  1913.71            15.3942
11 90.1323  1913.67            15.7798
59 90.0218  1908.25            16.2166
10 90.0277  1911.11            16.2525
> # Aggressive: fastest laps
> head(dOK[order(-dOK$distance),])
   max_runtime   dt reference_speed cte_weight epsi_weight  v_weight
19          90 0.05              50          1  0.48942483 0.8635292
64          90 0.05              50          1  1.36395097 0.5828646
25          90 0.05              50          1  0.26740834 0.8637655
22          90 0.05              50          1  2.90027620 1.0071999
9           90 0.05              50          1  0.09506206 0.8464539
6           90 0.05              50          1  1.14880705 0.8859923
   delta_weight throttle_weight delta_gap_weight throttle_gap_weight crashed
19    0.7391599      0.15809261        1205.4096           0.1897028   FALSE
64    3.9846583      0.09862498        1098.3177           0.1437580   FALSE
25  143.1512323      0.19321711        1248.9559           0.1886518   FALSE
22    7.2168487      0.18224109         647.1699           0.1732579   FALSE
9     0.6336257      0.17103902         695.9972           0.1509335   FALSE
6     1.7941819      0.21369379         853.2799           0.1392649   FALSE
   runtime distance total_absolute_cte
19 90.1670  1916.60            18.1583
64 90.0928  1915.05            24.4139
25 90.1307  1914.88            20.8281
22 90.1831  1914.70            57.4043
9  90.1438  1914.21            36.7277
6  90.1031  1913.94            20.2169
> head(dOK[order(-dOK$distance),])
```

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

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.
