#include <iomanip>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "MPC.h"
#include "problem.h"
#include "Eigen-3.3/Eigen/Core"

// Maximum steering angle (25 degrees) in radians.
const double MAX_STEER_RADIANS = 25.0 / 180 * M_PI;

// Wait this long before recording stats, in seconds.
const double WARMUP = 5;

// If car is going slower than this, in miles per hour, assume it has crashed.
const double MIN_SPEED = 5;

// If car has absolute CTE larger than this, in meters, assume it has crashed.
const double MAX_CTE = 4.5;

//
// MPC class definition implementation.
//
MPC::MPC(ReferencePolynomial &reference, Problem &problem) :
  reference(reference),
  problem(problem),
  vars(N_VARS),
  vars_lowerbound(N_VARS), vars_upperbound(N_VARS),
  constraints_lowerbound(N_CONSTRAINTS), constraints_upperbound(N_CONSTRAINTS)
{
  Reset();

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < throttle_start; i++) {
    vars_lowerbound[i] = -MAX_STEER_RADIANS;
    vars_upperbound[i] = MAX_STEER_RADIANS;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = throttle_start; i < N_VARS; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // All of these should be 0 except the initial
  // state indices.
  for (int i = 0; i < N_CONSTRAINTS; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
}

MPC::~MPC() {}

void MPC::Reset() {
  // Initial latency estimate, before we start estimating it.
  const double LATENCY_DEFAULT = 0.15;

  reference.Reset();

  t_init = std::chrono::steady_clock::now();
  t = t_init;
  crashed = false;
  runtime = 0;
  previous_speed = 0;
  distance = 0;
  previous_cte = 0;
  total_absolute_cte = 0;

  latency = LATENCY_DEFAULT;
}

void MPC::Update(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector,
  double px, double py, double psi,
  double speed_mph, double delta, double throttle)
{
  // Smoothing factor for the exponential moving average of the timestep.
  const double LATENCY_SMOOTH = 0.1;

  // Try to get the speed and acceleration into metric units so we can calculate
  // with them.
  double speed = speed_mph * MPH_TO_METERS_PER_SECOND;

  auto new_t = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt_duration = new_t - t;
  double new_latency = dt_duration.count();
  latency = new_latency * LATENCY_SMOOTH + latency * (1 - LATENCY_SMOOTH);
  t = new_t;

  reference.Update(ptsx_vector, ptsy_vector, px, py, psi);

  // calculate the cross track error
  double cte = reference.coeffs[0];

  if (tuning) {
    std::chrono::duration<double> runtime_duration = new_t - t_init;
    runtime = runtime_duration.count();
    if (runtime > WARMUP && (fabs(cte) > MAX_CTE || speed_mph < MIN_SPEED)) {
      crashed = true;
    }

    double average_speed = (speed + previous_speed) / 2.0;
    distance += average_speed * new_latency;
    previous_speed = speed;

    total_absolute_cte += fabs((cte + previous_cte) / 2.0) * new_latency;
    previous_cte = cte;
  }

  // Project forward to compensate for latency. These are the same equations
  // used in the optimization problem, but x0, y0 and psi0 are zero here,
  // because we have used them to transform the waypoints.
  double acceleration = throttle_to_acceleration(throttle, speed);
  double x0 = speed * latency;
  double y0 = 0;
  double psi0 = - speed * delta / Lf * latency;
  double v0 = speed + acceleration * latency;

  // Set the initial variable values
  vars[x_start] = x0;
  vars[y_start] = y0;
  vars[psi_start] = psi0;
  vars[v_start] = v0;

  // Lower and upper limits for constraints
  constraints_lowerbound[x_start] = x0;
  constraints_lowerbound[y_start] = y0;
  constraints_lowerbound[psi_start] = psi0;
  constraints_lowerbound[v_start] = v0;

  constraints_upperbound[x_start] = x0;
  constraints_upperbound[y_start] = y0;
  constraints_upperbound[psi_start] = psi0;
  constraints_upperbound[v_start] = v0;

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, Problem>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, problem, solution);

  // Print tracing info.
  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!tuning) {
    std::cout <<
      "ok=" << ok <<
      " cost=" << setw(8) << solution.obj_value <<
      " latency=" << setw(8) << latency << std::endl;
  }

  vars = solution.x;
}

double MPC::steer() const {
  // Note: the delta in the problem is positive for a left turn and negative
  // for a right turn; the simulator uses the opposite convention.
  return -vars[delta_start] / MAX_STEER_RADIANS;
}

double MPC::throttle() const {
  return vars[throttle_start];
}

std::vector<double> MPC::x_values() const {
  return get_variable(x_start, N);
}

std::vector<double> MPC::y_values() const {
  return get_variable(y_start, N);
}

std::vector<double> MPC::psi_values() const {
  return get_variable(psi_start, N);
}

std::vector<double> MPC::v_values() const {
  return get_variable(v_start, N);
}

std::vector<double> MPC::delta_values() const {
  return get_variable(delta_start, N - 1);
}

std::vector<double> MPC::throttle_values() const {
  return get_variable(throttle_start, N - 1);
}

std::vector<double> MPC::get_variable(size_t start, size_t count) const {
  std::vector<double> ys(count);
  for (size_t i = 0; i < count; ++i) {
    ys[i] = vars[start + i];
  }
  return ys;
}

std::ostream &operator<<(std::ostream &os, const MPC &mpc) {
  os << "{\"runtime\":" << mpc.runtime
    << ", \"distance\":" << mpc.distance
    << ", \"total_absolute_cte\":" << mpc.total_absolute_cte << "}";
  return os;
}
