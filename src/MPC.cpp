#include <iomanip>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "MPC.h"
#include "problem.h"
#include "Eigen-3.3/Eigen/Core"

// Maximum steering angle (25 degrees) in radians.
const double MAX_STEER_RADIANS = 25.0 / 180 * M_PI;

//
// MPC class definition implementation.
//
MPC::MPC() :
  t(std::chrono::steady_clock::now()),
  latency(0),
  vars(N_VARS),
  vars_lowerbound(N_VARS), vars_upperbound(N_VARS),
  constraints_lowerbound(N_CONSTRAINTS), constraints_upperbound(N_CONSTRAINTS)
{
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -MAX_STEER_RADIANS;
    vars_upperbound[i] = MAX_STEER_RADIANS;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < N_VARS; i++) {
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
  // Initial timestep estimate, before we start estimating the timestep.
  const double LATENCY_DEFAULT = 0.15;

  t = std::chrono::steady_clock::now();
  latency = LATENCY_DEFAULT;
}

void MPC::Update(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector,
  double px, double py, double psi, double v)
{
  // Smoothing factor for the exponential moving average of the timestep.
  const double LATENCY_SMOOTH = 0.1;

  auto new_t = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt_duration = new_t - t;
  double new_latency = dt_duration.count();
  latency = new_latency * LATENCY_SMOOTH + latency * (1 - LATENCY_SMOOTH);
  t = new_t;

  reference.Update(ptsx_vector, ptsy_vector, px, py, psi);

  // calculate the cross track error
  double cte = reference.coeffs[0];
  // calculate the orientation error
  double epsi = -atan(reference.coeffs[1]);

  // Set the initial variable values
  vars[x_start] = 0;
  vars[y_start] = 0;
  vars[psi_start] = 0;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for constraints
  constraints_lowerbound[x_start] = 0;
  constraints_lowerbound[y_start] = 0;
  constraints_lowerbound[psi_start] = 0;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = 0;
  constraints_upperbound[y_start] = 0;
  constraints_upperbound[psi_start] = 0;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  double dt = 0.05;
  Problem fg_eval(dt, reference.coeffs);

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
      constraints_upperbound, fg_eval, solution);

  // Print tracing info.
  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  std::cout <<
    "ok=" << ok <<
    " cost=" << setw(8) << solution.obj_value <<
    " latency=" << setw(8) << latency << std::endl;

  vars = solution.x;
}

double MPC::steer() const {
  return vars[delta_start] / MAX_STEER_RADIANS;
}

double MPC::throttle() const {
  return vars[a_start];
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

std::vector<double> MPC::cte_values() const {
  return get_variable(cte_start, N);
}

std::vector<double> MPC::epsi_values() const {
  return get_variable(epsi_start, N);
}

std::vector<double> MPC::delta_values() const {
  return get_variable(delta_start, N - 1);
}

std::vector<double> MPC::a_values() const {
  return get_variable(a_start, N - 1);
}

std::vector<double> MPC::get_variable(size_t start, size_t count) const {
  std::vector<double> ys(count);
  for (size_t i = 0; i < count; ++i) {
    ys[i] = vars[start + i];
  }
  return ys;
}
