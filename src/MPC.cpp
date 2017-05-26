#include <iomanip>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Number of time steps in the receding horizon problem.
const size_t N = 20;

// Number of variables (N timesteps => N - 1 actuations).
const size_t N_VARS = N * 6 + (N - 1) * 2;

// Number of constraints.
const size_t N_CONSTRAINTS = N * 6;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const double ref_v = 50; // mph

// Maximum steering angle (25 degrees) in radians.
const double MAX_STEER_RADIANS = 25.0 / 180 * M_PI;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  double dt;
  const Eigen::VectorXd &coeffs;
  // Coefficients of the fitted polynomial.
  FG_eval(double dt, const Eigen::VectorXd &coeffs) : dt(dt), coeffs(coeffs) { }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += CppAD::pow(vars[cte_start + i] - 0, 2);
      fg[0] += CppAD::pow(vars[epsi_start + i] - 0, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += 20 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 20 * CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] +
        coeffs[1] * x0 +
        coeffs[2] * x0 * x0 +
        coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(
        coeffs[1] +
        2 * coeffs[2] * x0 +
        3 * coeffs[3] * x0 * x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

// class FG_eval {
//  public:
//   // Fitted polynomial coefficients
//   Eigen::VectorXd coeffs;
//   FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
//
//   typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
//   void operator()(ADvector& fg, const ADvector& vars) {
//     // fg a vector of constraints, x is a vector of constraints.
//     // The cost is stored is the first element of `fg`.
//     // Any additions to the cost should be added to `fg[0]`.
//
//     // Add the cost at the end of the horizon first; will add the rest below.
//     fg[0] = CppAD::pow(vars[cte_start + N - 1], 2) +
//       CppAD::pow(vars[epsi_start + N - 1], 2) +
//       CppAD::pow(vars[v_start + N - 1] - ref_v, 2);
//
//       //
//       // Setup Constraints
//       //
//
//       // Initial constraints
//       //
//       // We add 1 to each of the starting indices due to cost being located at
//       // index 0 of `fg`.
//       // This bumps up the position of all the other values.
//       fg[1 + x_start] = vars[x_start];
//       fg[1 + y_start] = vars[y_start];
//       fg[1 + psi_start] = vars[psi_start];
//       fg[1 + v_start] = vars[v_start];
//       fg[1 + cte_start] = vars[cte_start];
//       fg[1 + epsi_start] = vars[epsi_start];
//
//       // The rest of the constraints
//       for (size_t i = 0; i < N - 1; i++) {
//         AD<double> x1 = vars[x_start + i + 1];
//         AD<double> y1 = vars[y_start + i + 1];
//         AD<double> psi1 = vars[psi_start + i + 1];
//         AD<double> v1 = vars[v_start + i + 1];
//         AD<double> cte1 = vars[cte_start + i + 1];
//         AD<double> epsi1 = vars[epsi_start + i + 1];
//
//         AD<double> x0 = vars[x_start + i];
//         AD<double> y0 = vars[y_start + i];
//         AD<double> psi0 = vars[psi_start + i];
//         AD<double> v0 = vars[v_start + i];
//         AD<double> cte0 = vars[cte_start + i];
//         AD<double> epsi0 = vars[epsi_start + i];
//
//         AD<double> delta0 = vars[delta_start + i];
//         AD<double> a0 = vars[a_start + i];
//
//         // Reference State Cost
//         fg[0] += CppAD::pow(cte0, 2) +
//           CppAD::pow(epsi0, 2) +
//           CppAD::pow(v0 - ref_v, 2);
//
//         // // Actuator smoothing cost.
//         // if (i < N - 2) {
//         //   AD<double> delta1 = vars[delta_start + i + 1];
//         //   AD<double> a1 = vars[a_start + i + 1];
//         //   fg[0] +=
//         //     0.1 * CppAD::pow(delta1 - delta0, 2) +
//         //     0.1 * CppAD::pow(a1 - a0, 2);
//         // }
//
//         AD<double> f0 = coeffs[0] + coeffs[1] * x0;
//         AD<double> psides0 = CppAD::atan(coeffs[1]);
//
//         // Setup the rest of the model constraints
//         fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
//         fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
//         fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
//         fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
//         fg[2 + cte_start + i] =
//             cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
//         fg[2 + epsi_start + i] =
//             epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
//       }
//   }
// };

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
  FG_eval fg_eval(dt, reference.coeffs);

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
  CppAD::ipopt::solve<Dvector, FG_eval>(
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
