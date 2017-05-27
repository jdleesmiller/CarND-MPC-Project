#include "problem.h"

const size_t N = 20;
const size_t N_VARS = N * 6 + (N - 1) * 2;
const size_t N_CONSTRAINTS = N * 6;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

using CppAD::AD;

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

const double DEFAULT_DT = 0.05;

Problem::Problem(const ReferencePolynomial &reference)
  : reference(reference), dt(DEFAULT_DT)
{ }

// `fg` is a vector containing the cost and constraints.
// `vars` is a vector containing the variable values (state & actuators).
void Problem::operator()(ADvector& fg, const ADvector& vars) {
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

    AD<double> f0 = reference.coeffs[0] +
      reference.coeffs[1] * x0 +
      reference.coeffs[2] * x0 * x0 +
      reference.coeffs[3] * x0 * x0 * x0;
    AD<double> psides0 = CppAD::atan(
      reference.coeffs[1] +
      2 * reference.coeffs[2] * x0 +
      3 * reference.coeffs[3] * x0 * x0);

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
