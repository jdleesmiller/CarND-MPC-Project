#include "problem.h"

using CppAD::AD;

const size_t N = 20;
const size_t N_VARS = N * 4 + (N - 1) * 2;
const size_t N_CONSTRAINTS = N * 4;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t delta_start = v_start + N;
const size_t throttle_start = delta_start + N - 1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
const double Lf = 2.67;

const double MPH_TO_METERS_PER_SECOND = (1609.34 / 3600.0);

const double DEFAULT_DT = 0.05;
const double DEFAULT_REF_V = 50; // mph

const double DEFAULT_CTE_WEIGHT = 1;
const double DEFAULT_EPSI_WEIGHT = 0.6;
const double DEFAULT_V_WEIGHT = 0.3;
const double DEFAULT_DELTA_WEIGHT = 9.3;
const double DEFAULT_A_WEIGHT = 0.1;
const double DEFAULT_DELTA_GAP_WEIGHT = 298;
const double DEFAULT_THROTTLE_GAP_WEIGHT = 0.6;

Problem::Problem(const ReferencePolynomial &reference) :
  reference(reference),
  dt(DEFAULT_DT),
  ref_v(DEFAULT_REF_V),
  cte_weight(DEFAULT_CTE_WEIGHT),
  epsi_weight(DEFAULT_EPSI_WEIGHT),
  v_weight(DEFAULT_V_WEIGHT),
  delta_weight(DEFAULT_DELTA_WEIGHT),
  throttle_weight(DEFAULT_A_WEIGHT),
  delta_gap_weight(DEFAULT_DELTA_GAP_WEIGHT),
  throttle_gap_weight(DEFAULT_THROTTLE_GAP_WEIGHT)
{ }

// `fg` is a vector containing the cost and constraints.
// `vars` is a vector containing the variable values (state & actuators).
void Problem::operator()(ADvector& fg, const ADvector& vars) {
  // The cost is stored is the first element of `fg`.
  // We add 1 to each of the starting indices due to cost being located at
  // index 0 of `fg`. This bumps up the position of all the other values.
  fg[0] = 0;

  //
  // Initial value constraints
  //
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + psi_start] = vars[psi_start];
  fg[1 + v_start] = vars[v_start];

  for (int i = 0; i < N - 1; i++) {
    // The state at time t.
    const AD<double> &x0 = vars[x_start + i];
    const AD<double> &y0 = vars[y_start + i];
    const AD<double> &psi0 = vars[psi_start + i];
    const AD<double> &v0 = vars[v_start + i];

    // The controls at time t.
    const AD<double> &delta0 = vars[delta_start + i];
    const AD<double> &throttle0 = vars[throttle_start + i];
    const AD<double> &a0 = throttle_to_acceleration(throttle0, v0);

    // The state at time t+1.
    const AD<double> &x1 = vars[x_start + i + 1];
    const AD<double> &y1 = vars[y_start + i + 1];
    const AD<double> &psi1 = vars[psi_start + i + 1];
    const AD<double> &v1 = vars[v_start + i + 1];

    //
    // State Constraints
    // Each of these expressions is constrained to be zero.
    //

    fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    fg[2 + v_start + i] = v1 - (v0 + a0 * dt);

    //
    // Objective
    //

    // Steering angle error: The reference angle comes from the derivative of
    // the reference polynomial, which here is written with the Horner scheme.
    const AD<double> &reference_slope =
      reference.coeffs[1] + x0 * (
        2 * reference.coeffs[2] + x0 * (
          3 * reference.coeffs[3]
        )
      );
    const AD<double> &psides0 = CppAD::atan(reference_slope);
    const AD<double> &epsi0 = (psi0 - psides0) + v0 * delta0 / Lf * dt;
    fg[0] += epsi_weight * CppAD::pow(epsi0, 2);

    // Cross track error: We just use the y coordinate of the reference
    // polynomial to find the CTE. This is approximately right when both the
    // car's steering angle (psi) and the reference slope are not too steep.
    const AD<double> &reference_y =
      reference.coeffs[0] + x0 * (
        reference.coeffs[1] + x0 * (
          reference.coeffs[2] + x0 * (
            reference.coeffs[3]
          )
        )
      );
    const AD<double> &cte0 = (reference_y - y0) + v0 * CppAD::sin(epsi0) * dt;
    fg[0] += cte_weight * CppAD::pow(cte0, 2);

    // Speed: Just have to be careful of the units.
    fg[0] += v_weight * CppAD::pow(vars[v_start + i] -
      ref_v * MPH_TO_METERS_PER_SECOND, 2);

    // Actuators: Minimize the use of actuators.
    fg[0] += delta_weight * CppAD::pow(delta0, 2);
    fg[0] += throttle_weight * CppAD::pow(throttle0, 2);

    // Actuator smoothness: Minimize the value gap between sequential
    // actuations.
    if (i < N - 2) {
      fg[0] += delta_gap_weight *
        CppAD::pow(vars[delta_start + i + 1] - delta0, 2);
      fg[0] += throttle_gap_weight *
        CppAD::pow(vars[throttle_start + i + 1] - throttle0, 2);
    }
  }
}
