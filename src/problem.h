#ifndef PROBLEM_H
#define PROBLEM_H

#include <cppad/cppad.hpp>
#include "reference_polynomial.h"

// Number of time steps in the receding horizon problem.
extern const size_t N;

// Number of variables (N timesteps => N - 1 actuations).
extern const size_t N_VARS;

// Number of constraints.
extern const size_t N_CONSTRAINTS;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
extern const size_t x_start;
extern const size_t y_start;
extern const size_t psi_start;
extern const size_t v_start;
extern const size_t cte_start;
extern const size_t epsi_start;
extern const size_t delta_start;
extern const size_t throttle_start;

// Length from front to CoG that has a similar radius.
extern const double Lf;

// 1609.34m / mile * 1h / 3600s = x (m / s) / (miles / h).
extern const double MPH_TO_METERS_PER_SECOND;

/**
 * Convert a throttle value to an acceleration, based on current speed. This
 * is an empirical formula based on recording the speed under full throttle
 * at the start with no steering (before the vehicle crashes); see
 * data/acceleration_estimate.xlsx for details.
 *
 * @param  throttle in [-1, 1]
 * @param  speed in m/s
 * @return in m/s^2
 */
template <typename T>
T throttle_to_acceleration(T throttle, T speed) {
  return throttle * (5.1886 - 0.0923 * speed);
}

/**
 * Functor to calculate the objective function and set up the the dynamic
 * constraints.
 */
struct Problem {
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

  const ReferencePolynomial &reference;
  double dt;
  double ref_v;
  double cte_weight;
  double epsi_weight;
  double v_weight;
  double delta_weight;
  double throttle_weight;
  double delta_gap_weight;
  double throttle_gap_weight;

  Problem(const ReferencePolynomial &reference);

  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars);
};

#endif /* PROBLEM_H */
