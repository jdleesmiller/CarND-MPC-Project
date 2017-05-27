#ifndef MPC_H
#define MPC_H

#include <chrono>
#include <vector>
#include <cppad/cppad.hpp>

#include "problem.h"
#include "reference_polynomial.h"

using namespace std;

class MPC {
public:
  typedef CPPAD_TESTVECTOR(double) Dvector;

  MPC(ReferencePolynomial &reference, Problem &problem);

  virtual ~MPC();

  ReferencePolynomial &reference;
  Problem &problem;

  // Time of last solve, if any.
  std::chrono::steady_clock::time_point t;

  // Time from last solve to current solve, in seconds.
  double latency;

  // Called upon a new connection.
  void Reset();

  // Called each time we receive telemetry to do the solve.
  void Update(
    const std::vector<double> &ptsx_vector,
    const std::vector<double> &ptsy_vector,
    double px, double py, double psi, double v);

  Dvector vars;
  Dvector vars_lowerbound;
  Dvector vars_upperbound;
  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;

  // The steering angle from the latest solve, in [-1, 1].
  double steer() const;

  // The throttle from the latest solve, in [-1, 1].
  double throttle() const;

  // Get the x values from the latest solve (vehicle coordinates).
  std::vector<double> x_values() const;

  // Get the y values from the latest solve (vehicle coordinates).
  std::vector<double> y_values() const;

  // Get the psi values from the latest solve (vehicle coordinates).
  std::vector<double> psi_values() const;

  // Get the v (speed) values from the latest solve.
  std::vector<double> v_values() const;

  // Get the CTE (cross track error) values from the latest solve.
  std::vector<double> cte_values() const;

  // Get the epsi (orientation error) values from the latest solve.
  std::vector<double> epsi_values() const;

  // Get the delta (steering control) values from the latest solve.
  std::vector<double> delta_values() const;

  // Get the a (throttle control) values from the latest solve.
  std::vector<double> a_values() const;

private:
  std::vector<double> get_variable(size_t start, size_t count) const;
};

#endif /* MPC_H */
