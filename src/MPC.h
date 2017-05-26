#ifndef MPC_H
#define MPC_H

#include <chrono>
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:
  typedef CPPAD_TESTVECTOR(double) Dvector;

  MPC();

  virtual ~MPC();

  // Time of last solve, if any.
  std::chrono::steady_clock::time_point t;

  // Time from last solve to current solve, in seconds.
  double latency;

  Dvector vars;
  Dvector vars_lowerbound;
  Dvector vars_upperbound;
  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;

  // Solve the model given an initial state and polynomial coefficients.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

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
