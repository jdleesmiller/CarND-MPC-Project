#ifndef REFERENCE_POLYNOMIAL_H
#define REFERENCE_POLYNOMIAL_H

#include <map>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

/**
 * Maintain an estimate of the reference polynomial based on the waypoints.
 *
 * The reference polynomial is in the vehicle coordinate system.
 */
struct ReferencePolynomial {
  ReferencePolynomial();

  void Reset(); // TODO: clear map

  /**
   * Compute new coefficients and transformed points.
   */
  void Update(
    const std::vector<double> &ptsx_vector,
    const std::vector<double> &ptsy_vector,
    double px, double py, double psi);

  /**
   * Evaluate the polynomial at the given x coordinate.
   */
  double Evaluate(double x) const;

  // Coefficients of the estimated polynomial.
  Eigen::VectorXd coeffs;

  // The latest set of waypoints, transformed into vehicle coordinates.
  Eigen::VectorXd vehicle_ptsx;
  Eigen::VectorXd vehicle_ptsy;

private:
  typedef std::pair<double, double> Point;
  typedef std::map<Point, double> PointWeightMap;

  PointWeightMap point_weight_map;
  Eigen::VectorXd point_weights;

  void UpdatePointWeightMap(
    const std::vector<double> &ptsx_vector,
    const std::vector<double> &ptsy_vector);
};

#endif /* REFERENCE_POLYNOMIAL_H */
