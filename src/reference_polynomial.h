#ifndef REFERENCE_POLYNOMIAL_H
#define REFERENCE_POLYNOMIAL_H

#include <map>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

/**
 * Maintain an estimate of the reference polynomial based on the waypoints.
 *
 * The reference polynomial is in the vehicle coordinate system.
 *
 * The waypoints are added and removed quite abruptly, so we use weighted least
 * squares to gradually increase the weight of new points and gradually decrease
 * the weight of old points. This avoids large changes in the reference
 * trajectory.
 */
struct ReferencePolynomial {
  ReferencePolynomial();

  /**
   * Forget known waypoints for a new run.
   */
  void Reset();

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
  typedef std::vector<Point> PointVector;

  PointVector points;
  Eigen::VectorXd point_weights;

  void UpdateKnownPoints(
    const std::vector<double> &ptsx_vector,
    const std::vector<double> &ptsy_vector);

  void TransformKnownPoints(double px, double py, double psi);

  bool PointIsPresent(const Point& point,
    const std::vector<double> &ptsx_vector,
    const std::vector<double> &ptsy_vector);

  bool PointIsKnown(const Point& point);
};

#endif /* REFERENCE_POLYNOMIAL_H */
