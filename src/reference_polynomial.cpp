#include "reference_polynomial.h"

#include <algorithm>
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"

const int DEGREE = 3;

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd &coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(
  const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals,
  const Eigen::VectorXd &weights, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  Eigen::MatrixXd W = weights.array().sqrt().matrix().asDiagonal();

  auto Q = (W * A).householderQr();
  auto result = Q.solve(W * yvals);
  return result;
}

ReferencePolynomial::ReferencePolynomial() : coeffs(DEGREE + 1) { }

void ReferencePolynomial::Reset() {
  points.clear();
  point_weights.resize(0);
}

void ReferencePolynomial::Update(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector,
  double px, double py, double psi)
{
  UpdateKnownPoints(ptsx_vector, ptsy_vector);
  TransformKnownPoints(px, py, psi);
  coeffs = polyfit(vehicle_ptsx, vehicle_ptsy, point_weights, DEGREE);
}

double ReferencePolynomial::Evaluate(double x) const {
  return polyeval(coeffs, x);
}

void PopFrontVectorXd(Eigen::VectorXd &v) {
  if (v.size() == 0) return;
  for (size_t i = 1; i < v.size(); ++i) v(i - 1) = v(i);
  v.conservativeResize(v.size() - 1);
}

void PushBackVectorXd(Eigen::VectorXd &v, double value) {
  v.conservativeResize(v.size() + 1);
  v(v.size() - 1) = value;
}

//
// Implement gradual up-weighting of new points and down-weight of old points,
// to provide input weights for a for weighted least square fit. Note that
// this has to preserve the order of the points, so the simulator can display
// the reference line based on the transformed points. This means we can't use
// a datastructure like a set, but the number of waypoints is quite small
// anyway (typically 6), so it would probably not be worth it anyway.
//
void ReferencePolynomial::UpdateKnownPoints(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector)
{
  // Amount to increase or decrease the weight of a point by, per time step.
  const double DELTA = 0.1;

  // Numerical tolerance for comparisons to the 0 (no weight) and 1 (full
  // weight) boundaries.
  const double EPSILON = 1e-6;

  // Find whether old points are present in the set of new points. If they are
  // present, and they are not already at full weight (1), then up-weight them
  // by DELTA. If they are not present, down-weight them by DELTA.
  for (size_t i = 0; i < points.size(); ++i) {
    if (PointIsPresent(points[i], ptsx_vector, ptsy_vector)) {
      if (point_weights(i) < 1.0 - EPSILON) {
        point_weights(i) += DELTA;
      } else {
        point_weights(i) = 1.0;
      }
    } else {
      point_weights(i) -= DELTA;
    }
  }

  // Remove points with zero weight. We assume that these are the ones at the
  // start of the array, since that is where old points seem to disappear from.
  while (!points.empty() && point_weights(0) < 0 + EPSILON) {
    points.erase(points.begin());
    PopFrontVectorXd(point_weights);
  }

  // If any new points were added, append them with a small weight; we'll
  // increase the weight gradually.
  for (size_t i = 0; i < ptsx_vector.size(); ++i) {
    Point point(ptsx_vector[i], ptsy_vector[i]);
    if (!PointIsKnown(point)) {
      points.push_back(point);
      PushBackVectorXd(point_weights, DELTA);
    }
  }
}

bool ReferencePolynomial::PointIsPresent(const Point& point,
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector) {
  for (size_t i = 0; i < ptsx_vector.size(); ++i) {
    Point new_point(ptsx_vector[i], ptsy_vector[i]);
    if (point == new_point) {
      return true;
    }
  }
  return false;
}

bool ReferencePolynomial::PointIsKnown(const Point& point) {
  return std::find(points.begin(), points.end(), point) != points.end();
}

//
// Transform the waypoints into vehicle coordinates, where the car is
// at (0, 0) pointing along the x axis (psi = 0).
//
void ReferencePolynomial::TransformKnownPoints(double px, double py, double psi)
{
  size_t num_points = points.size();
  vehicle_ptsx.resize(num_points);
  vehicle_ptsy.resize(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    double x = points[i].first - px;
    double y = points[i].second - py;
    double rx = x * cos(-psi) - y * sin(-psi);
    double ry = x * sin(-psi) + y * cos(-psi);
    vehicle_ptsx(i) = rx;
    vehicle_ptsy(i) = ry;
  }
}
