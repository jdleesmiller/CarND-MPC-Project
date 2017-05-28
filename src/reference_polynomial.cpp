#include "reference_polynomial.h"

#include <iostream>
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

void ReferencePolynomial::Update(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector,
  double px, double py, double psi)
{
  UpdatePointWeightMap(ptsx_vector, ptsy_vector);
  // std::cout << "UPDATED WEIGHT" << std::endl;

  size_t num_points = point_weight_map.size();
  vehicle_ptsx.resize(num_points);
  vehicle_ptsy.resize(num_points);
  point_weights.resize(num_points);
  size_t i = 0;
  for (auto it = point_weight_map.cbegin(); it != point_weight_map.cend(); ++it)
  {
    // std::cout << "RUN" << i << std::endl;
    double x = it->first.first - px;
    double y = it->first.second - py;
    double rx = x * cos(-psi) - y * sin(-psi);
    double ry = x * sin(-psi) + y * cos(-psi);
    vehicle_ptsx(i) = rx;
    vehicle_ptsy(i) = ry;
    point_weights(i) = it->second;
    ++i;
  }

  // std::cout << "PW" << std::endl << point_weights << std::endl;

  //
  // Transform the waypoints into vehicle coordinates, where the car is
  // at (0, 0) pointing along the x axis (psi = 0).
  //
  // size_t num_points = ptsx_vector.size();
  // vehicle_ptsx.resize(num_points);
  // vehicle_ptsy.resize(num_points);
  // for (size_t i = 0; i < num_points; ++i) {
  //   double x = ptsx_vector[i] - px;
  //   double y = ptsy_vector[i] - py;
  //   double rx = x * cos(-psi) - y * sin(-psi);
  //   double ry = x * sin(-psi) + y * cos(-psi);
  //   vehicle_ptsx(i) = rx;
  //   vehicle_ptsy(i) = ry;
  // }

  // fit a polynomial to the above x and y coordinates
  coeffs = polyfit(vehicle_ptsx, vehicle_ptsy, point_weights, DEGREE);
}

double ReferencePolynomial::Evaluate(double x) const {
  return polyeval(coeffs, x);
}

void ReferencePolynomial::UpdatePointWeightMap(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector)
{
  const double DELTA = 0.1;
  const double EPSILON = 1e-6;

  size_t num_points = ptsx_vector.size();
  for (size_t i = 0; i < num_points; ++i) {
    Point point(ptsx_vector[i], ptsy_vector[i]);
    auto point_it = point_weight_map.find(point);
    if (point_it == point_weight_map.end()) {
      point_weight_map.insert(std::pair<Point, double>(point, 2 * DELTA));
    } else if (point_it->second > 1 - EPSILON) {
      point_it->second = 1 + DELTA; // will subtract DELTA below
    } else {
      point_it->second += 2 * DELTA; // will subtract DELTA below
    }
  }

  for (auto it = point_weight_map.begin(); it != point_weight_map.end(); ) {
    it->second -= DELTA;
    if (it->second < EPSILON) {
      point_weight_map.erase(it++);
    } else {
      // std::cout << it->first.first << "," << it->first.second << ":" << it->second << std::endl;
      ++it;
    }
  }
}
