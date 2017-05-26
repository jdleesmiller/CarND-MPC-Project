#include "reference_polynomial.h"

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
  const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order)
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

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

ReferencePolynomial::ReferencePolynomial() : coeffs(DEGREE + 1) { }

void ReferencePolynomial::Update(
  const std::vector<double> &ptsx_vector,
  const std::vector<double> &ptsy_vector,
  double px, double py, double psi)
{
  //
  // Transform the waypoints into vehicle coordinates, where the car is
  // at (0, 0) pointing along the x axis (psi = 0).
  //
  size_t num_points = ptsx_vector.size();
  vehicle_ptsx.resize(num_points);
  vehicle_ptsy.resize(num_points);
  Eigen::Matrix3d transform;
  transform <<
    cos(psi), -sin(psi), px,
    sin(psi), cos(psi), py,
    0, 0, 1;
  Eigen::Matrix3d inverse_transform(transform.inverse());
  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector3d p;
    p << ptsx_vector[i], ptsy_vector[i], 1;
    p = inverse_transform * p;
    vehicle_ptsx(i) = p(0);
    vehicle_ptsy(i) = p(1);
  }

  // fit a polynomial to the above x and y coordinates
  coeffs = polyfit(vehicle_ptsx, vehicle_ptsy, DEGREE);
}

double ReferencePolynomial::Evaluate(double x) const {
  return polyeval(coeffs, x);
}
