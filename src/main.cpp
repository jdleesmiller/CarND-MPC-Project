#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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

std::ostream &operator<<(std::ostream &os, const std::vector<double> v) {
  for (auto it = v.begin(); it != v.end(); ++it) {
    os << " " << *it;
  }
  return os;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx_vector = j[1]["ptsx"];
          vector<double> ptsy_vector = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // Eigen::Map<Eigen::VectorXd> ptsx(
          //   ptsx_vector.data(), ptsx_vector.size());
          // Eigen::Map<Eigen::VectorXd> ptsy(
          //   ptsy_vector.data(), ptsy_vector.size());

          //
          // Transform the waypoints into vehicle coordinates, where the car is
          // at (0, 0) pointing along the x axis (psi = 0).
          //
          size_t num_points = ptsx_vector.size();
          Eigen::VectorXd ptsx(num_points);
          Eigen::VectorXd ptsy(num_points);
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
            ptsx(i) = p(0);
            ptsy(i) = p(1);
          }

          // fit a polynomial to the above x and y coordinates
          Eigen::VectorXd coeffs = polyfit(ptsx, ptsy, 3);

          // TODO calculate the cross track error
          double cte = coeffs[0];
          // calculate the orientation error
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          mpc.Solve(state, coeffs);

          // std::cout << "x =" << mpc.x_values() << std::endl;
          // std::cout << "y =" << mpc.y_values() << std::endl;
          // std::cout << "psi =" << mpc.psi_values() << std::endl;
          // std::cout << "v =" << mpc.v_values() << std::endl;

          // std::cout << "cte =" << mpc.cte_values() << std::endl;
          // std::cout << "epsi =" << mpc.epsi_values() << std::endl;

          // std::cout << "delta =" << mpc.delta_values() << std::endl;
          // std::cout << "a =" << mpc.a_values() << std::endl;

          /*
          * Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          */
          // steer right: angle positive
          // steer left: angle negative
          json msgJson;
          msgJson["steering_angle"] = -mpc.steer();
          msgJson["throttle"] = mpc.throttle();

          // Display the MPC predicted trajectory.
          // The points are in reference to the vehicle's coordinate system.
          // The points in the simulator are connected by a Green line.
          msgJson["mpc_x"] = mpc.x_values();
          msgJson["mpc_y"] = mpc.y_values();

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (size_t i = 0; i < num_points; ++i) {
            next_x_vals.push_back(ptsx(i));
            next_y_vals.push_back(ptsy(i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          // this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
