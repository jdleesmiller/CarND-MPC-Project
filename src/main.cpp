#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <sysexits.h>
#include <thread>
#include <vector>
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// Use this code when closing the socket after we detect that the car has
// crashed; this lets the server know that it was closed intentionally, rather
// than due to a network / simulator crashing problem.
const int CAR_CRASHED_CODE = 2000;
const int MAX_RUNTIME_CODE = 2001;

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

std::ostream &operator<<(std::ostream &os, const std::vector<double> v) {
  for (auto it = v.begin(); it != v.end(); ++it) {
    os << " " << *it;
  }
  return os;
}

int main(int argc, char **argv) {
  uWS::Hub h;

  ReferencePolynomial reference;
  Problem problem(reference);
  MPC mpc(reference, problem);

  double max_runtime = 24 * 3600;

  if (argc == 11) {
    mpc.tuning = true;
    max_runtime = atof(argv[1]);
    problem.dt = atof(argv[2]);
    problem.ref_v = atof(argv[3]);
    problem.cte_weight = atof(argv[4]);
    problem.epsi_weight = atof(argv[5]);
    problem.v_weight = atof(argv[6]);
    problem.delta_weight = atof(argv[7]);
    problem.throttle_weight = atof(argv[8]);
    problem.delta_gap_weight = atof(argv[9]);
    problem.throttle_gap_weight = atof(argv[10]);
  }

  h.onMessage([&mpc, max_runtime](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          mpc.Update(
            j[1]["ptsx"],
            j[1]["ptsy"],
            j[1]["x"],
            j[1]["y"],
            j[1]["psi"],
            j[1]["speed"],
            j[1]["steering_angle"],
            j[1]["throttle"]
          );

          if (mpc.tuning && mpc.crashed) {
            std::cout << mpc << std::endl;
            ws.close(CAR_CRASHED_CODE);
            return;
          }

          // If we've run all the way to the deadline, stop.
          if (mpc.tuning && mpc.runtime > max_runtime) {
            std::cout << mpc << std::endl;
            ws.close(MAX_RUNTIME_CODE);
            return;
          }

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
          for (size_t i = 0; i < mpc.reference.vehicle_ptsx.size(); ++i) {
            next_x_vals.push_back(mpc.reference.vehicle_ptsx(i));
            next_y_vals.push_back(
              mpc.reference.Evaluate(mpc.reference.vehicle_ptsx(i)));
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

  h.onConnection([&h, &mpc](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    if (!mpc.tuning) {
      std::cout << "Connected!!!" << std::endl;
    }
    mpc.Reset();
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    switch (code) {
      case CAR_CRASHED_CODE:
        // The car crashed; let the caller know.
        exit(1);
      case MAX_RUNTIME_CODE:
        // The simulator ran until our deadline; that's a success.
        exit(EX_OK);
      default:
        // If the simulator exits, we seem to get code 1006 or 0.
        std::cerr << "Disconnected: code=" << code << ":" <<
          std::string(message, length) << std::endl;
        exit(EX_UNAVAILABLE);
    }
  });

  int port = 4567;
  if (h.listen(port)) {
    if (!mpc.tuning) {
      std::cout << "Listening to port " << port << std::endl;
    }
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
