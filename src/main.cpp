#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"
#include "matplotlibcpp.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
namespace plt = matplotlibcpp;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  int iters = 50;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          VectorXd yvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

          // Map to vehicle relative
          for (int i = 0; i < ptsx.size(); ++i) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            xvals[i] = x * cos(psi) + y * sin(psi);
            yvals[i] = -x * sin(psi) + y * cos(psi);
          }

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          auto coeffs = polyfit(xvals, yvals, 3);

          // calculate the cross track error
          double cte = -polyeval(coeffs, 0);
          // calculate the orientation error
          double epsi = -atan(coeffs[1]);

          VectorXd state(6);
          // Use vehicle as origin
          //state << px, py, psi, v, cte, epsi;
          state << 0, 0, 0, v, cte, epsi;

          vector<double> x_vals = {state[0]};
          vector<double> y_vals = {state[1]};
          vector<double> psi_vals = {state[2]};
          vector<double> v_vals = {state[3]};
          vector<double> cte_vals = {state[4]};
          vector<double> epsi_vals = {state[5]};
          vector<double> delta_vals = {};
          vector<double> a_vals = {};

          auto vars = mpc.Solve(state, coeffs);

          double steer_value;
          double throttle_value;

          // Positive means left in the simulator
          steer_value = -1 * vars[6];
          throttle_value = vars[7];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Green line
           */

          int N = (vars.size() - 8) / 2;
          mpc_x_vals.resize(N);
          copy(begin(vars) + 8, begin(vars) + 8 + N, begin(mpc_x_vals));
          mpc_y_vals.resize(N);
          copy(begin(vars) + 8 + N, begin(vars) + 8 + 2 * N, begin(mpc_y_vals));

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Yellow line
           */

          next_x_vals.resize(xvals.size());
          next_y_vals.resize(yvals.size());

          VectorXd::Map(&next_x_vals[0], xvals.size()) = xvals;
          VectorXd::Map(&next_y_vals[0], yvals.size()) = yvals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
