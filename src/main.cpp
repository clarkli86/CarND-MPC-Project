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

          x_vals.push_back(vars[0]);
          y_vals.push_back(vars[1]);
          psi_vals.push_back(vars[2]);
          v_vals.push_back(vars[3]);
          cte_vals.push_back(vars[4]);
          epsi_vals.push_back(vars[5]);

          delta_vals.push_back(vars[6]);
          a_vals.push_back(vars[7]);
          /*
          cout << "x = " << vars[0] << endl;
          cout << "y = " << vars[1] << endl;
          cout << "psi = " << vars[2] << endl;
          cout << "v = " << vars[3] << endl;
          cout << "cte = " << vars[4] << endl;
          cout << "epsi = " << vars[5] << endl;
          cout << "delta = " << vars[6] << endl;
          cout << "a = " << vars[7] << endl;
          cout << endl;
          */
          // Plot values
          // NOTE: feel free to play around with this.
          // It's useful for debugging!
          /*
          plt::subplot(3, 1, 1);
          plt::title("CTE");
          plt::plot(cte_vals);
          plt::subplot(3, 1, 2);
          plt::title("Delta (Radians)");
          plt::plot(delta_vals);
          plt::subplot(3, 1, 3);
          plt::title("Velocity");
          plt::plot(v_vals);

          plt::show();
          */

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

          //int N =
          //for (int i = 0; i <
          //mpc_x_vals =

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

          next_x_vals = ptsx;
          next_y_vals = ptsy;
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
