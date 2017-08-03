#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

struct track{
    std::vector<double> x;
    std::vector<double> y;
};

/**
 * @brief ConvertPtsToLocalTrack
 * @param in the points making the input waypoints in global coordinate system
 * @param x current veh pos
 * @param y current veh po
 * @param psi current veh orientation
 * @return waypoints converted to vehicle coordinate system
 */
track ConvertPtsToLocalTrack(const track& in,double x,double y,double psi){
    track veh_coord_wp;

    Eigen::MatrixXd rot(2,2);
    rot << cos(-psi), -sin(-psi),
            sin(-psi), cos(-psi);
    Eigen::VectorXd veh(2);
    veh << x,y;
    for(size_t i = 0; i < in.x.size(); i++){
        Eigen::VectorXd wp(2);
        wp << in.x[i] , in.y[i];
        Eigen::VectorXd wp_veh_coord = rot*(wp - veh);
        veh_coord_wp.x.push_back(wp_veh_coord[0]);
        veh_coord_wp.y.push_back(wp_veh_coord[1]);
    }

    return veh_coord_wp;
}

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
// Evaluate the derivative of a polynomial
double polyDerivEval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i-1) * i;
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
          // All points here are given in world coordinate system
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
           // Fit a polynomial to the above x and y coordinates (path to follow)
          Eigen::VectorXd ptx = Eigen::Map<Eigen::VectorXd>(ptsx.data(),ptsx.size());
          Eigen::VectorXd pty = Eigen::Map<Eigen::VectorXd>(ptsy.data(),ptsy.size());
          auto coeffs = polyfit(ptx,pty,3) ;

          // calculate the cross track error
          auto y_out = polyeval(coeffs,px);
          double cte = y_out - py;
          // calculate the orientation error
          auto poly_deriv = polyDerivEval(coeffs,px);
          double epsi = psi - atan(poly_deriv);

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          auto vars = mpc.Solve(state,coeffs);
          double steer_value(vars[6]);
          double throttle_value(vars[7]);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals{vars[0],vars[2],vars[4]};
          vector<double> mpc_y_vals{vars[1],vars[3],vars[5]};

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          auto pred_track = ConvertPtsToLocalTrack({mpc_x_vals,mpc_y_vals},px,py,psi);

          msgJson["mpc_x"] = pred_track.x;
          msgJson["mpc_y"] = pred_track.y;

          //Display the waypoints/reference line
          track wp{ptsx,ptsy};
          auto ref_track = ConvertPtsToLocalTrack(wp,px,py,psi);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = ref_track.x;
          msgJson["next_y"] = ref_track.y;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(0));
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
