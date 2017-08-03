#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <cmath>
#include "optimizer.h"
#include <string>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

long long timestep = 0;

int main(int argc, char *argv[])
{
  uWS::Hub h;

  // Set to true if optimizer is used to tune PID coefficients
  const bool use_optimizer = true;

  // Default PID parameters
  //const double Kp = -0.09;
  //const double Ki = -0.0001;
  //const double Kd = -0.05;
  // Following coefficents should be quite close to optimal @30mph
  const double Kp = -0.1562;
  const double Ki = -3e-06;  // 0.000003;
  const double Kd = -0.05976;
  std::cout << "PID = " << Kp << " " << Ki << " " << Kd << std::endl;

  /* NOTE! PID and DP coefficients 2017.08.03 13:31 after 111 iterations
   * PID P=-0.119615 I=-3.50709e-06 D=-0.0427499
   * DP 0.00850834 1.70167e-06 0.00417682
   *
   * 120s intermediate results at loop 57
   * RMSE=0.635414
   * PID P=-0.158038 I=-3.21e-06 D=-0.05976
   * 0.00128637 5.54631e-08 0.00070307

   */

  PID pid_steer;
  Optimizer optimizer;
  if (use_optimizer) {
      pid_steer.Init(Kp, Ki, Kd);
      optimizer = Optimizer();
      optimizer.setPID(&pid_steer);
      optimizer.setPIDCoefficients(Kp, Ki, Kd);
      optimizer.setChangeCoefficients(std::abs(Kp*0.1), std::abs(Ki*0.1), std::abs(Kd*0.1));
      optimizer.setIterationTime(120);
      optimizer.setSkipTime(10);
  }
  else {
    pid_steer.Init(Kp, Ki, Kd);
  }


  h.onMessage([&pid_steer, &optimizer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Data from simulator
          const double cte = std::stod(j[1]["cte"].get<std::string>());
          const double speed = std::stod(j[1]["speed"].get<std::string>());
          const double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          const double throttle = std::stod(j[1]["throttle"].get<std::string>());

          double steer_value;

          double throttle_value = 1 / (1+cte);


          if (use_optimizer) {
              optimizer.Update(cte);
          }

          // Update error and calculate new steering value
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();

          // DEBUG
          //timestep++;
          //std::cout << pid.TotalRuntime() << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          if (use_optimizer) {
            if (optimizer.need_reset) {
                std::string msg = "42[\"reset\"]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                pid_steer.Reset();
                optimizer.need_reset = false;
                return;
              }
            else {
                // TODO: Solve duplication of steering message code
                json msgJson;
                msgJson["steering_angle"] = steer_value;
                msgJson["throttle"] = 0.3;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                //std::cout << msg << std::endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
          }
          else {
            // TODO: Solve duplication of steering message code
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


