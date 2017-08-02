#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
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
  const double Kp = -0.09;
  const double Ki = -0.0001;
  const double Kd = -0.05;

  PID pid;
  Optimizer optimizer;
  if (use_optimizer) {
      pid.Init(Kp, Ki, Kd);
      optimizer = Optimizer();
      optimizer.setPID(&pid);
      optimizer.setPIDCoefficients(Kp, Ki, Kd);
      optimizer.setChangeCoefficients(0.001, 0.000001, 0.001);
      optimizer.setSkipTime(5);
      optimizer.setIterationTime(120);
  }
  else {
    pid.Init(Kp, Ki, Kd);
  }


  h.onMessage([&pid, &optimizer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          if (use_optimizer) {
              optimizer.Update(cte);
          }

          // Update error and calculate new steering value
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          // DEBUG
          //timestep++;
          //std::cout << pid.TotalRuntime() << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          if (use_optimizer) {
            if (optimizer.need_reset) {
                std::string msg = "42[\"reset\"]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                pid.Reset();
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


