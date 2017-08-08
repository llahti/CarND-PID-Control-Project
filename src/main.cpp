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

  // Default steering control PID parameters which should work at 50mph
  const double Kp_steer = -0.055;
  const double Ki_steer = -0.0022;
  const double Kd_steer = -0.055;
  // default speed control PID parameters
  const double target_speed = 50.0;
  const double Kp_speed = 0.4;
  const double Ki_speed = 0.01;
  const double Kd_speed = 0.0;

  // Initialize PID controller for steering angle
  PID pid_steer = PID();
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer);
  // Initialize PID controller for speed controlling
  PID pid_speed = PID(target_speed, Kp_speed, Ki_speed, Kd_speed);



  // Optimizer is used only for steering
  Optimizer optimizer;
  if (use_optimizer) {
      //pid_steer.Init(Kp, Ki, Kd);
      optimizer = Optimizer(&pid_steer);
      optimizer.setChangeCoefficients(std::abs(Kp_steer*0.05), std::abs(Ki_steer*0.05), std::abs(Kd_steer*0.05));
      optimizer.setIterationTime(240);  // 240s more than 2 laps
      optimizer.setSkipTime(5);
  }

  std::cout << "Steering PID Coefficients are: " << Kp_steer << " " << Ki_steer << " " << Kd_steer << std::endl;
  std::cout << "Speed PID Coefficients are: " << Kp_speed << " " << Ki_speed << " " << Kd_speed << std::endl;


  h.onMessage([&pid_steer, &pid_speed, &optimizer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // If you need below variables then uncomment line
          const double speed = std::stod(j[1]["speed"].get<std::string>());
          //const double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          //const double throttle = std::stod(j[1]["throttle"].get<std::string>());

          double steer_value;

          // Throttle adjustment
          double throttle_value = pid_speed.Update(speed);

          // Update error and calculate new steering value
          if (use_optimizer) {
            optimizer.UpdateError(cte);
            steer_value = optimizer.TotalError();
          }
          else {
            pid_steer.UpdateError(cte);
            steer_value = pid_steer.TotalError();
            }



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
                msgJson["throttle"] = throttle_value;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                //std::cout << msg << std::endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
          }
          else {
            // TODO: Solve duplication of steering message code
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            // DEBUG
            //timestep++;
            //std::cout << pid_steer.TotalRuntime() << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;
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


