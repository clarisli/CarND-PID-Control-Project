#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

const double min_target_speed = 30;
const double adaptive_speed = 30;
const double steering_gain = 2.85;
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

double limit(double value) {
  double max = 1.0;
  double min = -1.0;
  if (value > max) {
    value = max;
  } else if (value < min) {
    value = min;
  }

  return value;
}

int main()
{
  uWS::Hub h;

  PID steer_pid;
  steer_pid.Init(0.114690, 0.000000, 2.045754);
  PID speed_pid;
  speed_pid.Init(0.121535, 0.006990, 0.942458);
  double speed_value = 0.0;

  // TODO: Initialize another pid for speed

  h.onMessage([&steer_pid, &speed_pid, &speed_value](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;
          /*
          * TCalcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          steer_pid.UpdateError(cte);
          steer_value = limit(steer_value - steer_pid.TotalError());
          steer_pid.Twiddle();


          //Calculate throttle value here, limit it to [-1, 1]
          double target_speed = adaptive_speed*(1.0 - steering_gain*fabs(steer_value)) + min_target_speed;
          double speed_error = speed - target_speed;
          speed_pid.UpdateError(speed_error);
          speed_value = limit(speed_value - speed_pid.TotalError());
          speed_pid.Twiddle();

          // DEBUG
          std::cout << "Steer PID Kp: " << steer_pid.Kp << ", Ki: " << steer_pid.Ki << ", Kd: " << steer_pid.Kd << std::endl;
          std::cout << "Speed PID Kp: " << speed_pid.Kp << ", Ki: " << speed_pid.Ki << ", Kd: " << speed_pid.Kd << std::endl;
          std::cout << "Steer CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "speed error: " << speed_error << " Speed Value: " << speed_value << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
