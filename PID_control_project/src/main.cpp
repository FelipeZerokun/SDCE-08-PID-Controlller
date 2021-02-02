#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  //pid.Init(0,0,0);
  // I found a question of how to tune the parameters using twiddle:
  // https://knowledge.udacity.com/questions/411699
  // I will create Twiddle files and use a Twiddle class

  // Due to my inexperience in C++, creating a class for Twiddle was very difficult for me.
  // For now, I will try different values for the PID gains (doing twiddle manually)

  // First, just P controller
  //pid.Init(1,0,0); // <------- The car wiggles a lot side to side
  //pid.Init(2.1,0,0); // <----- a bit less wiggle but still too inestable (fast response but overshoots)
  //pid.Init(3.31,0,0); //<----- much faster to get to middle lane, but overshoots. 
  //pid.Init(4.64,0,0); //<----- It is clear that higher P will reduce error faster, but makes the car inestable
  //pid.Init(0.1,0,0); //<-------- This has a much slower response, it has to be faster
  //pid.Init(0.9, 0, 0); //<-------- The lower the P, a more soft and slower path to the center of the lane
  //pid.Init(0.3,0,0); //<--------- Can't control the car totally, now I will try PD controller

  // Now, I will add the D controller for a more stable response
  //pid.Init(1,0,1); //<---------- way better, but overshoots
  //pid.Init(0.5,0,1); //<-------- Still overhsoots a lot
  //pid.Init(0.1,0,1);//<--------- This is my first try where the car is able to do a full lap. Wiggles a bit
  //pid.Init(0.05, 0, 1);//<------ Better control, but in curves is too close to the side.
  //pid.Init(0.01, 0, 1); //------  This is not good enough to keep the car in the middle
  //pid.Init(0.05, 0, 1.5); //<---- close, but still too close to edge in curves
  //pid.Init(0.05,0,2); //<---------Really, really close to a good stable response Time to get the I control

  // Finally, I will add the integral control for unexpected car control
  //pid.Init(1, 1, 1); //<------------- as Expected, not a good control
  //pid.Init(0.05, 1, 2); //<----------- the I control is making my car go off the road, I'll make it really low
  //pid.Init(0.05, 0.1, 2); //<---------- not off the road, but still bad control
  //pid.Init(0.05, 0.01, 2); //<---------- Still bad. The PD control was so close, so I will make the I value really really small
  //pid.Init(0.05, 0.0001, 2); //<--------- Almost there. I will try a bit lower i gain
  //pid.Init(0.05, 0.00005, 2); //<-------- the car stays almost to the right side. I think I need a higher P.
  //pid.Init(0.1, 0.00005, 1.5); //<------- Better, I will try with a lower i
  //pid.Init(0.1, 0.00001, 1.2); //<------- Really good response! I will play with the gains a bit
  //pid.Init(0.1, 0.00001, 2);//<---------- Also a good control. A bit more aggresive
  //pid.Init(0.5, 0.00001, 1.5); //<------- too mucho wiggle.
  //pid.Init(0.05, 0.00001, 0.5); //<-------- Good response, but too much to the edges.
  
  //In the end, This was the best control
  pid.Init(0.1, 0.00001, 1.2);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          bool twiddle_done = true;
          double throttle = 0.3;
          if(!twiddle_done)
          {
            std::cout<<"This is a test";

          }
          else
          {
            //Begin with the PID control
            // First, update the PID errors
            pid.UpdateError(cte);
            // And use the errors to calculate how much the car should steer to stay in lane
            steer_value = pid.TotalError();
            if(steer_value > 1)
            {
              steer_value = 1;
            }
            else if(steer_value < -1)
            {
              steer_value = -1;
            }
          }


          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; // <------------------------- Here I add the Throttle Variable
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
