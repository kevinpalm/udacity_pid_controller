#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

int main()
{
  uWS::Hub h;

  PID pid;
  PID speed_pid;
  
  // Initialize the pid variable.
  pid.Init(-1.011, -1.0, 1.58459);
  speed_pid.Init(-2.1, -1.0, 1.0);
  
  /*********************************************************************
   * Twiddle Tuning Block 1 Start - Parameters
   ********************************************************************/
  // Switch the true/false here to decide if we're twiddle tuning
  bool twiddle_tuning = true;
  int timestamp = 0;
  int timesteps = 100;
  double tolerance = 0.01;
  std::vector<double*> params {&pid.Kp, &pid.Ki, &pid.Kd, &speed_pid.Kp, &speed_pid.Ki, &speed_pid.Kd};
  std::vector<double*> param_errors {&pid.p_error, &pid.i_error, &pid.d_error, &speed_pid.p_error, &speed_pid.i_error, &speed_pid.d_error};
  std::vector<double> best_tunings(6);
  bool first_round = true;
  double best_score;
  double avg_score = 0.0;
  int param_index = 5;
  int plus_minus_index = 0;
  /*********************************************************************
   * Twiddle Tuning Block 1 End
   ********************************************************************/

  h.onMessage([&pid, &speed_pid, &twiddle_tuning, &timestamp, &timesteps, &tolerance, &params, &param_errors, &best_tunings, &first_round, &best_score, &avg_score, &param_index, &plus_minus_index](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;

		  // Calculate the steering and throttle
          double throttle = speed_pid.OutputValue(fmax(0.0001, fabs(cte) * fabs(cte))/fmax(0.0001, speed/100));
          steer_value = pid.OutputValue(fmax(0.0001, fabs(cte) * fabs(cte))/fmax(0.0001, speed/100));
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          // increment the timestamp;
          timestamp += 1;

  /*********************************************************************
   * Twiddle Tuning Block 2 Start - Implimentation
   ********************************************************************/
          // do twiddle stuff if we're tuning
          if (twiddle_tuning) {
			  
			// Increment the timestamp and add the round's score to the round average
			timestamp ++;
			avg_score += (fmax(0.0001, fabs(cte) * fabs(cte))/fmax(0.0001, speed/100))/timesteps; // scoring on cte divided by scaled speed clipped to avoid div 0 and negatives
			  
			// Check if we've done enough timestamps
			if (timestamp >= timesteps) {
				
              //Check if this was the first round
              if (first_round) {
			    best_score = avg_score;
			    std::cout << "This round had the new best score of " << best_score << ", using the parameters of:" << std::endl;
			    for (int i=0; i<best_tunings.size(); i++) {
			      best_tunings[i] = *params[i];
			      std::cout << best_tunings[i] << ", " << std:: endl;
			    }
                first_round = false;
                
              } else {

				  // Check if we had improvements
				  if (avg_score < best_score) {

					  // Improvement in this direction, add momentum
					  best_score = avg_score;
					  std::cout << "This round had the new best score of " << best_score << ", using the parameters of:" << std::endl;
					  for (int i=0; i<best_tunings.size(); i++) {
						best_tunings[i] = *params[i];
						std::cout << best_tunings[i] << ", " << std:: endl;
					  }
					  *param_errors[param_index] *= 1.1;
					  plus_minus_index = 0;

					  } else {
						  // Check if we already tried switching directions (sub round)
						  if (plus_minus_index == 0) {
							  
							  // Haven't switched yet, try switching directions
							  std::cout << "No improvements, try switching the tuning direction" << std::endl;
							  *params[param_index] -= 2 * *param_errors[param_index];
							  plus_minus_index = 1;
							  
							  } else {
						
						// Also no improvement in this direction, we passed a minimum... reduce momentum
						std::cout << "Also no improvements, reduce momentum" << std::endl;
						*params[param_index] += *param_errors[param_index];
						*param_errors[param_index] *= 0.9;
						plus_minus_index = 0;
						
					  }
				   }
			   }
				 
			   //check if we need another round
			   double total_error = 0.0;
			   for (int i=0; i<param_errors.size(); i++) {
			     total_error += *param_errors[i];
			     
			   }
			   if (total_error > tolerance) {
				   
			     //prep for next round
			     if (plus_minus_index == 0) {
					 param_index = (param_index + 1) % params.size();
					 *params[param_index] += *param_errors[param_index];
				 }
				 std::cout << "Starting new round with tunings of:" << std::endl;
				 for (int i=0; i<params.size(); i++) {
			       std::cout << *params[i] << std::endl;
			     }
				 timestamp = 0;
				 avg_score = 0.0;
				 std::string reset_msg = "42[\"reset\",{}]";
                 ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
				 
  
			   }
             }
          }
  /*********************************************************************
   * Twiddle Tuning Block 2 End
   ********************************************************************/
   
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
