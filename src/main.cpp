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
#include <ctime>
#include <ratio>
#include <chrono>
#include <fstream>

using namespace std; 
using namespace chrono;

// create output file to track processing time
fstream fout;

// set to true to print out processing time of MPC
bool write_time_data = false; 

// set variables for time
// time from http://www.cplusplus.com/reference/chrono/steady_clock/
chrono::steady_clock::time_point t0, t1;

//Variable to hold steering Angle and accel values at t+1, 
// used for latency calculations
double delta_1 = 0; 
double a_1 = 0; 

// Variables to handle kinematic calc for latency
// dt is time step and also cooresponds to simulator latency 100 ms
double dt_ = 0.15;
double Lf_ = 2.67;

// for convenience
using json = nlohmann::json;

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

if(write_time_data){
		fout.open("./mpc_t.dat",ios::out);
		if(!fout.is_open()){
			cout << "file not open, cannot write." << endl << endl;
			write_time_data = true;
		}
	}


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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];


          //*** Calculate steering angle and throttle using MPC. Both normalized between [-1, 1]. ***

          double steer_value;
          double throttle_value;

					//*** Shift Waypt coordinates into vehcile coordinates and 3rd degree ploynomial to pts 
					Eigen::VectorXd ptsx_Eig = Eigen::VectorXd(ptsx.size());
					Eigen::VectorXd ptsy_Eig = Eigen::VectorXd(ptsy.size());

					for(unsigned int i = 0; i< ptsx.size(); i++){
						double shift_x = ptsx[i] - px;
						double shift_y = ptsy[i] - py;

						ptsx_Eig[i] = (shift_x * cos(-psi) - shift_y * sin(-psi));
						ptsy_Eig[i] = (shift_x * sin(-psi) + shift_y * cos(-psi));
 					}

					// calculate polynomial coeff for path
					auto coeff = polyfit(ptsx_Eig,ptsy_Eig,3);


					//*** calc cross-track error  and psi error ***//
					// cte is y - f(x), desired - trajectory evaluated at car position, px=0
					double cte = polyeval(coeff, 0); // ==> double cte = coeff[0]; 
							
					// calc psi error, [angle of vehicle] - [desired angle (straight line)], at px = 0, py = 0
					double epsi = -atan(coeff[1]); 

//debug
//cout << "Coeff:" << endl << coeff << endl << endl;
//cout << "cte: " << cte << endl << "epsi: " << epsi << endl;

					

// For Latency: Calculating future state t+1, and passing values to "curr" state to apply to solver
//	refered to https://carnd.slack.com/archives/C54DV4BK6/p1538209080000100 for state t+1 
// 		idea
// Also used https://github.com/jinchaolu/CarND-Term2-P5-MPC-Project/blob/ikc-solution/src/main.cpp 
//		as a reference
					double x0 = 0; 
					double y0 = 0; 
					double psi0 = 0;
					double x1 = 0; 
					double y1 = 0; 
					double psi1 = 0; 
					double v1=v; 
					double cte1 = 0; 
					double epsi1 = 0; 

					x1 = x0 + v* cos(psi0)*dt_;
					y1 = y0 + v * sin(psi0)* dt_;
					psi1 = psi0 - v/Lf_* delta_1 * dt_;
					v1 = v + a_1* dt_;					
					cte1 = cte + v * sin(epsi)* dt_;
					epsi1 = epsi - v/Lf_ * delta_1 * dt_; 
					
				
					// fill state vector with current state and pass to mpc solver
					Eigen::VectorXd state = Eigen::VectorXd(6);
					state << x1, y1, psi1, v1, cte1, epsi1;
					
					// get time duration of mpc solver
					vector<double> solved_var; 
					if(write_time_data){
						t0 = steady_clock::now();
						solved_var = mpc.Solve(state, coeff);
						t1 = steady_clock::now();
						duration<double> elapsed_t = duration_cast<duration<double>>(t1 - t0);
						fout << elapsed_t.count() << endl;
					}else{
						solved_var = mpc.Solve(state, coeff);
					}

					// set steer angle and acc for use in calculating,  next time step for latency handling
					delta_1 = solved_var[0];
					a_1 = solved_var[1];

					// extract steering angle and throttle vals and pass to simulator
					steer_value = solved_var[0];
					throttle_value = solved_var[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals ;
          vector<double> mpc_y_vals ;

					// taken from https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be , t= 13:02

					for(unsigned int i = 2; i < solved_var.size(); i++){
						if(i%2 == 0){
							mpc_x_vals.push_back(solved_var[i]);			
						}else{
							mpc_y_vals.push_back(solved_var[i]);			
						}

					}	

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
					
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals ;
          vector<double> next_y_vals;

					// taken from https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be , t= 10:16
					double poly_inc = 2.5;
					int n_pts = 10; 
					for(int i = 1; i< n_pts; i++){
						next_x_vals.push_back(poly_inc *i);
						next_y_vals.push_back(polyeval(coeff,poly_inc *i));
					}


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
