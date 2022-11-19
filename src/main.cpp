#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  //start in lane 1;
  int lane = 1;
  
  //speed limit
  double ref_vel = 0.0; //mph;

  double speed_diff = 0.5;
  const double max_speed = 49;
  

  h.onMessage([&max_speed,&speed_diff,&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event                
    
                
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          int prev_size = previous_path_x.size(); 

          //PREDICTION PART
          //==========================================================================================

          if(prev_size>0){
            car_s = end_path_s;
          }

          bool car_left = false;
          bool car_ahead = false;
          bool car_right = false;
          
          for(int i=0;i<sensor_fusion.size();i++){

            float d = sensor_fusion[i][6];
            int check_car_lane = -1; 

            if(d>0 && d<4){
              check_car_lane = 0;
            }

            else if(d>4 && d<8){
              check_car_lane = 1;
            }

            else if(d>8){
              check_car_lane = 2;
            }

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*.02*check_speed); //predict where will the other car will be

            if (check_car_lane == lane){
              if(check_car_s>car_s && check_car_s-car_s<30){
                car_ahead = true;
              }               
            }

            else if (check_car_lane - lane == -1){
              if(check_car_s>car_s-30 && check_car_s<car_s+30){
                car_left = true;
              }
            }

            else if (check_car_lane - lane == 1){
              if(check_car_s>car_s-30 && check_car_s<car_s+30){
                car_right = true;
              }
            } 
          }    
          //===========================================================================
          //END PREDICTION


          //BEHAVIOR
          //===========================================================================
          if(car_ahead == true){ //there is a slow car ahead
            if(lane > 0 && car_left == false){ //if ego car is not on lane 0  and left lane has not car nearby
              lane = lane - 1;  //change lane to left
            }
            
            else if(lane != 2 && car_right == false){
              lane = lane + 1;
            }

            else if(lane == 1 && car_left == false){
              lane = lane - 1;
            }
            else if(lane == 1 && car_right == false){
              lane = lane + 1;
            }

            else{
              ref_vel = ref_vel - speed_diff;
            }
          }

          else if(ref_vel < max_speed){
              ref_vel = ref_vel + speed_diff;
          }

          //============================================================================
          //END BEHAVIOR

          //TRAJECTORY GENERATION
          //============================================================================
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
				  double ref_yaw = deg2rad(car_yaw);
          
          //check if previous path have enough points
          //if not, use current car_x and car_y to calculate previous point
          if( prev_size < 2 ){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);         
          }
          
          //if previous path have enough points          
          else{
            //get the last 1 and 2 point as reference
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          //add some points to generate spline          
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          //transform points to car coordiante        
          for ( int i = 0; i < ptsx.size(); i++ ) {            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          //creat a spline;
          tk::spline s;
          
          //add points to spline
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;   
          
          //get the rest of the points from previous path to next path
          //reminder: previous_path dose NOT always contain 50 points, it is the previous path points that the simulator hasn't consume
          
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
           }
          
          //get target points(end points) from spline s          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          for( int i = 1; i < 50 - prev_size; i++ ) {
            
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //transform from car coordinate to map coordinate            
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          } 

          //========================================================================
          //END TRAJECTORY
          
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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