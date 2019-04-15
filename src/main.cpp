#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.hpp"
#include "json.hpp"
#include <typeinfo>

// my headers
#include "path_planner.hpp"
#include "spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;
double ref_vel = 0;
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  BehaviouralPLanning::PathPlanner path_planner;
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

    path_planner.m_map_waypoints_x.push_back ( x );
    path_planner.m_map_waypoints_y.push_back ( y );
    path_planner.m_map_waypoints_s.push_back ( s );
    
  }

  
#ifdef _WIN32
  h.onMessage ( [ &map_waypoints_x , &map_waypoints_y , &map_waypoints_s ,
                &map_waypoints_dx , &map_waypoints_dy, &path_planner ]
                ( uWS::WebSocket<uWS::SERVER>* ws , char *data , size_t length ,
                  uWS::OpCode opCode )
  {
#else
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &path_planner ]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
#endif
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
         
          path_planner.process_data ( j [ 1 ] [ "x" ] ,
                                      j [ 1 ] [ "y" ] ,
                                      j [ 1 ] [ "s" ] ,
                                      j [ 1 ] [ "d" ] ,
                                      j [ 1 ] [ "yaw" ] ,
                                      j [ 1 ] [ "speed" ] ,
                                      j [ 1 ] [ "previous_path_x" ] , j [ 1 ] [ "previous_path_y" ] ,
                                      j [ 1 ] [ "end_path_s" ] , j [ 1 ] [ "end_path_d" ] ,
                                      j [ 1 ] [ "sensor_fusion" ] );
          json msgJson;

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          ///**
          // * TODO: define a path made up of (x,y) points that the car will visit
          // *   sequentially every .02 seconds
          // */
         
          //tk::spline fitter;
          //int lane = getLaneId(car_d);
          ////cosntants
          //const double FRAME_RATE = 0.02;
          //const double MPH_TO_MTSPS = 1.0/2.24; // factor to transform miles per hour to meters per second
          //const double CLOSE_RANGE = 15; // expressed in meters
          //const double MAX_SPEED = 49.5; // expressed in miles per hour
          //const double SPEED_INCREMENT = 0.224; // expressed in miles per hour
          //

          //size_t prev_size = previous_path_x.size ( );
          //if ( prev_size > 0 )
          //{
          //    car_s = end_path_s;
          //}

          //bool other_car_too_close = false;
          //std::vector<std::vector <double>> sens = sensor_fusion;
          //// check if other cars are close to our car
          //for ( size_t i = 0; i < sensor_fusion.size(); i++ )
          //{
          //    double d = sensor_fusion [ i ] [ 6 ];
          //    if (isCarInLane(d,lane))
          //    {
          //        // if car is in my lane check its speed
          //        double vx = sensor_fusion [ i ] [ 3 ];
          //        double vy = sensor_fusion [ i ] [ 4 ];
          //        double linear_speed = sqrt ( ( vx*vx ) + ( vy*vy ) )* MPH_TO_MTSPS;
          //        // get car position in the lane
          //        double other_car_s = sensor_fusion [ i ] [ 5];
          //        // compensate through time and preditc car_s
          //        other_car_s +=  double ( prev_size ) * FRAME_RATE * linear_speed ;
          //        // if car is closer thatn a certain threshold 
          //        if (other_car_s > car_s && ((other_car_s - car_s) < CLOSE_RANGE ))
          //        {
          //            // set ref vel to a lower value
          //            other_car_too_close = true;
          //            if (lane > 0)
          //            {
          //                lane = 0;
          //            }
          //        }

          //    }
          //}

          //if (other_car_too_close)
          //{
          //    ref_vel -= SPEED_INCREMENT;
          //}
          //else if ( ref_vel < MAX_SPEED)
          //{
          //    ref_vel += SPEED_INCREMENT;
          //}


          //std::vector<double> ptsx;
          //std::vector<double> ptsy;

          //double ref_car_x = car_x;
          //double ref_car_y = car_y;
          //double ref_car_yaw = deg2rad( car_yaw);

          //if ( prev_size < 2)
          //{
          //    double prev_carx = car_x - std::cos ( ref_car_yaw );
          //    double prev_cary = car_y - std::sin ( ref_car_yaw );

          //    ptsx.push_back (prev_carx );
          //    ptsx.push_back ( car_x );

          //    ptsy.push_back ( prev_cary );
          //    ptsy.push_back ( car_y );
          //}
          //else
          //{
          //    ref_car_x = previous_path_x [ prev_size - 1 ];
          //    ref_car_y = previous_path_y [ prev_size - 1 ];

          //    double ref_car_prev_x = previous_path_x [ prev_size - 2 ];
          //    double ref_car_prev_y = previous_path_y [ prev_size - 2 ];
          //    ref_car_yaw = std::atan2 (ref_car_y - ref_car_prev_y,  ref_car_x - ref_car_prev_x );

          //    ptsx.push_back ( ref_car_prev_x );
          //    ptsx.push_back ( ref_car_x );

          //    ptsy.push_back ( ref_car_prev_y );
          //    ptsy.push_back ( ref_car_y );
          //}

          //vector<double> wp0 = getXY ( car_s + 30 , ( 2 + 4 * lane ) , map_waypoints_s , map_waypoints_x , map_waypoints_y );
          //vector<double> wp1 = getXY ( car_s + 60 , ( 2 + 4 * lane ) , map_waypoints_s , map_waypoints_x , map_waypoints_y );
          //vector<double> wp2 = getXY ( car_s + 90 , ( 2 + 4 * lane ) , map_waypoints_s , map_waypoints_x , map_waypoints_y );

          //ptsx.push_back ( wp0 [ 0 ] );
          //ptsx.push_back ( wp1 [ 0 ] );
          //ptsx.push_back ( wp2 [ 0 ] );

          //ptsy.push_back ( wp0 [ 1 ] );
          //ptsy.push_back ( wp1 [ 1 ] );
          //ptsy.push_back ( wp2 [ 1 ] );

          //// transform points from map frame to car frame
          //for ( size_t i = 0; i < ptsx.size ( ); i++ )
          //{
          //    double tempx = ptsx [ i ] - ref_car_x;
          //    double tempy = ptsy [ i ] - ref_car_y;

          //    ptsx [ i ] = tempx * cos ( -ref_car_yaw ) - tempy * sin ( -ref_car_yaw );
          //    ptsy [ i ] = tempx * sin ( -ref_car_yaw ) + tempy * cos ( -ref_car_yaw );
          //}

          //fitter.set_points (ptsx, ptsy );

          //// insert previous path points into the curren trajectory
          //next_x_vals.insert ( next_x_vals.end ( ) , previous_path_x.begin ( ) , previous_path_x.end ( ) );
          //next_y_vals.insert ( next_y_vals.end ( ) , previous_path_y.begin ( ) , previous_path_y.end ( ) );

          ////  compute target points from fitted polynomial
          //double target_x = 30;
          //double target_y = fitter ( target_x);
          //
          //double target_dist = std::sqrt ((target_x*target_x) + (target_y*target_y) );
          //double N = target_dist / ( FRAME_RATE*ref_vel*MPH_TO_MTSPS );
          //double dist_increment = target_x / N;
          //double x_offset = 0;
          //for ( size_t i = 0; i < 50 - previous_path_x.size(); i++ )
          //{
          //    // point in local vehicle coordinate syste
          //    double x_point_vcs = x_offset + dist_increment;
          //    double y_point_vcs = fitter (x_point_vcs );
          //    x_offset = x_point_vcs; /*save for next iteration*/

          //    // point in map coordinate system
          //    double x_point_mcs = ( x_point_vcs*cos ( ref_car_yaw ) ) - ( y_point_vcs * sin(ref_car_yaw) ) + ref_car_x;
          //    double y_point_mcs = ( x_point_vcs*sin ( ref_car_yaw ) ) + ( y_point_vcs * cos ( ref_car_yaw ) ) + ref_car_y;

          //    // add point to the path sent to sim
          //    next_x_vals.push_back ( x_point_mcs );
          //    next_y_vals.push_back ( y_point_mcs );

          //}

          
          msgJson["next_x"] = path_planner.m_trajectory.next_ptsx;
          msgJson["next_y"] = path_planner.m_trajectory.next_ptsy;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
#ifdef _WIN32
          ws->send ( msg.data ( ) , msg.length ( ) , uWS::OpCode::TEXT );
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef _WIN32
        ws->send ( msg.data ( ) , msg.length ( ) , uWS::OpCode::TEXT );
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }  // end websocket if
  }); // end h.onMessage
#ifdef _WIN32
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
#else
  h.onConnection ( [ &h ]( uWS::WebSocket<uWS::SERVER> ws , uWS::HttpRequest req )
  {
      std::cout << "Connected!!!" << std::endl;
  } );
#endif


#ifdef _WIN32
  h.onDisconnection ( [ &h ]( uWS::WebSocket<uWS::SERVER>* ws , int code ,
                              char *message , size_t length )
  {
      ws->close ( );
      std::cout << "Disconnected" << std::endl;
  } );
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
#endif
  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen( host , port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}