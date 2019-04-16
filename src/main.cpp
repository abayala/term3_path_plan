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
int main() {
  uWS::Hub h;

  //create object that will handle all path planning tasks
  BehaviouralPLanning::PathPlanner path_planner;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
 

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
    path_planner.m_map_waypoints_x.push_back ( x );
    path_planner.m_map_waypoints_y.push_back ( y );
    path_planner.m_map_waypoints_s.push_back ( s );
    
  }

  
#ifdef _WIN32
  h.onMessage ( [  &path_planner ]
                ( uWS::WebSocket<uWS::SERVER>* ws , char *data , size_t length ,
                  uWS::OpCode opCode )
  {
#else
  h.onMessage([ &path_planner]
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
         
            double  car_s = j[ 1 ][ "s" ];
            double end_s = j[ 1 ][ "end_path_s" ];

          
         // Process input data
          path_planner.process_data ( j [ 1 ] [ "x" ] ,
                                      j [ 1 ] [ "y" ] ,
                                      car_s,
                                      j [ 1 ] [ "d" ] ,
                                      j [ 1 ] [ "yaw" ] ,
                                      j [ 1 ] [ "speed" ] ,
                                      j [ 1 ] [ "previous_path_x" ] , j [ 1 ] [ "previous_path_y" ] ,
                                      end_s , j [ 1 ] [ "end_path_d" ] ,
                                      j [ 1 ] [ "sensor_fusion" ] );
          json msgJson;

          // Send output message
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