# Path Planning Project - Term 3

## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Provided Information

A sparse map list of waypoints around the highway is provided in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

The car's localization and sensor fusion data is provided, which is received in a message and described below:

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Development

### Explanation of Logic Used
To tackle this problem I decided to implement a Finite State Machine (FSM) that consists of 3 states :

* Keep Lane (KL):
    - Description: In this state the car must stay in the current lane
    - Possible states to change to : KL,PLC
    - Conditions to change state : If a car in the current lane is closer than 30 meters then the system shall change to Prepare Lane Change state, otherwise keep in KL.

* Prepare Lane Change (PLC) :
    - Description : This state is in charge of looking at other vehicles in the right and left lane, this with the purpose to determine if is possible to change lane safely and choose the best lane to change.
    - Possible states to change: KL,LC
    - Conditions to change state: If is possible to move to the right or to the left lane, check in the available lanes if there are no cars 30 mts in front of my position and 30 mts behind my position. If one of the lane fits the criteria, set the lane as target lane for Lane Change state. If no lane fits criteria, move to KL state
                
* Lane Change (LC):
    - Description : This state will be in charged of monitoring and performing a lane change to the target lane defined in PLC
    - Possible state to change : KL, LC 
    - Conditions to change state : If the car is less than 0.5 mts from the center of target lane, then it shall change to KL, otherwise continue in LC state
 
 
 The reference velocity is monitored at all times, if needed is modified to avoid hitting cars and maintain it the correct limits.  
 To avoid overpassing the allowed threshold for max acceleration and jerk, increments of 0.224 are added to the reference velocity till the max limit ( MAX_DESIRED_SPEED = 49.5 mph) is reached.
 When a car is too close in my current lane in front of me (less than 30 mts), then the reference velocity is decreased proportionally to my current speed using the next formula with a max magnitude of 0.224 mph

~~~~
 m_ref_velocity -= SPEED_INCREMENT *( m_ref_velocity / MAX_DESIRED_SPEED );
~~~~ 
 
 The closest the ref_velocity is to the max MAX_DESIRED_SPEED it will be decremented more, the slower we travel the less velocity is decremented. This approach helps to maximize the time we go at faster speeds and to not slow down too much when we want to change lanes.
 
 ### Explanation of implementation
 
 The FSM implementation is contained in the class called PathPlanner inside the namespace BehaviouralPLanning, this can be found in the files src/path_planner.hpp and src/path_planner.cpp
 
 The main entry point which is called in main.cpp  is PathPlanner::process_data(...). This function receives as input all the simulator data and if necessary stores it in class member variables. With the vehicle d position it computes the current lane id, to then process the sensor fusion data and determine if there are cars too close in the current lane. 
 The velocity in vx and vy from the other cars is used to compute the linear speed and compensate their displacement in S relative to the refresh rate of the data and the previous path elements. 
 If any of the cars is within the range of less than 30 mts in front of the current position then a flag is activated. 
 This flag is monitored to increase or decrease the ref_velocity.
 
 After the vehicle velocity and other vehicles positions are monitored, then the function  BehaviouralPLanning::e_possible_states PathPlanner::choose_next_state( ) is called, and contains the main logic of the FSM. It goes through all the cases and states described in the section before to select the state with the least cost and returns it.

Once a next state is chosen, the function   void PathPlanner::execute_next_state( e_possible_states next_state ) is called, which contains the main code to generate the trajectory to be send to the simulator. My implementation is mainly based from the class material, and follows the next sequences:

*  The local variable lane is later used to generate trajectory points and is set depending on the next state to execute, depending on the state other variables are set or resetted

~~~~
  switch( next_state )
        {
            case BehaviouralPLanning::KL:
                lane = m_current_lane;
                m_current_state = KL;
                m_target_lane_set = false;
                break;

            case BehaviouralPLanning::PLC:
                lane = m_current_lane;
                m_current_state = PLC;
                break;
            case LC:
                lane = m_target_lane;
                m_current_state = LC;
                break;
            default:
                break;
        }
~~~~


* The ancor points vectors of X and Y coordinates will be used in a later step to fit a polynomial for the trajectory path, and the first points to add are the previpus and current position of the car in map coordinates, in case the previous path array is empty, we obtain the previous position of the car by retrodicting the current x y and yaw variables. 

~~~~
    double ref_car_x = m_car_x;
    double ref_car_y = m_car_y;
    double ref_car_yaw = deg2rad( m_car_yaw );
    m_achor_points_x.clear( );
    m_achor_points_y.clear( );
    size_t previous_size = m_previous_path_x.size( );

    if( previous_size < 2 )
    {
        double prev_carx = m_car_x - std::cos( ref_car_yaw );
        double prev_cary = m_car_y - std::sin( ref_car_yaw );

        m_achor_points_x.push_back( prev_carx );
        m_achor_points_x.push_back( m_car_x );

        m_achor_points_y.push_back( prev_cary );
        m_achor_points_y.push_back( m_car_y );
    }
    else
    {
        ref_car_x = m_previous_path_x[ previous_size - 1 ];
        ref_car_y = m_previous_path_y[ previous_size - 1 ];

        double ref_car_prev_x = m_previous_path_x[ previous_size - 2 ];
        double ref_car_prev_y = m_previous_path_y[ previous_size - 2 ];
        ref_car_yaw = std::atan2( ref_car_y - ref_car_prev_y, ref_car_x - ref_car_prev_x );

        m_achor_points_x.push_back( ref_car_prev_x );
        m_achor_points_x.push_back( ref_car_x );

        m_achor_points_y.push_back( ref_car_prev_y );
        m_achor_points_y.push_back( ref_car_y );
    }
~~~~
* From the current position of the car in Frenet coordinates, 3 waypoints are created at curr_s + 30 mts, curr_s + 60 mts, and curr_s + 90 mts. The cartesians coordinates of the waypoints are computed with the getXY function from the helpers header, and added to the correspondant anchor point array. 

~~~~
    double s_1 = ( m_car_s + 30 );
    double s_2 = ( m_car_s + 60 );
    double s_3 = ( m_car_s + 90 );

    std::vector<double> wp0 = getXY( s_1, ( 2 + 4 * lane ), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y );
    std::vector<double> wp1 = getXY( s_2, ( 2 + 4 * lane ), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y );
    std::vector<double> wp2 = getXY( s_3, ( 2 + 4 * lane ), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y );

    m_achor_points_x.push_back( wp0[ 0 ] );
    m_achor_points_x.push_back( wp1[ 0 ] );
    m_achor_points_x.push_back( wp2[ 0 ] );

    m_achor_points_y.push_back( wp0[ 1 ] );
    m_achor_points_y.push_back( wp1[ 1 ] );
    m_achor_points_y.push_back( wp2[ 1 ] );
~~~~
* The anchor points are transfromed from map frame to vehicle frame 
~~~~
 for( size_t i = 0; i < m_achor_points_y.size( ); i++ )
    {
        double tempx = m_achor_points_x[ i ] - ref_car_x;
        double tempy = m_achor_points_y[ i ] - ref_car_y;

        m_achor_points_x[ i ] = tempx * cos( -ref_car_yaw ) - tempy * sin( -ref_car_yaw );
        m_achor_points_y[ i ] = tempx * sin( -ref_car_yaw ) + tempy * cos( -ref_car_yaw );
    }
~~~~

* The anchor points are input to the set_points function from the Spline.h impleemntation to approximate a polynomial with the data. 
~~~~
    tk::spline fitter;
    fitter.set_points( m_achor_points_x, m_achor_points_y );

 ~~~~
 
* Previously visited points are added to the output trajectory
~~~~
    m_trajectory.next_ptsx.insert( m_trajectory.next_ptsx.end( ), m_previous_path_x.begin( ), m_previous_path_x.end( ) );
    m_trajectory.next_ptsy.insert( m_trajectory.next_ptsy.end( ), m_previous_path_y.begin( ), m_previous_path_y.end( ) );
~~~~

* As explained in class, we have to compute the distance increment or separation between the output points in the trajectory, this will be affected by the desired reference velocity in meters and  the refresh rate of the sensor (20 ms). By the use of the fitted polynomial, we generated waypoints in vehicle coordinate frame with the correct separation, and transform them into map coordinate system to output to the simulator. 
~~~~
    //  compute target points from fitted polynomial
    double target_x = 30; // expressed in meters
    double target_y = fitter( target_x );

    double target_dist = std::sqrt( ( target_x*target_x ) + ( target_y*target_y ) );
    double N = target_dist / ( REFRESH_RATE*m_ref_velocity*MPH_TO_MTSPS );
    double dist_increment = target_x / N;
    double x_offset = 0;
    for( size_t i = 0; i < 50 - m_previous_path_x.size( ); i++ )
    {
        // point in local vehicle coordinate system
        double x_point_vcs = x_offset + dist_increment;
        double y_point_vcs = fitter( x_point_vcs );
        x_offset = x_point_vcs; /*save for next iteration*/

        // point in map coordinate system
        double x_point_mcs = ( x_point_vcs*cos( ref_car_yaw ) ) - ( y_point_vcs * sin( ref_car_yaw ) ) + ref_car_x;
        double y_point_mcs = ( x_point_vcs*sin( ref_car_yaw ) ) + ( y_point_vcs * cos( ref_car_yaw ) ) + ref_car_y;

        // add point to the path sent to sim
        m_trajectory.next_ptsx.push_back( x_point_mcs );
        m_trajectory.next_ptsy.push_back( y_point_mcs );

    }

~~~~
* Finally in main.cpp the  m_trajectory.next_ptsx and  m_trajectory.next_ptsy are send to simulator. 