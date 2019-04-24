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

Once a next state is chosen, the function  
 
 
 

