#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_
#include <vector>
namespace BehaviouralPLanning
{
    //constants
    const double FRAME_RATE = 0.02;
    const double MPH_TO_MTSPS = 1.0 / 2.24; // factor to transform miles per hour to meters per second
    const double CLOSE_RANGE = 20; // expressed in meters
    const double DESIRED_SPEED = 49.5; // expressed in miles per hour
    const int MAX_LANE_ID = 2;
    const int MIN_LANE_ID = 0;

    const double SPEED_INCREMENT = 0.224; // expressed in miles per hour

    enum e_possible_states { KL,  LCL,  LCR };
    enum e_sensor_fus_indexes {ID=0,X_COOR,Y_COOR, VX,VY,S,D };
    struct s_trajectory 
    {
        std::vector<double> next_ptsx;
        std::vector<double> next_ptsy;

    };

    class PathPlanner
    {
    public: 
        PathPlanner ( );
        ~PathPlanner ( );
        //member declarations
        int m_current_lane;
        double m_ref_velocity;
        bool m_other_car_too_close;
        e_possible_states m_state;
        s_trajectory m_trajectory;
        // Load up map values for waypoint's x,y,s and d normalized normal vectors
        std::vector<double> m_map_waypoints_x;
        std::vector<double> m_map_waypoints_y;
        std::vector<double> m_map_waypoints_s;

        std::vector<double> m_achor_points_x;
        std::vector<double> m_achor_points_y;

     

        //function declartions
        std::vector<e_possible_states> successor_states ( );
        double compute_cost ( e_possible_states state_to_check );

        e_possible_states choose_next_state ( );
        void execute_next_state ( e_possible_states next_state );
        void process_data ( double car_x , double car_y, double car_s, double car_d, double car_yaw,double car_speed,
                            std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                            double end_path_s, double end_path_d, std::vector<std::vector <double>> sensor_fusion );
        
        std::vector<double> predict_obstacles_s( );
       
    private:
        // Main car's localization Data
        double m_car_x ;
        double m_car_y ;
        double m_car_s ;
        double m_car_d ;
        double m_car_yaw ;
        double m_car_speed ;

        // Previous path data given to the Planner
        std::vector<double> m_previous_path_x;
        std::vector<double> m_previous_path_y;
        
        // Sensor Fusion Data, a list of all other cars on the same side 
        //   of the road.
        std::vector<std::vector<double>> m_sensor_fusion ;
    };                    
}                         
#endif                    