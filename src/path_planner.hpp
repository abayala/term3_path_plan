#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_
#include <vector>
namespace BehaviouralPLanning
{
    //constants
    const double REFRESH_RATE = 0.02;
    const double MPH_TO_MTSPS = 1.0 / 2.24; // factor to transform miles per hour to meters per second
    const double CLOSE_RANGE = 30; // expressed in meters
    const double MAX_DESIRED_SPEED = 49.5; // expressed in miles per hour
    const int MAX_LANE_ID = 2;
    const int MIN_LANE_ID = 0;
    const double LANE_WIDTH = 4;
    const double LANE_CENTER_D = LANE_WIDTH / 2;

    const double SPEED_INCREMENT = 0.224; // expressed in miles per hour

    enum e_possible_states { KL,  PLC, LC }; // possible states for the state machine, KL= Keep Lane, PLC= Prepare Lane Change, LC = Lane Change
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
    /************************************************************************/
    /* class members declarations                                           */
    /************************************************************************/
        int m_current_lane;
        int m_target_lane;
        bool m_target_lane_set;
        double m_ref_velocity;
        bool m_other_car_too_close;
        e_possible_states m_current_state;
        s_trajectory m_trajectory;
        // Load up map values for waypoint's x,y,s and d normalized normal vectors
        std::vector<double> m_map_waypoints_x;
        std::vector<double> m_map_waypoints_y;
        std::vector<double> m_map_waypoints_s;

        std::vector<double> m_achor_points_x;
        std::vector<double> m_achor_points_y;

     
        /************************************************************************/
        /*function declarations                                                */
        /************************************************************************/

        /**
        /* \fn:    		compute_cost_lane_change
        /* \brief:      Computes from 0 to 1 the cost of changing to the target lane
        /* \parameter:	target_lane
        /* \returns:	double
        */
        double compute_cost_lane_change(int target_lane );
        
        /**
        /* \fn:    		choose_next_state
        /* \brief:      Taking into account the current state, it will determine which state is best to change to
        /* \returns:	BehaviouralPLanning::e_possible_states
        */
        e_possible_states choose_next_state( );
        /**
        /* \fn:    		execute_next_state
        /* \brief:      Performs state change, and trajectory computation
        /* \parameter:	next_state
        /* \returns:	void
        */
        void execute_next_state ( e_possible_states next_state );
        /**
        /* \fn:    		process_data
        /* \brief:      Main input for the class, it performs one cycle for th einput data
        /* \returns:	void
        */
        void process_data ( double car_x , double car_y, double car_s, double car_d, double car_yaw,double car_speed,
                            std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                            double end_path_s, double end_path_d, std::vector<std::vector <double>> sensor_fusion );
        
        /**
        /* \fn:    		predict_obstacles_s
        /* \brief:      predicts the position of the vehicles in sensor fusion array
        /* \returns:	std::vector<double>
        */
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