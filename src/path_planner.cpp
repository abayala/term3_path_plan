#include "path_planner.hpp"
namespace BehaviouralPLanning
{
    PathPlanner::PathPlanner ( )
    {
        m_trajectory.next_ptsx.clear ( );
        m_trajectory.next_ptsy.clear ( );
        m_current_lane = 1;
        m_ref_velocity = 0.0;
        m_state = KL;
        m_other_car_too_close = false;
    }

    PathPlanner::~PathPlanner ( )
    {

    }

    std::vector<e_possible_states> PathPlanner::successor_states ( )
    {
        std::vector<e_possible_states> succ_states;
        if ( KL == m_state )
        {
            succ_states.push_back ( KL );
            if ( m_other_car_too_close )
            {
                succ_states.push_back ( LCL );
                succ_states.push_back ( LCR );

            }
        }
        else if ( LCL == m_state )
        {
            succ_states.push_back ( LCL );
            succ_states.push_back ( KL );
        }
        else if ( LCR == m_state )
        {
            succ_states.push_back ( LCR );
            succ_states.push_back ( KL );
        }
        return succ_states;
    }

    double PathPlanner::compute_cost ( e_possible_states state )
    {
        double cost;

        return cost;
    }

    e_possible_states PathPlanner::choose_next_state ( )
    {
        e_possible_states out_next_state;

        // get a list of succesor states

        std::vector<e_possible_states> next_states = successor_states ( );
        std::vector<double> costs;

        for ( size_t i = 0; i < next_states.size ( ); i++ )
        {
            double option_cost = compute_cost ( next_states.at ( i ) );
        }
        return out_next_state;

    }

    void PathPlanner::execute_next_state ( e_possible_states next_state )
    {

    }

    void PathPlanner::process_data ( double car_x , double car_y , double car_s , double car_d , double car_yaw , double car_speed , std::vector<double> previous_path_x , std::vector<double> previous_path_y , double end_path_s , double end_path_d , std::vector<std::vector <double>> sensor_fusion )
    {
        // save data to private members
        m_previous_path_x = previous_path_x;
        m_previous_path_y = previous_path_y;
        m_car_yaw = car_yaw;
        m_sensor_fusion = sensor_fusion;


        size_t prev_size = previous_path_x.size ( );

        if ( prev_size > 0 )
        {
            m_car_s = end_path_s;
            m_car_d = end_path_d;
        }
        else
        {
            m_car_s = car_s;
            m_car_d = car_d;
        }

        m_current_lane = getLaneId ( m_car_d );

        m_other_car_too_close = false;

        // check if other cars are close to our car
        for ( size_t i = 0; i < m_sensor_fusion.size ( ); i++ )
        {
            double d = m_sensor_fusion [ i ] [ 6 ];
            if ( isCarInLane ( d , m_current_lane ) )
            {
                // if car is in my lane check its speed
                double vx = m_sensor_fusion [ i ] [ 3 ];
                double vy = m_sensor_fusion [ i ] [ 4 ];
                double linear_speed = sqrt ( ( vx*vx ) + ( vy*vy ) )* MPH_TO_MTSPS;
                // get car position in the lane
                double other_car_s = sensor_fusion [ i ] [ 5 ];
                // compensate through time and preditc car_s
                other_car_s += double ( prev_size ) * FRAME_RATE * linear_speed;
                // if car is closer thatn a certain threshold 
                if ( other_car_s > car_s && ( ( other_car_s - car_s ) < CLOSE_RANGE ) )
                {
                    // set ref vel to a lower value
                    m_other_car_too_close = true;

                }

            }
        }

        if ( m_other_car_too_close )
        {
            m_ref_velocity -= SPEED_INCREMENT;
        }
        else if ( m_ref_velocity < MAX_SPEED )
        {
            m_ref_velocity += SPEED_INCREMENT;
        }

        e_possible_states next_sate = choose_next_state ( );


    }

}