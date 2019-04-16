#include "path_planner.hpp"
#include "helpers.hpp"
#include <iterator>
#include <algorithm>
#include "spline.h"
namespace BehaviouralPLanning
{
    PathPlanner::PathPlanner ( )
    {
        m_trajectory.next_ptsx.clear ( );
        m_trajectory.next_ptsy.clear ( );
        m_current_lane = 1;
        m_ref_velocity = 0.0;
        m_current_state = KL;
        m_other_car_too_close = false;
        m_target_lane_set = false;
    }

    PathPlanner::~PathPlanner ( )
    {

    }

    

    double PathPlanner::compute_cost_lane_change( int target_lane ) {

        double out_cost = 0.0;
        // We shall not overtake if the target lane is outside the boundaries of the road
        if( target_lane > MAX_LANE_ID || target_lane < MIN_LANE_ID )
        {
            out_cost = 1.0;
        }
        else
        {
            // predict position of other vehicles around me
            std::vector<double> other_cars_pred_s = predict_obstacles_s( );
            for( size_t i = 0; i < m_sensor_fusion.size( ); i++ )
            {
                //check if vehicle is in desired target lane
                if( isCarInLane( m_sensor_fusion[ i ][ e_sensor_fus_indexes::D ], target_lane ) )
                {
                    // check how far is the other car from me
                    double DTC = std::abs( other_cars_pred_s[ i ] ) - std::abs( m_car_s );
                    //if car is in a critical distance from my car
                    if( abs( DTC ) <= CLOSE_RANGE / 2 )
                    {
                        out_cost = 1.0;
                        break;
                    }
                    // if car is closer than a certain threshold 
                    else  if( other_cars_pred_s[ i ] > m_car_s && ( DTC <= CLOSE_RANGE ) )
                    {
                        double current_cost = 1.0 ;
                        //output the highest cost 
                        if( current_cost > out_cost )
                        {

                            out_cost = current_cost;
                        }

                    }
                 
                }

            }
        }

        return out_cost;
    }

    
    BehaviouralPLanning::e_possible_states PathPlanner::choose_next_state()
{

        e_possible_states out_next_state = KL;
        switch( m_current_state )
        {
            case BehaviouralPLanning::KL:
                if( m_other_car_too_close )
                {
                    out_next_state = PLC;
                }
                else
                {
                    out_next_state = KL;
                }
                break;
            case BehaviouralPLanning::LC:
                if( m_target_lane_set &&
                    m_current_lane == m_target_lane && 
                    ( 0.5 < ( LANE_CENTER_D - std::abs( double( int( m_car_d ) % int( LANE_WIDTH ) ) ) ) ) 
                    )
                {
                    out_next_state = KL;
                }
                else
                {
                    out_next_state = LC;
                }
                break;
            case BehaviouralPLanning::PLC:
            {
                double cost_LCL, cost_LCR, cost_KL;
                std::vector<double> costs;

                cost_LCL = compute_cost_lane_change( m_current_lane - 1 );
                cost_LCR = compute_cost_lane_change( m_current_lane + 1 );
                cost_KL = 1 / ( cost_LCR + cost_LCL + 0.0000000001 );

                costs.push_back( cost_LCL );
                costs.push_back( cost_LCR );
                costs.push_back( cost_KL );



                std::vector<double>::iterator best_cost = std::min_element( costs.begin( ), costs.end( ) );

                int best_idx = std::distance( costs.begin( ), best_cost );

                switch( best_idx )
                {
                    case 0:
                        out_next_state = LC;
                        m_target_lane = m_current_lane - 1;
                        m_target_lane_set = true;
                        break;
                    case 1:
                        out_next_state = LC;
                        m_target_lane = m_current_lane + 1;
                        m_target_lane_set = true;
                        break;
                    
                    case 2:
                        out_next_state = KL;
                        m_target_lane_set = false;

                    default:
                        break;
                }


            }
                break;
            default:
                break;
        }

        return out_next_state;
    }

    void PathPlanner::execute_next_state( e_possible_states next_state )
    {
        int lane;
        m_trajectory.next_ptsx.clear( );
        m_trajectory.next_ptsy.clear( );
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

        double s_1 = m_car_s + 30;
        double  s_2 = m_car_s + 60;
        double  s_3 = m_car_s + 90;
        if (s_1 > m_map_waypoints_s[ m_map_waypoints_s .size ()-1] )
        {
            s_1 = std::abs(MAX_S-s_1);
        }
        if( s_2 >  m_map_waypoints_s[ m_map_waypoints_s.size( ) - 1 ] )
        {
            s_2 = std::abs(MAX_S- s_2);
        }
        if( s_3 >  m_map_waypoints_s[ m_map_waypoints_s.size( ) - 1 ] )
        {
            s_3 = std::abs (MAX_S-s_3);
        }
        std::vector<double> wp0 = getXY(s_1, ( 2 + 4 * lane ), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y );
        std::vector<double> wp1 = getXY(s_2 , ( 2 + 4 * lane ), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y );
        std::vector<double> wp2 = getXY(s_3, ( 2 + 4 * lane ), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y );

        m_achor_points_x.push_back( wp0[ 0 ] );
        m_achor_points_x.push_back( wp1[ 0 ] );
        m_achor_points_x.push_back( wp2[ 0 ] );

        m_achor_points_y.push_back( wp0[ 1 ] );
        m_achor_points_y.push_back( wp1[ 1 ] );
        m_achor_points_y.push_back( wp2[ 1 ] );

        // transform points from map frame to car frame
        for( size_t i = 0; i < m_achor_points_y.size( ); i++ )
        {
            double tempx = m_achor_points_x[ i ] - ref_car_x;
            double tempy = m_achor_points_y[ i ] - ref_car_y;

            m_achor_points_x[ i ] = tempx * cos( -ref_car_yaw ) - tempy * sin( -ref_car_yaw );
            m_achor_points_y[ i ] = tempx * sin( -ref_car_yaw ) + tempy * cos( -ref_car_yaw );
        }

        tk::spline fitter;
        fitter.set_points( m_achor_points_x, m_achor_points_y );

        // insert previous path points into the curren trajectory
        m_trajectory.next_ptsx.insert( m_trajectory.next_ptsx.end( ), m_previous_path_x.begin( ), m_previous_path_x.end( ) );
        m_trajectory.next_ptsy.insert( m_trajectory.next_ptsy.end( ), m_previous_path_y.begin( ), m_previous_path_y.end( ) );

        //  compute target points from fitted polynomial
        double target_x = 30;
        double target_y = fitter( target_x );

        double target_dist = std::sqrt( ( target_x*target_x ) + ( target_y*target_y ) );
        double N = target_dist / ( FRAME_RATE*m_ref_velocity*MPH_TO_MTSPS );
        double dist_increment = target_x / N;
        double x_offset = 0;
        for( size_t i = 0; i < 50 - m_previous_path_x.size( ); i++ )
        {
            // point in local vehicle coordinate syste
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

    }

    void PathPlanner::process_data ( double car_x , double car_y , double car_s , double car_d , double car_yaw , double car_speed , std::vector<double> previous_path_x , std::vector<double> previous_path_y , double end_path_s , double end_path_d , std::vector<std::vector <double>> sensor_fusion )
    {
        // save data to private members
        m_car_x = car_x;
        m_car_y = car_y;
        m_car_yaw = car_yaw;
        m_car_speed = car_speed;
        m_previous_path_x = previous_path_x;
        m_previous_path_y = previous_path_y;
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
            double d = m_sensor_fusion [ i ] [ e_sensor_fus_indexes::D ];
            if ( isCarInLane ( d , m_current_lane ) )
            {
                // if car is in my lane check its speed
                double vx = m_sensor_fusion [ i ] [ 3 ];
                double vy = m_sensor_fusion [ i ] [ 4 ];
                double linear_speed = sqrt ( ( vx*vx ) + ( vy*vy ) )* MPH_TO_MTSPS;
                // get car position in the lane
                double other_car_s = sensor_fusion [ i ] [ e_sensor_fus_indexes::S ];
                // compensate through time and preditc car_s
                other_car_s += double ( prev_size ) * FRAME_RATE * linear_speed;
                // if car is closer thatn a certain threshold 
                if ( other_car_s > m_car_s && ( ( other_car_s - m_car_s ) < CLOSE_RANGE ) )
                {
                    // set ref vel to a lower value
                    m_other_car_too_close = true;

                }

            }
        }
       
            if( m_other_car_too_close )
            {
                m_ref_velocity -= SPEED_INCREMENT *( m_ref_velocity/ DESIRED_SPEED );
            }
            else if( m_ref_velocity < DESIRED_SPEED )
            {
                m_ref_velocity += SPEED_INCREMENT;
            }
        
      

        e_possible_states next_sate = choose_next_state ( );
        execute_next_state( next_sate );


    }

    std::vector<double> PathPlanner::predict_obstacles_s( ) {

        std::vector<double> output;

        // check if other cars are close to our car
        for( size_t i = 0; i < m_sensor_fusion.size( ); i++ )
        {
            
           
                // if car is in my lane check its speed
                double vx = m_sensor_fusion[ i ][ 3 ];
                double vy = m_sensor_fusion[ i ][ 4 ];
                double linear_speed = sqrt( ( vx*vx ) + ( vy*vy ) )* MPH_TO_MTSPS;
                // get car position in the lane
                double other_car_s = m_sensor_fusion[ i ][ 5 ];
                // compensate through time and preditc car_s
                other_car_s += double( m_previous_path_x.size () ) * FRAME_RATE * linear_speed;
                output.push_back( other_car_s );

        }
        return output;

    }

}