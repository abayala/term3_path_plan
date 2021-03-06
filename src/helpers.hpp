#ifndef __HELPERS_H
#define __HELPERS_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <string>
#include <vector>
#include <algorithm>

double pi( );
std::string hasData ( std::string s );
double deg2rad ( double x );
double rad2deg ( double x );
double distance ( double x1 , double y1 , double x2 , double y2 );
int ClosestWaypoint ( double x , double y , const std::vector<double> &maps_x , const std::vector<double> &maps_y );
int NextWaypoint ( double x , double y , double theta , const std::vector<double> &maps_x , const std::vector<double> &maps_y );
std::vector<double> getFrenet ( double x , double y , double theta , const std::vector<double> &maps_x , const std::vector<double> &maps_y );
std::vector<double> getXY ( double s , double d , const std::vector<double> &maps_s , const std::vector<double> &maps_x , const std::vector<double> &maps_y );
bool isCarInLane ( double d , int lane );
int getLaneId ( double d );

#endif  // HELPERS_H
