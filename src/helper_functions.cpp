//
// Created by John on 11/21/2018.
//

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "Vehicle.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void helper_test()
{
	int a = 50;
	int b = 30;

}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//------------------------------
//      Has Data
//------------------------------
	string hasData(string s) {
		auto found_null = s.find("null");
		auto b1 = s.find_first_of("[");
		auto b2 = s.find_first_of("}");
		if (found_null != string::npos) {
			return "";
		} else if (b1 != string::npos && b2 != string::npos) {
			return s.substr(b1, b2 - b1 + 2);
		}
		return "";
	}

//------------------------------
//      Distance
//------------------------------
	double distance(double x1, double y1, double x2, double y2)
	{
		return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	}

//------------------------------
//      Way Point -- Closest
//------------------------------
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
	{

		double closestLen = 100000; //large number
		int closestWaypoint = 0;

		for(int i = 0; i < maps_x.size(); i++)
		{
			double map_x = maps_x[i];
			double map_y = maps_y[i];
			double dist = distance(x,y,map_x,map_y);
			if(dist < closestLen)
			{
				closestLen = dist;
				closestWaypoint = i;
			}

		}

		return closestWaypoint;

	}

//------------------------------
//      Way Point -- Next
//------------------------------
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
	{

		int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

		double map_x = maps_x[closestWaypoint];
		double map_y = maps_y[closestWaypoint];

		double heading = atan2((map_y-y),(map_x-x));

		double angle = fabs(theta-heading);
		angle = min(2*pi() - angle, angle);

		if(angle > pi()/4)
		{
			closestWaypoint++;
			if (closestWaypoint == maps_x.size())
			{
				closestWaypoint = 0;
			}
		}

		return closestWaypoint;
	}


//------------------------------
//      Get Frenet
//          - Transform from Cartesian (x,y)
//          - coordinates to Frenet s,d coordinates
//------------------------------
	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
	{
		int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

		int prev_wp;
		prev_wp = next_wp-1;
		if(next_wp == 0)
		{
			prev_wp  = maps_x.size()-1;
		}

		double n_x = maps_x[next_wp]-maps_x[prev_wp];
		double n_y = maps_y[next_wp]-maps_y[prev_wp];
		double x_x = x - maps_x[prev_wp];
		double x_y = y - maps_y[prev_wp];

		// find the projection of x onto n
			double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
			double proj_x = proj_norm*n_x;
			double proj_y = proj_norm*n_y;

			double frenet_d = distance(x_x,x_y,proj_x,proj_y);

		//see if d value is positive or negative by comparing it to a center point
			double center_x = 1000-maps_x[prev_wp];
			double center_y = 2000-maps_y[prev_wp];
			double centerToPos = distance(center_x,center_y,x_x,x_y);
			double centerToRef = distance(center_x,center_y,proj_x,proj_y);

			if(centerToPos <= centerToRef)
			{
				frenet_d *= -1;
			}

		// calculate s value
			double frenet_s = 0;
			for(int i = 0; i < prev_wp; i++)
			{
				frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
			}

			frenet_s += distance(0,0,proj_x,proj_y);

			return {frenet_s,frenet_d};

	}


//------------------------------
//      Get X Y
//          - Transform from Frenet
//          - s,d coordinates to Cartesian x,y
//------------------------------
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
	{
		int prev_wp = -1;

		while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
		{
			prev_wp++;
		}

		int wp2 = (prev_wp+1)%maps_x.size();

		double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
		// the x,y,s along the segment
		double seg_s = (s-maps_s[prev_wp]);

		double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
		double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

		double perp_heading = heading-pi()/2;

		double x = seg_x + d*cos(perp_heading);
		double y = seg_y + d*sin(perp_heading);

		return {x,y};

	}

//------------------------------
//      Get Velocity
//------------------------------
	double get_velocity(double vx, double vy)
	{
		double mph = (sqrt( pow(vx, 2) + pow(vy, 2) ));
		return mph * 2.24;
	}

//-------------------------------
//      Set Way Points
//-------------------------------
	void set_waypoints(
			vector<double> & map_waypoints_s,
			vector<double> & map_waypoints_x,
			vector<double> & map_waypoints_y,
			vector<double> & next_wp0,
			vector<double> & next_wp1,
			vector<double> & next_wp2,
			vector<double> & ptsx,
			vector<double> & ptsy,
			const double car_s,
			double ref_x,
			double ref_y,
			double ref_yaw,
			const int lane)

	{
		// Set Way Points
			next_wp0 = getXY(car_s + 30, (2 + 4* lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			next_wp1 = getXY(car_s + 60, (2 + 4* lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			next_wp2 = getXY(car_s + 90, (2 + 4* lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

		// Transformation to car's reference
			for (int i = 0; i < ptsx.size(); i++)
			{
				// Angle to zero
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
				ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
			}
	}

//----------------------------------
//      Set Future Points (x , y)
//----------------------------------
	void set_future_points(
			vector<double> & next_x_vals,
			vector<double> & next_y_vals,
			tk::spline s,
			const vector<double> & previous_path_x,
			const vector<double> & previous_path_y,
			double ref_x,
			double ref_y,
			double ref_yaw,
			double ref_velocity,
			double target_x)
	{
		// Start with all the previous path points from last time
		for (int i=0; i < previous_path_x.size(); i++)
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		// Calculate how to break up spline ponits so that we travel at our desired reference velocity
			double target_y = s(target_x);
			double target_dist = sqrt( (target_x * target_x) + (target_y * target_y));

			double x_add_on = 0;

		// Fill up the rest of the path planner after filling it with previous points.
		// Here we will always output 50 points
			for (int i=0; i <= 50 - previous_path_x.size(); i++)
			{
				double N = (target_dist / (0.02 * ref_velocity / 2.24) );
				double x_point = x_add_on + (target_x) / N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// rotate back to normal after rotating it earlier
				x_point = ( x_ref * cos(ref_yaw) - ( y_ref * sin(ref_yaw)));
				y_point = ( x_ref * sin(ref_yaw) + ( y_ref * cos(ref_yaw)));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}
	}

//-------------------------------------
//      Sensor Fusion -- Extract Cars
//-------------------------------------
	void extract_cars(const vector<vector<double>> & sensor_fusion, vector<Vehicle> & other_cars)
	{
		for (auto car: sensor_fusion)
		{

			// Lane Widths
				const int lane_line_0 = 0;
				const int lane_line_1 = 4*1;
				const int lane_line_2 = 4*2;
				const int lane_line_3 = 4*3;

			// Lane
				double d = car[6];
				int car_lane;

				if ( (d > lane_line_0) && (d < lane_line_1))
					car_lane = 0;

				if ( (d > lane_line_1) && (d < lane_line_2))
					car_lane = 1;

				if ( (d > lane_line_2) && (d < lane_line_3))
					car_lane = 2;

			// Construct new Vehicle
				Vehicle car_vehicle = Vehicle(car_lane);

			// Position
				car_vehicle.s = car[5];
				car_vehicle.d = car[6];

			// Velocity
				double vx = car[3];
				double vy = car[4];
				car_vehicle.speed = get_velocity(vx, vy);

			// Adjust Position
				car_vehicle.s += car_vehicle.speed * 0.02;

				other_cars.push_back(car_vehicle);
		}
	}





















































