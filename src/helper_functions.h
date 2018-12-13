//
// Created by John on 11/21/2018.
//

#ifndef PATH_PLANNING_HELPER_FUNCTIONS_H
#define PATH_PLANNING_HELPER_FUNCTIONS_H

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


//-----------------------------
//      Vehicle info
//-----------------------------
	string hasData(string s);
	double distance(double x1, double y1, double x2, double y2);
	void extract_cars(const vector<vector<double>> & sensor_fusion, vector<Vehicle> & other_cars);

//-----------------------------
//      Map Functions
//-----------------------------
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	double get_velocity(double vx, double vy);

	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


//-----------------------------
//      Path Planning
//-----------------------------
	void set_waypoints(
			vector<double> & map_waypoints_s,
			vector<double> & map_waypoints_x,
			vector<double> & map_waypoints_y,
			vector<double> & next_wp0,
			vector<double> & next_wp1,
			vector<double> & next_wp2,
			vector<double> & ptsx,
			vector<double> & ptsy,
			double car_s,
			double ref_x,
			double ref_y,
			double ref_yaw,
			int lane);

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
			double target_x);



#endif //PATH_PLANNING_HELPER_FUNCTIONS_H
