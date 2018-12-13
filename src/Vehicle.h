//
// Created by John on 11/22/2018.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

//---------------------
//      Include
//---------------------
	#include <vector>
	#include <map>
	#include <string>
	#include <math.h>
	#include "json.hpp"

	using namespace std;

//-------------------------
//      Parameters
//-------------------------
	const double safety_buffer = 3;
	const double following_distance = 20;
	const double max_accl = 0.28;
	const int speed_adjust = 4;

	bool lane_change_in_progress;

//==========================
//      Vehicle
//==========================
	class Vehicle {

	public:

		// Position
			double x;
			double y;
			double yaw;

			double s;
			double vs;
			double as;

			double d;
			double vd;
			double ad;

			int lane;
			int target_lane;
			double speed;

		// Other Cars
			double front_distance;
			double target_speed = 49.5;
			double leading_car_speed = 0;
			double target_distance = 10;


		enum State { keep_lane, change_left, change_right, prep_2_left, prep_2_right, prep_left, prep_right, drop_back};

		State current_state;
		State next_state;

		//----------------------
		//      Constructors
		//----------------------
			Vehicle() {};

			Vehicle(double s, double vs, double as, double d, double vd, double ad)
				: s(s), vs(vs), as(as), d(d), vd(vd), ad(ad), speed(0) {};

			Vehicle(int lane)
				: lane(lane){};

		//----------------------
		//      Destructor
		//----------------------
			virtual ~Vehicle() = default;

		//----------------------
		//      Functions
		//----------------------
			void adjust_speed();

			void plan_next_state(const vector<Vehicle> &other_cars);

			void stay_in_lane();

			void calculate_cost(const vector<Vehicle> &other_cars);

			double change_lane_cost(const vector<Vehicle> &other_cars, int direction);

			void find_lane_speeds(const vector<Vehicle> &other_cars);

			void populate_data_from_json(const nlohmann::json &j);

			double change_2_lane_cost(const vector<Vehicle> &other_cars, int direction);

			void execute_planned_next_state( const vector<Vehicle> & other_cars);

			void prep_lane_change(const vector<Vehicle> & other_cars, int direction);

			void execute_lane_change(const vector<Vehicle> & other_cars, int direction);

			void wait_or_drop_back(const vector<Vehicle> & other_cars, int direction);

			bool wait_to_pass();
	};

//-------------------------
//      Helper Functions
//-------------------------
	int fastest_lane();


#endif //PATH_PLANNING_VEHICLE_H
