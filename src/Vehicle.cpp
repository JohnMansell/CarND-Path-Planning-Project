//
// Created by John on 11/22/2018.
//

#include <iostream>
#include "Vehicle.h"
#include "helper_functions.h"
#include <math.h>
#include <algorithm>
#include "json.hpp"
#include "spline.h"

#define LEFT -1
#define RIGHT 1
#define KEEP 0


using namespace std;

	constexpr double v_pi() { return M_PI; }
	double v_deg2rad(double x) { return x * v_pi() / 180; }
	double v_rad2deg(double x) { return x * 180 / v_pi(); }


// Lane Speeds
	double lane_0_speed;
	double lane_1_speed;
	double lane_2_speed;

	double lane_0_back_speed;
	double lane_1_back_speed;
	double lane_2_back_speed;

	double lane_0_clearance;
	double lane_1_clearance;
	double lane_2_clearance;

	double lane_0_back_clearance;
	double lane_1_back_clearance;
	double lane_2_back_clearance;

	double speed_limit = 50;

	double * lane_clearances[] = {& lane_0_clearance, & lane_1_clearance, & lane_2_clearance};
	double * back_clearances[] = {& lane_0_back_clearance, & lane_1_back_clearance, & lane_2_back_clearance};

	double * lane_speeds[] = { & lane_0_speed, & lane_1_speed, & lane_2_speed};
	double * back_speeds[] = { & lane_0_back_speed, & lane_1_back_speed, & lane_2_back_speed};

//--------------------------
//      Find Lane Speeds
//--------------------------
	void Vehicle::find_lane_speeds(const vector<Vehicle> &other_cars)
	{
		// Reset Variables
			lane_0_clearance = std::numeric_limits<double>::max();
			lane_1_clearance = std::numeric_limits<double>::max();
			lane_2_clearance = std::numeric_limits<double>::max();

			lane_0_speed = 50;
			lane_1_speed = 50;
			lane_2_speed = 50;


		// Lane Speeds
			for (auto other_car : other_cars)
			{
				// Declare Variables
					double clearance = (other_car.s - this->s);
					int lane = other_car.lane;

				// Front Clearance
					if ( ( clearance >= 0) && (clearance <= 75) && (clearance < * lane_clearances[lane]) )
					{
						* lane_clearances[lane] = clearance;
						* lane_speeds[lane] = other_car.speed;
					}

				// Back Clearance
					if ( ( clearance <= 0) && (clearance > -30) && (clearance > * back_clearances[lane]) )
					{
						* back_clearances[lane] = clearance;
						* back_speeds[lane] = other_car.speed;
					}
			}

	}


//--------------------------
//      Change 2 Lane Cost
//--------------------------
	double Vehicle::change_2_lane_cost(const vector<Vehicle> &other_cars, int direction)
	{
		// Stay on the Road
			if ( (lane == 0) && (direction == LEFT))  { return 999.9;}
			if ( (lane == 2) && (direction == RIGHT)) { return 999.9;}

			if (  lane == 1) { return 999.9;}

		// Cost -- Parameters
			double cost = 0;
			double lane_speeds[] = {lane_0_speed, lane_1_speed, lane_2_speed};
			double lane_clearance[] = {lane_0_clearance, lane_1_clearance, lane_2_clearance};

		// Cost -- Lane Speed
			double c1 = (50 - lane_speeds[lane + direction * 2]) / 50;
			double c2 = (1 / lane_clearance[lane + direction * 2]);

			cost = c1 + c2;
//
//			cout << "\n-----------------------------" << endl;
//			cout << "   Direction = " << direction * 2 << endl;
//			cout << "   C1 = " << c1 << endl;
//			cout << "   C2 = " << c2 << endl;
//			cout << "   CT = " << cost << endl;
//			cout << "------------------------------\n\n" << endl;

			return cost;

	}

//--------------------------
//      Change Lane Cost
//--------------------------
	double Vehicle::change_lane_cost(const vector<Vehicle> &other_cars, int direction)
	{
		// Stay on the Road
			if ( (lane == 0) && (direction == LEFT))  { return 999.9;}
			if ( (lane == 2) && (direction == RIGHT)) { return 999.9;}

		// Cost -- Parameters
			double cost = 0;
			double lane_speeds[] = {lane_0_speed, lane_1_speed, lane_2_speed};
			double lane_clearance[] = {lane_0_clearance, lane_1_clearance, lane_2_clearance};

		// Cost -- Lane Speed
			double c1 = (50 - lane_speeds[lane + direction]) / 50;
			double c2 = (1 / lane_clearance[lane + direction]);

			cost = c1 + c2;
//
//			cout << "\n-----------------------------" << endl;
//			cout << "   Direction = " << direction << endl;
//			cout << "   C1 = " << c1 << endl;
//			cout << "   C2 = " << c2 << endl;
//			cout << "   CT = " << cost << endl;
//			cout << "------------------------------\n\n" << endl;


		return cost;

	}

//--------------------------
//      Calculate Cost
//--------------------------
	void Vehicle::calculate_cost(const vector<Vehicle> &other_cars)
	{

		// Costs
			double keep_lane_cost;
			double change_left_cost;
			double change_right_cost;

			double change_left_2_cost;
			double change_right_2_cost;

		// Lane Speeds
			this->find_lane_speeds(other_cars);

		// Keep Lane Cost
			keep_lane_cost = change_lane_cost(other_cars, KEEP);

		// Change Lane Costs
			change_left_cost = change_lane_cost(other_cars, LEFT);
			change_right_cost = change_lane_cost(other_cars, RIGHT);

			change_left_2_cost = change_2_lane_cost(other_cars, LEFT);
			change_right_2_cost = change_2_lane_cost(other_cars, RIGHT);


		// Evaluate Costs
			double costs[] = {change_left_2_cost, change_left_cost, keep_lane_cost, change_right_cost, change_right_2_cost};
			double * best_cost = min_element(costs, costs + 5);


			if (* best_cost == change_left_2_cost)
			{
				this->target_lane = lane - 1;
				this->current_state = prep_2_left;
				this->next_state = prep_left;
				cout << "\n\n\n =========  !!!!!!!! =========  =========  !!!!!!!! ========= " << endl;
				cout << "               -- Prep 2 Left " << endl;
				cout << "               -- Target Lane = " << target_lane << endl;
				cout << "=========  !!!!!!!! =========  =========  !!!!!!!! ========= " << endl;
			}


			else if (* best_cost == change_right_2_cost)
			{
				this->target_lane = lane + 1;
				this->current_state = prep_2_right;
				this->next_state = prep_right;
				cout << "\n\n\n =========  !!!!!!!! =========  =========  !!!!!!!! ========= " << endl;
				cout << "               -- Prep 2 Right " << endl;
				cout << "               -- Target Lane = " << target_lane << endl;
				cout << "=========  !!!!!!!! =========  =========  !!!!!!!! ========= " << endl;
			}

			if (* best_cost == change_left_cost)
			{
				this->target_lane = lane - 1;
				this->current_state = prep_left;
				this->next_state = keep_lane;
				cout << "\n\n---------------------------" << endl;
				cout << " -- Prep Left " << endl;
				cout << " -- Target Lane = " << target_lane << endl;
				cout << "----------------------------" << endl;
			}



			else if (* best_cost == change_right_cost)
			{
				this->target_lane = lane + 1;
				this->current_state = prep_right;
				this->next_state = keep_lane;
				cout << "\n\n---------------------------" << endl;
				cout << " -- Prep Right " << endl;
				cout << " -- Target Lane = " << target_lane << endl;
				cout << "----------------------------" << endl;
			}

			else if (* best_cost == keep_lane_cost)
			{
				this->current_state = keep_lane;
				stay_in_lane();
			}


			if (* best_cost != keep_lane_cost)
			{
				cout << "Cost  = " << costs[0] << " -- " << costs[1] << " -- " << costs[2] << " -- " << costs[3] << " -- " << costs[4] << endl;
				cout << "Speed = " << lane_0_speed << " || " << lane_1_speed << " || " << lane_2_speed  << endl;
				cout << "Clear = " << lane_0_clearance << " -- " << lane_1_clearance << " -- " << lane_2_clearance << endl;

				cout << "------------------------------------" << endl;
				cout << "   Front Distance = " << this->front_distance << endl;
				cout << "------------------------------------" << endl;
			}

	}


//--------------------------
//      Adjust Speed
//--------------------------
	void Vehicle::adjust_speed()
	{
		// Close Enough
			if (abs(speed - target_speed) < 0.1)
				return;

		// Adjust Speed
			speed < target_speed ? (speed += max_accl) : (speed -= max_accl);
	}

//--------------------------
//      Plan Next State
//--------------------------
	void Vehicle::plan_next_state( const vector<Vehicle> & other_cars)
	{

		this->find_lane_speeds(other_cars);

		// Finish Change before planning another change
			if (current_state != keep_lane)
			{
				execute_planned_next_state( other_cars );
				return;
			}

		// Keep Lane
			else if (front_distance > 2.3 * following_distance)
				stay_in_lane();


		// Calculate Costs
			else this->calculate_cost(other_cars);

	}

//------------------------------------
//      Execute Planed Lane Change
//------------------------------------
	void Vehicle::execute_planned_next_state( const vector<Vehicle> & other_cars)
	{
//		// Lane Change complete ?
			if ( ( (1.5 + 4 * target_lane) < this->d ) && (this->d < (2.5 + 4 * target_lane)) )
			{
				cout << "\n\n" << endl;
				cout << " || Lane Change Complete || " << endl;
				cout << "\n--------- Before ----------\n" << endl;
				cout << "--- Lane = " << lane << endl;
				cout << "--- Targ = " << target_lane << endl;
				cout << "--- " << 1 + 4 * target_lane << " < " << (int) this->d << " < " << 3 + 4 * target_lane << endl;
				cout << "--- State = " << current_state << endl;
				cout << "--- Next  = " << next_state << "\n------------------------------------\n\n\n\n" << endl;

				// Update
					if (next_state == prep_left)
						target_lane = lane - 1;

					if (next_state == prep_right)
						target_lane = lane + 1;

					current_state = next_state;
					next_state = keep_lane;

					stay_in_lane();

				cout << "\n\n--------- After ----------\n" << endl;
				cout << "--- Lane = " << lane << endl;
				cout << "--- Targ = " << target_lane << endl;
				cout << "--- " << 1 + 4 * target_lane << " < " << (int) this->d << " < " << 3 + 4 * target_lane << endl;
				cout << "--- State = " << current_state << endl;
				cout << "--- Next  = " << next_state << "\n------------------------------------\n\n\n\n" << endl;

				lane_change_in_progress = false;
				return;
			}

		// Prep Lane Change
			switch (current_state)
			{
				case prep_left:
				case prep_2_left:
					prep_lane_change(other_cars, LEFT);
					break;

				case prep_right:
				case prep_2_right:
					prep_lane_change(other_cars, RIGHT);
					break;

				default:
					cout << "\n\n\n\n-----------------------------" << endl;
					cout << "!!!! -- Error -- !!!!" << endl;
					cout << "current state = " << current_state << endl;
					cout << "----------------------------------\n\n\n\n" << endl;
			}


//		adjust_speed();
	}

//-----------------------------
//      Wait or Drop Back
//-----------------------------
	void Vehicle::wait_or_drop_back(const vector<Vehicle> &other_cars, int direction)
	{

		if (* lane_clearances[lane + direction * 2] > * lane_clearances[lane + direction])
			return;

		// Cout
			cout << "Wait or Drop Back " << endl;

		// Find Car in Next Lane
			double next_car_distance = std::numeric_limits<double>::max();
			Vehicle * car_next_to_me;
			for (auto car : other_cars)
			{
				if (car.lane == this->target_lane)
				{
					double distance = abs(this->s - car.s);
					if ( (distance < next_car_distance) && (distance > -2));
					{
						car_next_to_me = & car;
						next_car_distance = distance;
					}

				}
			}

		// Error Check
			if(car_next_to_me == nullptr)
				return;

		// Compare Speeds
			double speed_difference = abs(leading_car_speed - car_next_to_me->speed);

			cout << " --- Speed diff = " << speed_difference << endl;

		// Take Action
			if (speed_difference < 0.5)
			{
				target_speed = *lane_speeds[target_lane] - 10;
				adjust_speed();
			}

			else
				stay_in_lane();

			cout << " --- Target Speed = " << target_speed << "\n\n" << endl;


	}

//--------------------------
//      Keep Lane
//--------------------------
	void Vehicle::stay_in_lane()
	{
		// Don't Crash
			if (front_distance <= safety_buffer)
				target_speed = leading_car_speed - 3;

		// Follow
			else if ( (front_distance >= safety_buffer) && (front_distance <= following_distance) )
				target_speed = leading_car_speed;

			else if ( (front_distance >= following_distance) && (front_distance <= 2 * following_distance) )
				target_speed = leading_car_speed + 2;

		// Speed Limit
			else
				target_speed = speed_limit - 0.5;

		adjust_speed();
	}

//--------------------------
//      Prep Lane Change
//--------------------------
	void Vehicle::prep_lane_change(const vector<Vehicle> & other_cars, int direction)
	{
		// Only Prep Once
			if (lane_change_in_progress)
				return;

		// Safety
			find_lane_speeds(other_cars);
			bool target_lane_clear = true;
			for (auto car : other_cars)
			{
				double clearance = ( car.s - this->s );

				if ((car.lane == this->lane + direction) && (clearance >= - 2 * safety_buffer ) && (clearance <= following_distance))
				{
					target_lane_clear = false;
				}
			}

		// Execution
			if ( target_lane_clear && (front_distance > 3 * safety_buffer) && ( !wait_to_pass() ))
				execute_lane_change(other_cars, direction);

		// Adjust
			else
			{
				if (current_state == prep_2_left)
					wait_or_drop_back(other_cars, LEFT);

				if (current_state == prep_2_right)
					wait_or_drop_back(other_cars, RIGHT);
			}

			if ( ! lane_change_in_progress )
				stay_in_lane();


	}


//-----------------------------
//      Wait to Pass
//-----------------------------
	bool Vehicle::wait_to_pass()
	{
		if (* lane_clearances[this->target_lane] > *lane_clearances[this->lane])
			return false;

		// Time to Pass
		//      s  = v1(t)
		//      s  = v2(t) + c
		//      t  = c / (v - v2)
		double v1 = this->speed;
		double v2 = *lane_speeds[target_lane];

		double c = * lane_clearances[target_lane];

		double t = c / (v1 - v2);

		// Time to leading Car
			double m1 = this->speed;
			double m2 = * lane_speeds[lane];

			double d = this->front_distance;

			double t2 = d / (m1 - m2);

		// Pass or Wait
			bool wait =  (t <= t2) ? true : false;

			if (wait)
			{
				cout << "Wait to pass" << endl;
				return true;

			}
	}

//-----------------------------
//      Execute Lane Change
//-----------------------------
	void Vehicle::execute_lane_change(const vector<Vehicle> & other_cars, int direction)
	{

	// Only Execute Once
		if (lane_change_in_progress)
		return;

	// Safety
	find_lane_speeds(other_cars);

		cout << "\n\n\n-----------------------------\n Execute Lane Change" << endl;
		cout << "------------------------------" << endl;

		this->lane = this->target_lane;
		this->target_distance = 10;

		cout << " Lane Speeds = " << * lane_speeds[0] << " || " << * lane_speeds[1] << " || " << * lane_speeds[2] << endl;
		cout << " Clearance   = " << * lane_clearances[0] << " -- " << * lane_clearances[1] << " -- " << * lane_clearances[2] << endl;
		cout << "\n Target = " << endl;
		cout << "-- Lane  : " << target_lane << endl;
		cout << "-- Speed : " << speed << " --> " << target_speed << endl;
		cout << "-- Dist  : " << target_distance << endl;

		lane_change_in_progress = true;

		cout << " -- Execution Complete -- " << endl;
	}

//---------------------------------
//      Populate Data from JSON
//---------------------------------
	void Vehicle::populate_data_from_json(const nlohmann::json &j)
	{
		this->x = j[1]["x"];
		this->y = j[1]["y"];
		this->s = j[1]["s"];
		this->d = j[1]["d"];
		this->yaw = j[1]["yaw"];

		this->s += 0.02 * (double)j[1]["speed"];
	}


// ToDo:: Double lane changes marked complete after 1 lane change
// ToDo:: Check "lane change complete" step





























































