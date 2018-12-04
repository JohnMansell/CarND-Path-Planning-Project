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

	double lane_0_clearance;
	double lane_1_clearance;
	double lane_2_clearance;

int fastest_lane()
{
	if ( (lane_0_speed > lane_1_speed) && (lane_0_speed > lane_2_speed))
		return 0;

	if ( (lane_1_speed > lane_0_speed) && (lane_1_speed > lane_2_speed))
		return 1;

	if ( (lane_2_speed > lane_0_speed) && (lane_2_speed > lane_1_speed))
		return 2;

	return 0;
}

//--------------------------
//      Find Lane Speeds
//--------------------------
	void Vehicle::find_lane_speeds(const vector<Vehicle> &other_cars)
	{
		// Lane Distances
			lane_0_clearance = std::numeric_limits<double>::max();
			lane_1_clearance = std::numeric_limits<double>::max();
			lane_2_clearance = std::numeric_limits<double>::max();

			lane_0_speed = 50;
			lane_1_speed = 50;
			lane_2_speed = 50;

		// Lane Speeds
			for (auto other_car : other_cars)
			{

				double clearance = (other_car.s - this->s);

				if (-5 <= clearance)
				{
					cout << "\n\nEgo s = " << this->s << endl;
					cout << "Oth s = " << other_car.s << endl;
					cout << "D     = " << other_car.d << endl;
					cout << "Lane  = " << other_car.lane << endl;
					cout << "Clear = " << clearance << endl;
				}


				if ((other_car.s >= (this->s)) && (clearance < 50))
				{

					switch (other_car.lane)
					{
						case 0:
							if (clearance < lane_0_clearance)
							{
								lane_0_clearance = clearance;
								lane_0_speed = other_car.speed;
							}
							break;

						case 1:
							if (clearance < lane_1_clearance)
							{
								lane_1_clearance = clearance;
								lane_1_speed = other_car.speed;
							}
							break;

						case 2:
							if (clearance < lane_2_clearance)
							{
								lane_2_clearance = clearance;
								lane_2_speed = other_car.speed;
							}
							break;

						default:
							cout << "\n\n -- Error -- \n\n   other_car.lane = " << other_car.lane << endl;
							break;
					}

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

			if ( lane == 1) { return 999.9;}

		// Is it secret? Is it safe?
			for (auto other_car : other_cars)
			{
				if ( (other_car.lane == lane + direction) && (other_car.s >= s - 1.5 ) && (other_car.s <= s + safety_buffer))
					return 999;

				if ( (other_car.lane == lane + (direction * 2)) && (other_car.s >= s - 1.7 ) && (other_car.s <= s + 2 * safety_buffer))
					return 999;
			}

		// Cost -- Parameters
			double cost = 0;
			double lane_speeds[] = {lane_0_speed, lane_1_speed, lane_2_speed};
			double lane_clearance[] = {lane_0_clearance, lane_1_clearance, lane_2_clearance};

		// Cost -- Lane Speed
			cost += (50 - lane_speeds[lane + direction * 2]) / 50;
			cost += (1 / lane_clearance[lane + direction * 2]);

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

		// Is it secret? Is it safe?
			for (auto other_car : other_cars)
			{
				if ( (other_car.lane == lane + direction) && (other_car.s >= s - 1.7 ) && (other_car.s <= s + safety_buffer))
					return 999;
			}

		// Cost -- Parameters
			double cost = 0;
			double lane_speeds[] = {lane_0_speed, lane_1_speed, lane_2_speed};
			double lane_clearance[] = {lane_0_clearance, lane_1_clearance, lane_2_clearance};

		// Cost -- Lane Speed
			cost += (50 - lane_speeds[lane + direction]) / 50;
			cost += (1 / lane_clearance[lane + direction]);


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

		// Flags
			bool safety_flag_left = true;
			bool safety_flag_right = true;

		// Lane Speeds
			this->find_lane_speeds(other_cars);
			double lane_speeds[] = { lane_0_speed, lane_1_speed, lane_2_speed};

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


			cout << "\n" << endl;
			int speed_adjust = 4;



			if (* best_cost == change_left_2_cost)
			{
				lane -= 2;
				this->target_speed = lane_speeds[lane - 2] - speed_adjust;
				this->current_state = change_left;
				adjust_speed();
				cout << " Change 2x Left " << lane << endl;
			}



			else if (* best_cost == change_right_2_cost)
			{
				lane += 2;
				this->target_speed = lane_speeds[lane + 2] - speed_adjust;
				this->current_state = change_right;
				adjust_speed();
				cout << " Change 2x Right " << lane << endl;
			}

			else if (* best_cost == change_left_cost)
			{
				lane -= 1;
				this->target_speed = lane_speeds[lane - 1] - speed_adjust;
				this->current_state = change_left;
				adjust_speed();
				cout << " Change Left " << lane << endl;
			}



			else if (* best_cost == change_right_cost)
			{
				lane += 1;
				this->target_speed = lane_speeds[lane + 1] - speed_adjust;
				this->current_state = change_right;
				adjust_speed();
				cout << " Change Right " << lane << endl;
			}

			else if (* best_cost == keep_lane_cost)
			{
				this->current_state = keep_lane;
				cout << "Stay in Lane " << lane << endl;
				stay_in_lane();
			}



			cout << "Cost  = " << costs[0] << " -- " << costs[1] << " -- " << costs[2] << " -- " << costs[3] << " -- " << costs[4] << endl;
			cout << "Speed = " << lane_speeds[0] << " || " << lane_speeds[1] << " || " << lane_speeds[2] << " --- " <<  lane_speeds[lane] << endl;
			cout << "Clear = " << lane_0_clearance << " -- " << lane_1_clearance << " -- " << lane_2_clearance << endl;

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
	void Vehicle::plan_next_state( const vector<Vehicle> &other_cars)
	{
		// Finish Change before planning another change
			if (current_state != keep_lane)
			{
				// Lane Change complete ?
					if ((this->d >= (4 * lane) + 2 ) || (this->d <= (4 * lane) - 2))
					{
						return;
					}

					else {current_state = keep_lane;}
			}

		// Keep Lane
			if (front_distance > 3 * following_distance)
				stay_in_lane();

		// Calculate Costs
			else this->calculate_cost(other_cars);

	}

//--------------------------
//      Keep Lane
//--------------------------
	void Vehicle::stay_in_lane()
	{
		if (front_distance <= safety_buffer)
		{
			target_speed = leading_car_speed - 3;
			cout << "Safety Buffer" << endl;
		}

		else if ( (front_distance >= safety_buffer) && (front_distance <= following_distance) )
		{
			target_speed = leading_car_speed;
		}

		else if ( (front_distance >= following_distance) && (front_distance <= 2 * following_distance) )
		{
			target_speed = leading_car_speed + 2;
		}

		else {
			target_speed = 49.5;
		}

		adjust_speed();
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
//
		this->s += 0.02 * (double)j[1]["speed"];
	}































































