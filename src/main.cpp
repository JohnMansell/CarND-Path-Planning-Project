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
#include "helper_functions.cpp"
#include "Vehicle.cpp"
#include "Vehicle.h"

using namespace std;

// JSON - for convenience
	using json = nlohmann::json;

//// For converting back and forth between radians and degrees.
//	constexpr double pi() { return M_PI; }
//	double deg2rad(double x) { return x * pi() / 180; }
//	double rad2deg(double x) { return x * 180 / pi(); }



//==============================
//      -- Main --
//==============================
	int main() {

	// Connect to Socket
		uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
		  vector<double> map_waypoints_x;
		  vector<double> map_waypoints_y;
		  vector<double> map_waypoints_s;
		  vector<double> map_waypoints_dx;
		  vector<double> map_waypoints_dy;

	// Waypoint map to read from
	    string map_file_ = "../data/highway_map.csv";

    // The max s value before wrapping around the track back to 0
		double max_s = 6945.554;

	// Read from File
		ifstream in_map_(map_file_.c_str(), ifstream::in);

	// Waypoints
		string line;
		while (getline(in_map_, line)) {
	        istringstream iss(line);
		    double x;
		    double y;
		    float s;
		    float d_x;
		    float d_y;
		    iss >> x;
		    iss >> y;
		    iss >> s;
		    iss >> d_x;
		    iss >> d_y;
		    map_waypoints_x.push_back(x);
		    map_waypoints_y.push_back(y);
		    map_waypoints_s.push_back(s);
		    map_waypoints_dx.push_back(d_x);
		    map_waypoints_dy.push_back(d_y);
		}

	// Parameters
		int lane = 1;
		double ref_velocity = 0.0; // mph
		double target_distance = 10.0;
		Vehicle ego = Vehicle(lane);
		ego.current_state = Vehicle::keep_lane;

	// Lane Widths
		const int lane_line_0 = 0;
		const int lane_line_1 = 4;
		const int lane_line_2 = 4*2;
		const int lane_line_3 = 4*3;


	// JSON Message
		h.onMessage([&ego, &lane, &ref_velocity, &target_distance, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		                     uWS::OpCode opCode) {
		    // "42" at the start of the message means there's a websocket message event.
		    // The 4 signifies a websocket message
		    // The 2 signifies a websocket event
		    //auto sdata = string(data).substr(0, length);
		    //cout << sdata << endl;
	    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

	    auto s = hasData(data);

//----------------------
//      Main Loop
//----------------------
if (s != "")
{
	// Parse JSON
	    auto j = json::parse(s);
	    string event = j[0].get<string>();

    if (event == "telemetry") {
	    // j[1] is the data JSON object

	    // Localization -- Ego
	        ego.populate_data_from_json(j);

	    // Previous path data given to the Planner
		    auto previous_path_x = j[1]["previous_path_x"];
		    auto previous_path_y = j[1]["previous_path_y"];

	    // Previous path's end s and d values
		    double end_path_s = j[1]["end_path_s"];
		    double end_path_d = j[1]["end_path_d"];

	    // Sensor Fusion Data, a list of all other cars on the same side of the road.
	        auto sensor_fusion = j[1]["sensor_fusion"];

	    // Previous Size
	        int prev_size = previous_path_x.size();

	        double car_s = ego.s;
	        double car_d = ego.d;

		    if (prev_size > 0) {
		    	car_s = end_path_s;
		    	car_d = end_path_d;
		    }

	    // Flags
		    bool too_close = false;
		    double follow_speed;

	    // Initialize Parameters
		    ego.front_distance = 999;
		    ego.target_speed = 49.5;

	    // Generate Predictions from Sensor Fusion Data
		    vector<Vehicle> other_cars;
		    map<int, vector<vector<double>>> predictions;

		    other_cars.reserve(sensor_fusion.size());


	    //-------------------------------------
	    //      Get Cars from Sensor Fusion
	    //-------------------------------------
	        extract_cars(sensor_fusion, other_cars);

		    for (auto other_car: other_cars)
		    {

			    // Handle Cars in my lane
			    if (other_car.lane == ego.lane)
			    {
				    // Car is in front of ego
				    if ( other_car.s >= (ego.s - 0.5) )
				    {
					    double front_distance = other_car.s - ego.s;

					    if (front_distance < ego.front_distance)
					    {
						    ego.front_distance = front_distance;
						    ego.leading_car_speed = other_car.speed;
					    }
				    }
			    }

		    }

	// Plan Next State
		ego.plan_next_state(other_cars);
	    target_distance = ego.target_distance;
	    ref_velocity = ego.speed;
	    lane = ego.lane;


    json msgJson;

    // Way Point Vectors
        vector<double> ptsx;
        vector<double> ptsy;

    // Futre Point Vectors
	    vector<double> next_x_vals;
	    vector<double> next_y_vals;

    // Reference x, y, yaw states
        double ref_x = ego.x;
        double ref_y = ego.y;
        double ref_yaw = deg2rad(ego.yaw);

    // If previous size is almost empty, use car as starting reference
        if(prev_size < 2)
        {
            // Use two points that make the path tangent to the car
            double prev_car_x = ego.x - cos(ego.yaw);
            double prev_car_y = ego.y - sin(ego.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ego.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ego.y);
        }

    // Use the previous path's end point as a starting reference
        else
        {
            // Redefine reference state as previous path end point
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
        }

	// Set Way Points
		vector<double> next_wp0;
        vector<double> next_wp1;
        vector<double> next_wp2;

		set_waypoints(
				map_waypoints_s,
				map_waypoints_x,
				map_waypoints_y,
				next_wp0,
				next_wp1,
				next_wp2,
				ptsx,
				ptsy,
				car_s,
				ref_x,
				ref_y,
				ref_yaw,
				lane);


    // Spline
        tk::spline spline;
	    spline.set_boundary(tk::spline::second_deriv, 0.0, tk::spline::second_deriv, 0.0, true);
	    spline.set_points(ptsx, ptsy);

    // Set Future Points
        set_future_points(
        		next_x_vals,
        		next_y_vals,
        		spline,
        		previous_path_x,
        		previous_path_y,
        		ref_x,
        		ref_y,
        		ref_yaw,
        		ref_velocity,
        		target_distance);


    // Send Trajectory to car
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;
        auto msg = "42[\"control\","+ msgJson.dump()+"]";

    //this_thread::sleep_for(chrono::milliseconds(1000));
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }

}
//--------------------------
//      Manual Driving
//--------------------------
	else
	{
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
}
});







//===========================
//      Ignore
//---------------------------

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(


	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
	                     size_t, size_t) {
	    const std::string s = "<h1>Hello world!</h1>";
	    if (req.getUrl().valueLength == 1) {
	      res->end(s.data(), s.length());
	    } else {
	      // i guess this should be done more gracefully?
	      res->end(nullptr, 0);
	    }
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	    std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	                         char *message, size_t length) {
	    ws.close();
	    std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
	    std::cout << "Listening to port " << port << std::endl;
	} else {
	    std::cerr << "Failed to listen to port" << std::endl;
	    return -1;
	}
	h.run();
	}
