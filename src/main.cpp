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
#include "helperFunctions.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
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

// Transform from Frenet s,d coordinates to Cartesian x,y
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

int main() {
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

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
  
  double ref_vel = 0;
  double lane = 1;
  bool uptospeed = false;
  double timer = 0;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane, &uptospeed, &timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
			////////////////////////////////////////////////////////////////////////////////////////////////////

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			
			
			
			int prev_size = previous_path_x.size();	
			
			// WHY??????
			if (prev_size > 0 )
			{
				car_s = end_path_s;
			}
			
			
			// Go through sensor fusion list, see if any other cars are in our lane, and if they are close
			
			bool too_close = false;
			vector<int> buffer_cost = {0, 0, 0};
			int current_lane = find_current_lane(car_d);
			//cout << "current lane: " << current_lane << endl;
			vector<int> feasible_lanes = get_feasible_lanes(lane);
			
			for (vector<int>::iterator fl_it = feasible_lanes.begin(); fl_it != feasible_lanes.end(); fl_it++)
				{
				cout << *fl_it  << " | ";
				}
			double follow_speed = 0;
			double target_speed = 49.5;
			
			for(int i = 0; i < sensor_fusion.size(); i++)
			{
				float d = sensor_fusion[i][6];
			//check if any other cars are in my lane or in lane(s) I may switch to (i.e. feasible lanes)
				for (vector<int>::iterator fl_it = feasible_lanes.begin(); fl_it != feasible_lanes.end(); fl_it++)
				{
					if(d < (2+4* *fl_it+2) && d > (2+4* *fl_it-2))
					{
						// if a car is in my lane or a lane I might switch to, find its velocity and position (in s)
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx*vx+vy*vy);
						double check_car_s = sensor_fusion[i][5];
						
						//if using previous points can project s value out
						check_car_s += ((double)prev_size*.02*check_speed);
						double buffer = check_car_s-car_s;
						//if another car is in front of us and close to us, then take action (i.e. slow down or change lanes)
						//cout << (*fl_it == current_lane) <<  (check_car_s > car_s) << (buffer < 30) << (buffer > 0) << endl;
						if((*fl_it == lane) && (check_car_s > car_s) && (buffer < 30) )
						{
							//too_close = true;
							target_speed = 2.24*check_speed;
							//cout << "target speed: " << target_speed << " ID: " << sensor_fusion[i][0] << endl;
							//cout << "too close: " << too_close <<  " current lane: " << current_lane << " fl_it " << *fl_it << endl;
						}
						double acost = 0;
						if(*fl_it != current_lane) // if another car is in front or behind us in another lane we are considering moving into, calculate buffer cost
						{
							//Calculate buffer cost for that lane
							// the smaller the distance (i.e. buffer), the larger the cost should be
							if (buffer >= -7 && buffer <7)
							{
								//No go zone; infinite cost
								acost = 500;
							}
							else if (buffer >= 7 && buffer <30)
							{
								acost = .1*pow((buffer-30),2);
							}
						 	/* else if (buffer < -5 && buffer > -10)
							{
								acost = 3.6*pow((fabs(buffer)-10),2);
							}  */
							else
							{
								acost = 0;
							}
							buffer_cost[*fl_it] =  buffer_cost[*fl_it] + acost;
						}
						//cout << "too1 close: " << too_close << endl;
					}
					//cout << "too2 close: " << too_close << endl;
				}
				//cout << "too3 close: " << too_close << endl;
			}	
			//cout << "too4 close: " << too_close << endl;
			
			unsigned int ineff_cost = inefficency_cost(car_speed, 49.5);
			buffer_cost[lane] = ineff_cost;
			
			
			
			cout << "Buffer Cost: ";
			for (vector<int>::iterator fl_it = buffer_cost.begin(); fl_it != buffer_cost.end(); fl_it++)
				{			
				cout << setw(10) << *fl_it  << " | ";
				}
			//cout << endl;	
				
			

 			if(ref_vel > target_speed)
			{
				ref_vel -= .224;
				;
				//cout << ref_vel << endl;
				//cout << "decrease speed" << ref_vel << endl;
			}
			else if(ref_vel < target_speed)
			{
				ref_vel += .224;
				//cout << "increase speed" << ref_vel << endl;
			} 
			
			if (buffer_cost[lane] < 1)
			{
				uptospeed = true;
			}
			// find lowest cost lane
			
			timer = timer + .02;
			int minlane = 99;
			vector<int> all_lanes = {0, 1, 2};
			vector<int> left_lanes = {0, 1};
			vector<int> right_lanes = {1, 2};
			
			if (uptospeed) //don't change lanes when the vehicle is starting up from a stop at the begining of simulation
			{
				if (feasible_lanes == all_lanes)
				{
					cout << " all ";
					minlane = min_element(buffer_cost.begin(), buffer_cost.end()) - buffer_cost.begin();
				}
				else if (feasible_lanes == left_lanes)
				{	
					cout << " left ";
					if (buffer_cost[0] < buffer_cost[1])
					{
						minlane = 0;
					}
					else
					{
						minlane = 1;
					}
				}
				else if (feasible_lanes == right_lanes)
				{	
					cout << " left ";
					if (buffer_cost[1] < buffer_cost[2])
					{
						minlane = 1;
					}
					else
					{
						minlane = 2;
					}
				}
				else
				{
					cout << "ERROR" << endl;
				}
				cout << " minlane: " << minlane;
				// The new lane must cost signifncatly less, and at least some time must have passed since last lane change (to avoid unnecessary lanes changes). 
				if ( (minlane != 99) && (buffer_cost[lane] - buffer_cost[minlane] > 20) && timer > 3)
				{
					cout << "LANE CHANGE!   minlane: " << minlane << " current lane: " << lane << " buffer_cost[minlane]: " << buffer_cost[minlane] << " buffer_cost[lane]: "<< buffer_cost[lane] << " target speed: " << target_speed << endl; //" diff: " << buffer_cost[lane] - buffer_cost[minlane] << endl;
					lane = minlane;
					timer = 0; // reset timer 
					
				}
			}
			
			cout << " lane: " << lane << " ref_vel: " << ref_vel  << " target speed: " << target_speed  << " buffer_cost[lane]: " << buffer_cost[lane]  << " current lane: " << lane << endl;
			
			
			//////// Define coarse trajectories waypoints BEGIN
			
			//vector<vector<double>> ptsX;
			//vector<vector<double>> ptsY;
			
			vector<double> ptsx;
			vector<double> ptsy;
		  
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			

			
			
			
			ptsx.clear();
			ptsy.clear();
			// if previous size is almost empty, use the car as starting point	
			//cout << "1" << endl;
			if (prev_size < 2){
				//Use two points that make the path tangent to the car
				cout << "2" << endl;
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);
				
				ptsx.push_back(prev_car_x);
				//cout << "3" << endl;
				ptsx.push_back(car_x);
				
				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}			
			else {
				//cout << "3" << endl;
				//Redefine reference state as previous path end point
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];
				
				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
				
				//Use two points that make the path tangent to the previous path's end point
				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);
				
				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}			
			//cout << "4" << endl;
			//In Frenet add evenly 30m spaced points ahead of starting reference
			vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);
			
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);
			
			//ptsX.push_back(ptsx);
			//ptsY.push_back(ptsy);
			
			//cout << "5" << endl;
			//cout << ptsX.size() << endl;
			//cout << ptsX[0].size() << endl;
			//cout << ptsX[1].size() << endl;
			//cout << ptsX[2].size() << endl;
			//cout << ptsX[2][1] << endl;
			//vector<double> ptsxx = ptsX;
			//vector<double> ptsyy = ptsY;
			//cout << "6" << endl;
			//cout << ptsxx.size() << " " << ptsyy.size() << endl;
			
			for (int i =0; i < ptsx.size(); i++)
			{
				//shift car refrence angle to 0 degrees\
				// transformation to local car's coordinate system
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
				ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
			}
			//cout << "7" << endl;
			//Define coarse trajectory waypoints END, Fit waypoints to spline BEGIN
			
			// create a spline
			tk::spline spl;
			//cout << "7.1" << endl;
			// set(x, y) points to the spline
			spl.set_points(ptsx, ptsy);
			//cout << "8" << endl;
			//Start with all of the previous path points from last time
			for (int i = 0; i < previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}
			//cout << "9" << endl;
			// Calculate how to break up spline points so that we travel at our desired reference velocity
			double target_x = 30.0;
			double target_y = spl(target_x);
			double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
			
			double x_add_on = 0;
			
			// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
			
			for (int i = 1; i <= 50 - previous_path_x.size(); i++){
				double N = (target_dist/(.02*ref_vel/2.24));
				double x_point = x_add_on+target_x/N;
				double y_point = spl(x_point);
				
				x_add_on = x_point;
				
				double x_ref = x_point;
				double y_ref = y_point;
				
				//roate back to normal after rotating it earlier
				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
				
				x_point += ref_x;
				y_point += ref_y;
				
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}

			//////////////////////////////////////////////////////////////////////////////////////		
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

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
