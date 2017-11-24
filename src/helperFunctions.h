#include <vector>
#include <iostream>
using namespace std;

//function declarations
vector<int> get_feasible_lanes(int current_lane);
int find_current_lane(double car_d);
double average_car_speed(double car_speed);
unsigned int inefficency_cost(double average_speed, double target_speed);
//vector<double> find_buffer_cost(sensor_fusion, feasible_lanes, current_lane, car_s);
//double calculate_speed_setpoint (double speed_goal, double current_speed, auto sensor_fusion, int current_lane, double car_s, double car_d, int prev_size);

vector<double> speeds;



//function definitions
vector<int> get_feasible_lanes(int current_lane)
		{
	//lane 0 == far left lane
	//lane 1 == middle lane
	//lane 2 == far right lane
	vector<int> states = {0, 1, 2};
	if (current_lane == 0)
	{
		states.erase(remove(states.begin(), states.end(), 2), states.end());
	}
	else if (current_lane == 2)
	{
		states.erase(remove(states.begin(), states.end(), 0), states.end());
	}
	
	for (vector<int>::const_iterator i = states.begin(); i != states.end(); ++i)
	{
	//	cout << *i << ' ';
	}
	//cout << endl; //*/
	return states;
		}
		
int find_current_lane(double car_d)
{
	int current_lane = 99;
	if (car_d > 0 && car_d < 4)
		{
			current_lane = 0;
		}
	else if (car_d > 4 && car_d < 8)
		{
			current_lane = 1;
		}
	else if (car_d > 8 && car_d < 12)
		{
			current_lane = 2;
		}
	return current_lane;
}

double average_car_speed(double car_speed)
{
	//moving average over last n cycles
	
	speeds.push_back(car_speed);
	if (speeds.size() > 100 )
	{
		speeds.erase(speeds.begin());
	}
	double average_speed = accumulate(speeds.begin(), speeds.end(), 0) / speeds.size(); 
	//cout << speeds.size() << " " << average_speed << endl;
	return average_speed;
}


unsigned int inefficency_cost(double average_speed, double target_speed)
{
	//inefficency cost
	//if the average speed of a ego vehicle is lower than the target speed, then there is a high cost for staying in the current lane
	// this cost is only applied to current lane trajectory
	double diff = target_speed - average_speed;
	double pct = float(diff) / target_speed;
	return pct * 100*1.5;
}

/*
vector<double> find_buffer_cost(sensor_fusion, feasible_lanes, current_lane, car_s)
{
	//buffer cost
	//does the trajectory overlap with another vehicle's position or come close to a collision
	//distance between ego vehicle and predicted vehicles	
	//check if lane is clear, if it is clear, no cost, if it is not, high cost
	vector<double> buffer_cost = {0, 0, 0};
	// Go through sensor fusion list, see if any other cars are in our lane, and if they are close, then slow down
	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		//check if any other cars are in my lane or in lane(s) I may switch to (i.e. feasible lanes)
		for (vector<double>::iterator fl_it = feasible_lanes.begin(); fl_it != feasible_lanes.end(); fl_it++;)
		{
			if(d < (2+4* *fl_it+2) && d > (2+4* *fl_it-2))
			{
				// find velocity and position (in s) of other cars
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx+vy*vy);
				double check_car_s = sensor_fusion[i][5];
				
				//if using previous points can project s value out
				check_car_s += ((double)prev_size*.02*check_speed);
			
				if(*fl_it == current_lane)
				{
				//if another car is in front of us and close to us, then take action (i.e. slow down or change lanes)
					if((check_car_s > car_s) && ((check_car_s-car_s) < 30) )
					{
						too_close = true;
						//ref_vel = check_speed;
						if (lane > 0)
						{
							lane = 0;
						}
					
					}
				}
				else //lane we might switch into
				{
					//Calculate buffer_cost
					//longitudinal distane between ego vehicle and other vehicle
					double buffer = check_car_s - car_s;
					double min_buffer = 10;
					if (buffer < min_buffer || buffer > -min_buffer)
					{
						buffer_cost[*fl_it] += buffer * 1000;
					}
					
				}
			}
		}
	}
	return 1;
	
	
}
*/
	

// create function to calculate target speed, look ahead only in current lane. 
//if slower vehicle ahead, maintain a distance. (i.e. match speed of vehicle in front)
	
/* double calculate_speed_setpoint (double speed_goal, double current_speed, auto sensor_fusion, int current_lane, double car_s, double car_d, int prev_size)
{	
	//positive acceleration limit (to prevent violating jerk violation)
	//no deceleration limit
	//distance_target = 10;
	double speed_reference = speed_goal;
	double max_accel = 2; 
	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		if(d < (2+4*current_lane+2) && d > (2+4*current_lane-2))
		{
			// find velocity and position (in s) of other cars
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensor_fusion[i][5];
			
			//if using previous points can project s value out
			check_car_s += ((double)prev_size*.02*check_speed);
		
			//if another car is in front of us and close to us, then take action (i.e. slow down or change lanes)
			if((check_car_s > car_s) && ((check_car_s-car_s) < 30) )
			{
				speed_reference = check_speed;
			}
		}
	}
	// decelerate if we are going fasater than lead vehicle or we are too close to lead vehicle
	if (current_speed > speed_reference)
	{
		speed_reference = current_speed - max_accel*.02;
	}
	// accelerate if we are going slower than lead vehicle
	else if (current_speed < speed_reference)
	{
		speed_reference = current_speed + max_accel*.02;
	}
	
	
	return speed_reference;
}	 */









