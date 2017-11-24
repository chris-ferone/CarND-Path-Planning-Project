#include <vector>
#include <iostream>
using namespace std;

//function declarations
vector<int> get_feasible_lanes(int current_lane);
int find_current_lane(double car_d);
double average_car_speed(double car_speed);
unsigned int inefficency_cost(double average_speed, double target_speed);

//variable definitions
vector<double> speeds;

//function definitions

//This function returns a set of lanes that the ego vehicle can remain in or switch to
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
	return states;
}

//This function find the current lane the ego vehicle is in	
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

//if the speed of the ego vehicle matches the target speed, then there is not cost, but
// as ego vehicle speed drops below target speed, inefficency_cost increases
unsigned int inefficency_cost(double average_speed, double target_speed)
{
	double diff = target_speed - average_speed;
	double pct = float(diff) / target_speed;
	return pct * 100*1.5;
}