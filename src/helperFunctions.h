#include <vector>
#include <iostream>
using namespace std;

vector<int> get_feasible_lanes(int current_lane);

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
	/*for (vector<int>::const_iterator i = states.begin(); i != states.end(); ++i)
	{
		cout << *i << ' ';
	}
	cout << endl; */
	return states;
		}
