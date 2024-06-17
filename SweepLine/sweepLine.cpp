#include "sweepLine.h"
#include "points.h"

// Find predecessor of iterator in s.
set<event>::iterator pred(set<event>& s, set<event>::iterator it) {
	return it == s.begin() ? s.end() : --it;
}

// Find successor of iterator in s.
set<event>::iterator succ(set<event>& s, set<event>::iterator it) {
	return ++it;
}

int checkInteresections(segment arr[], int n) {

	//variables
	set<event> T;
	int numInt = 0;
	vector<event> allEvents;
	unordered_map<string, int> mp;

	//insert all endpoints
	for (int i = 0; i < n; i++)
	{
		allEvents.push_back(event(arr[i].left, true, i));
		allEvents.push_back(event(arr[i].right, false, i));
	}

	//sort endpoints
	sort(allEvents.begin(), allEvents.end(), [](event& e1, event& e2) {return e1.p.x < e2.p.x; });

	//start event loop

	for (int i = 0; i < 2 * n; i++)
	{
		event curr = allEvents[i];

		if (curr.isLeft) {
			auto next = T.lower_bound(curr);
			auto previous = pred(T, next);

			//check if prev is intersecting
			if (previous != T.end() && doIntersect(arr[curr.index].left, arr[curr.index].right, arr[previous->index].left, arr[previous->index].right)) {
				
				string s = to_string(previous->index+1) + " " + to_string( curr.index+1);
				if (mp.count(s) == 0)
				{
					mp[s]++;
					numInt++;
				}
			}
			//check if next is intersecting
			if (next != T.end() && doIntersect(arr[curr.index].left, arr[curr.index].right, arr[next->index].left, arr[next->index].right)) {
				string s = to_string(curr.index+1) + " " + to_string(next->index+1);
				if (mp.count(s) == 0)
				{
					mp[s]++;
					numInt++;
				}
			}
			if (previous != T.end() && next != T.end() && next->index == previous->index)numInt--;
			T.insert(curr);
		}
		else {
			auto it = T.find(event(arr[curr.index].left, true, curr.index));
			auto previous = pred(T, it);
			auto next = succ(T, it);

			//check if previous intersects next
			if (previous != T.end() && next != T.end()) {
				string s = to_string(previous->index+1) + " " + to_string(next->index+1);
				string s1 = to_string(next->index+1) + " " + to_string(previous->index+1);
				if (mp.count(s) == 0 && mp.count(s1)==0 && doIntersect(arr[previous->index].left, arr[previous->index].right, arr[next->index].left, arr[next->index].right))
				{
					mp[s]++;
					numInt++;
				}
			}
			T.erase(it);
		}
	}
	for (auto& pr : mp)
	{
		cout << "\n" << pr.first;
	}
	return numInt;
};