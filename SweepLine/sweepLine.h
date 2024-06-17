#pragma once
#include "points.h"
#include <set>
#include <unordered_map>
#include<string>
#include <vector>
using namespace std;

struct segment {
	point2D left;
	point2D right;
};


struct event {
	point2D p;
	bool isLeft;
	int index;

	event();
	event(point2D p1, bool l, int i) : p(p1), isLeft(l), index(i) {};

	//for binary tree (set data structure)
	bool operator<(const event& e) const {
		if (p.y == e.p.y) return p.x < e.p.x;
		return p.y < e.p.y;
	}

};

set<event>::iterator pred(set<event>&, set<event>::iterator);
set<event>::iterator succ(set<event>&, set<event>::iterator);
//for check intersections
int checkInteresections(segment[], int);


