#pragma once
#include <iostream>
#include <algorithm>

using namespace std;

struct point2D{
	double x;
	double y;
};

//check if points are collinear, clockvise or anticlockwise
int checkOrientation(point2D, point2D, point2D);

//check if points intersect
bool doIntersect(point2D, point2D, point2D, point2D);

//check intersections for collinear points
bool doCollinearIntersects(point2D, point2D, point2D);

