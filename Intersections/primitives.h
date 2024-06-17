#pragma once

#include <vector>
#include <set>
#include <iostream>
#include <cmath>

#define X 0
#define Y 1
#define Z 2
#define DIM 3
typedef double tPointd[DIM];
typedef tPointd tFace[DIM];
typedef int tFaceIndcies[DIM];



std::ostream& operator<<(std::ostream& os, const tPointd& p);

float areaTriangle2D(const tPointd& p, const tPointd& q, const tPointd& r);

// Function to determine if a tPointd is on the left side of a directed line
bool isLeft(const tPointd& p, const tPointd& q, const tPointd& r);

// Function to determine if a tPointd is on the left side of or on the directed line
bool isLeftOn(const tPointd& p, const tPointd& q, const tPointd& r);

// Function to determine if a tPointd is on the left side of a directed line
bool isCollinear(const tPointd& p, const tPointd& q, const tPointd& r);

// Function to determine if a tPointd is on the right side of a directed line
bool isRight(const tPointd& p, const tPointd& q, const tPointd& r);

// Function to determine if a tPointd is between two points. All three are collinear
bool isBetween(const tPointd& p, const tPointd& q, const tPointd& r);
// Function to determine if two lines intersect properly
    bool intersectProp(const tPointd & p, const tPointd & q, const tPointd & r, const tPointd & s);
// Function to determine if two lines intersect properly
    bool intersect(const tPointd& p, const tPointd& q, const tPointd& r, const tPointd& s);

//intersection using parametric formula and gives intersection points. Here we can overload the intersect function using 
//return type and number of variables but since return types are different, it can lead misunderstandings at different points in code
//We can load it within another interest function to keep it familiar
    char segSegIntersect(const tPointd& p, const tPointd& q, const tPointd& r, const tPointd& s, tPointd& found);

//xor function
    bool Xor(bool first, bool second);

// Used for two segments which are parallel to check for collinear, edge connections
    char parallelSegInt(const tPointd& p,const  tPointd& q,const tPointd& r, const tPointd& s, tPointd& found);

// for getting plane equation
    int planecCoeff(const tPointd& p, const tPointd& q, const tPointd& r, tPointd& N, double& D);

//calculate normal vector from three points
    void normalVec(const tPointd& p, const tPointd& q, const tPointd& r, tPointd& N);

//dot product
    double Dot(const tPointd& p, const tPointd& q);

    void subVec(const tPointd& p, const tPointd& q, tPointd& r);

// check if seg and plane intersect. Return type can be p : parallel, 0: non-intersecting, 1: intersecting, q: first endpoint lies on plane, r: second enpoint lies on plane
    char segPlaneIntersect(const tFace& T, const tPointd& q, const tPointd& r, tPointd& intersectP, int* m);


//if any endpoint lies or if point of intersection of seg and plane is known, then use this function to find out if trinagle and segment intersect
    char intTri3D(const tFace& T, int m, const tPointd& p);

//project 3D to 2D
    char intTri2D(const tFace& T, const tPointd& p);

    void assign(const tPointd& first, tPointd& second);

//volume of tetrahedral
    double volume(const tPointd & T0, const tPointd& T1, const tPointd& T2, const tPointd& vertex);

//check if volume is positive, negative or zero
    int volumeSign(const tPointd& T0, const tPointd& T1, const tPointd& T2, const tPointd& vertex);

//for proper intersection between segment and plane, this function helps to check if triangle and segment intersects
    char segTriCross(const tFace& T, const tPointd& q, const tPointd& r);

// if the segment lies in plane use this to check for intersections
    char inPlane(const tFace& T, int m, const tPointd& q, const tPointd& r, tPointd& p);

// function to check if triangle and segment intersect in 3D space
    char intSegTri(const tFace& T, int m, const tPointd& q, const tPointd& r, tPointd& p);