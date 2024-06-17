#ifndef CONVEX_HULL_ALGORITHMS_H
#define CONVEX_HULL_ALGORITHMS_H

#include <vector>
#include <set>
#include <iostream>
#include <cmath>

// Class representing a 3D point
class Point {

public:

    double x{ 0.0 };
    double y{ 0.0 };
    double z{ 0.0 };

    // Constructors for different cases
    Point() : x(0.0), y(0.0), z(0.0) {}
    Point(double p, double q) : x(p), y(q), z(0.0) {}
    Point(double p, double q, double r) : x(p), y(q), z(r) {}
    
    void drawPoints();
    // Copy the values from another point
    void Assign(const Point&);

    // Overloading output stream operator for convenient printing
    friend std::ostream& operator<<(std::ostream& os, const Point& pt);

    // Overloading the addition operator for point addition
    Point operator+(const Point& other) const;

    // Overloading the less-than operator for sorting points
    bool operator<(const Point& other) const;

    // Overloading the equality operator for point comparison
    bool operator==(const Point& other) const;
};

// Class representing an edge connecting two points
class Edge {
public:
    Point v1;
    Point v2;

    // Constructors for different cases
    Edge() = default;
    Edge(const Point& p1, const Point& p2) : v1(p1), v2(p2) {}

    // Calculate the length of the edge
    float Length() const;

    // Overloading output stream operator for convenient printing
    friend std::ostream&operator<<(std::ostream& os, const Edge& e);
};

// Function to determine if a point is on the left side of a directed line
bool IsLeft(const Point& p, const Point& q, const Point& r);
// Function to determine if a point is on the right side of a directed line
bool IsRight(const Point& p, const Point& q, const Point& r);

// Function to compute the convex hull using the naive approach
std::vector<Edge> ConvexHull(const std::vector<Point>& input);

// Function to compute the convex hull using the quick approach
std::vector<Point> ConvexHullFast(const std::vector<Point>& input);
// Function to convert edges to polygon points
std::vector<Point> FromEdgesToPolyPoints(std::vector<Edge>& allEdges);

#endif // CONVEX_HULL_ALGORITHMS_H