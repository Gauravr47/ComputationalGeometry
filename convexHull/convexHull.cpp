#include"convexHull.h"

    // Copy the values from another point
void Point::Assign(const Point& other) {
    x = other.x;
    y = other.y;
    z = other.z;
};

    // Overloading the addition operator for point addition
Point Point::operator+(const Point& other) const {
    return { x + other.x, y + other.y, z + other.z };
};

    // Overloading the less-than operator for sorting points
bool Point::operator<(const Point& other) const {
    if (this->x < other.x) {
        return true;
    }
    else if (std::abs(this->x - other.x) <= 1e-7) {
        return this->y > other.y;
    }
    else {
        return false;
    }
};

    // Overloading the equality operator for point comparison
bool Point::operator==(const Point& other) const {
    return (x == other.x) && (y == other.y) && (z == other.z);
};


float Edge::Length() const {
    return std::sqrt(std::pow((v1.x - v2.x), 2) + std::pow((v1.y - v2.y), 2) + std::pow((v1.z - v2.z), 2));
};


// Function to determine if a point is on the left side of a directed line
bool IsLeft(const Point& p, const Point& q, const Point& r) {
    return (((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)) >= 0);
}

// Function to determine if a point is on the right side of a directed line
bool IsRight(const Point& p, const Point& q, const Point& r) {
    return (((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)) < 0);
}

// Function to compute the convex hull using the naive approach
std::vector<Edge> ConvexHull(const std::vector<Point>& input) {
    std::vector<Edge> allEdges;  // Vector to store all legitimate edges

    int np = input.size();

    // Iterate over all ordered pairs (p, q) in P x P with p != q
    for (int i = 0; i < np; ++i) {
        Point p(input[i]);

        for (int j = 0; j < np; ++j) {
            if (i != j) {
                Point q(input[j]);
                Edge temp(p, q);
                bool valid = true;

                // Iterate over all points r in P != p !! q
                for (const auto& r : input) {
                    if (!(r == p) && !(r == q)) {
                        // If r lies to the left of the directed line from p to q, set valid to false
                        if (IsLeft(temp.v1, temp.v2, r)) {
                            valid = false;
                        }
                    }
                }

                // If valid, add the directed edge pq to E
                if (valid) {
                    allEdges.push_back(temp);
                }
            }
        }
    }

    return allEdges;
}

// Function to compute the convex hull using the quick approach
std::vector<Point> ConvexHullFast(const std::vector<Point>& input) {
    std::vector<Point> upperHull;
    int n = input.size();

    // Put points p1 and p2 as the first two points in the upper hull
    upperHull.push_back(input[0]);
    upperHull.push_back(input[1]);

    // Iterate from i = 3 to n
    for (int i = 2; i < n; ++i) {
        // Append pi to the upper hull
        upperHull.push_back(input[i]);

        int sizeUpperHull = upperHull.size();

        // While the upper hull contains more than two points and the last three points do not make a right turn
        while (sizeUpperHull > 2 && !IsRight(*(upperHull.end() - 3), *(upperHull.end() - 2), *(upperHull.end() - 1))) {
            // Delete the middle of the last three points from the upper hull
            upperHull.erase(upperHull.end() - 2);
            sizeUpperHull = upperHull.size();
        }
    }

    std::vector<Point> lowerHull;

    // Put points pn and pn-1 in the lower hull with pn as the first point
    lowerHull.push_back(input[n - 1]);
    lowerHull.push_back(input[n - 2]);

    // Iterate from i = n-2 to 1
    for (int i = n - 3; i >= 0; --i) {
        // Append pi to the lower hull
        lowerHull.push_back(input[i]);

        int sizeLowerHull = lowerHull.size();

        // While the lower hull contains more than two points and the last three points do not make a right turn
        while (sizeLowerHull > 2 && !IsRight(*(lowerHull.end() - 3), *(lowerHull.end() - 2), *(lowerHull.end() - 1))) {
            // Delete the middle of the last three points from the lower hull
            lowerHull.erase(lowerHull.end() - 2);
            sizeLowerHull = lowerHull.size();
        }
    }

    // Remove the first and last point from the lower hull to avoid duplication
    lowerHull.erase(lowerHull.begin());

    // Append the lower hull to the upper hull to create the convex hull
    std::vector<Point> convexHull;
    convexHull.reserve(upperHull.size() + lowerHull.size());
    convexHull.insert(convexHull.end(), upperHull.begin(), upperHull.end());
    convexHull.insert(convexHull.end(), lowerHull.begin(), lowerHull.end());

    return convexHull;
}

// Function to convert edges to polygon points
std::vector<Point> FromEdgesToPolyPoints(std::vector<Edge>& allEdges) {
    std::vector<Point> hull;

    int size = allEdges.size();

    // If there are edges, start with the last edge's second vertex and build the hull
    if (size > 0) {
        hull.push_back(allEdges[size - 1].v1);
        hull.push_back(allEdges[size - 1].v2);
        Point enext(allEdges[size - 1].v2);
        allEdges.pop_back();

        // While there are still edges
        while (allEdges.size() > 0) {
            // Iterate through the remaining edges
            for (int i = 0; i < allEdges.size(); ++i) {
                // If the next point is found in the edges
                if (enext == allEdges[i].v1) {
                    // Update enext and add it to the hull
                    enext = allEdges[i].v2;
                    hull.push_back(enext);
                    // Remove the edge from the list
                    allEdges.erase(allEdges.begin() + i);
                    size = allEdges.size();
                    break;
                }
            }
        }
    }
    else {
        std::cout << "No points found. Something went wrong." << std::endl;
    }

    return hull;
}



std::ostream& operator<<(std::ostream& os, const Point& pt) {
    os << " " << pt.x << " " << pt.y << " " << pt.z;
    return os;
}


std::ostream& operator<<(std::ostream& os, const Edge& e) {
    os << " v1 " << e.v1.x << " " << e.v1.y << " " << e.v1.z << " v2 " << e.v2.x << " " << e.v2.y << " " << e.v2.z;
    return os;
}


