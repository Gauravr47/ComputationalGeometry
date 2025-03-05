#pragma once

//Headers
#include <vector>
#include <set>
#include <unordered_set>
#include <iostream>
#include <cmath>
#include <fstream>
#include <exception>
#include <map>
#include <queue>
#include <unordered_map>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/geometry/geometries/register/segment.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/io/svg/write.hpp>
#include <boost/program_options.hpp>


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

# define M_PI           3.14159265358979323846
static float EPSILON= 1e-10;
static float PIbyTwo= 0.0;

using point2d = Eigen::Vector2f;
using point3d = Eigen::Vector3f;
using vector3d = Eigen::Vector3f;
using face =  Eigen::Matrix3f;
using faceIndex = Eigen::Vector3i;
using edge = std::vector<point3d>;
using edgeIndex = Eigen::Vector2i;
using box = Eigen::AlignedBox<float,3>;
using rtree = bgi::rtree<pair<box, unsigned int>, bgi::rstar<2, 1>>;
using ring = std::vector<point3d>;
using polygon3d = bg::model::polygon<point3d>;
using polygon2d = bg::model::polygon<point2d>;


BOOST_GEOMETRY_REGISTER_POINT_2D(point2d, float, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_3D(point3d, float, cs::cartesian, x(), y(), z())
BOOST_GEOMETRY_REGISTER_BOX(box, point3d, min(), max())
BOOST_GEOMETRY_REGISTER_LINESTRING(edge)


//template functions
template<typename T>
struct matrix_hash : std::function<size_t(T)> {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

template <typename T>
void Append(std::vector<T>& a, const std::vector<T>& b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
}

template <typename Geometry>
void create_svg(std::string const& file_path, Geometry const& geometry, std::string const& style) {
    std::ofstream svg_file(file_path.c_str());
    bg::svg_mapper<point2d> mapper(svg_file, 800, 400);
    mapper.add(geometry);
    mapper.map(geometry, style);
}

//Helpers
edgeIndex createEdge(int , int);

bool isCollinear(const point3d&, const point3d&, const point3d&);

vector3d unitVec(const vector3d&);

vector3d unitVec(const point3d&, const point3d&);

vector3d getPlaneNormal(float, float);

void updateMinMax(const point3d&, point3d&, point3d&);

bool isApproxEqual(const vector3d&, const vector3d&);

//define classes
class Solid;
class Contour;

//Intersection functions
void slicingPlaneIntersection(const vector3d&, const point3d&, vector<edge>& , vector<point3d>& );

point3d segSegIntersectEigen(const point3d&, const point3d&, const point3d&, const point3d&);

char segPlaneIntersectEigen(const vector3d&, const point3d&, const point3d&, const point3d&, point3d&);

void bfs_intersection(const vector3d&, std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>&, vector<faceIndex>&, std::vector<point3d>&, set<int>&, vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>>&, int);

vector<ring> project(const vector3d&, const point3d&, const vector3d&, Contour&);

vector<ring> project(const vector3d&, const point3d&, const vector3d&, vector<ring>&, Solid&);




