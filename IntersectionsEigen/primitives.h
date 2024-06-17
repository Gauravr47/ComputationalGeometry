#pragma once
#pragma once

#include <vector>
#include <set>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <exception>
#include <map>
#include <queue>
#include<unordered_map>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/geometry/geometries/register/segment.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <Eigen/Geometry>
using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

#ifdef STL_READER_NO_EXCEPTIONS
#define STL_READER_THROW(msg) return false;
#define STL_READER_COND_THROW(cond, msg) if(cond) return false;
#else
/// Throws an std::runtime_error with the given message.
#define STL_READER_THROW(msg) {std::stringstream ss; ss << msg; throw(std::runtime_error(ss.str()));}

/// Throws an std::runtime_error with the given message, if the given condition evaluates to true.
#define STL_READER_COND_THROW(cond, msg)  if(cond){std::stringstream ss; ss << msg; throw(std::runtime_error(ss.str()));}
#endif

using point2d = Eigen::Vector2f;
using point3d = Eigen::Vector3f;
using vector3d = Eigen::Vector3f;
using face =  Eigen::Matrix3f;
using faceIndex = Eigen::Vector3i;
using edge = std::vector<Eigen::Vector3f> ;
using edgeIndex = Eigen::Vector2i;
using box = boost::geometry::model::box<point3d>;

namespace boost {
    namespace serialization {

        template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
        inline void save(
            Archive& ar,
            const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
            const unsigned int version)
        {
            int rows = g.rows();
            int cols = g.cols();

            ar& rows;
            ar& cols;
            ar& boost::serialization::make_array(g.data(), rows * cols);
        }

        template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
        inline void load(
            Archive& ar,
            Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
            const unsigned int version)
        {
            int rows, cols;
            ar& rows;
            ar& cols;
            g.resize(rows, cols);
            ar& boost::serialization::make_array(g.data(), rows * cols);
        }

        template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
        inline void serialize(
            Archive& ar,
            Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
            const unsigned int version)
        {
            split_free(ar, g, version);
        }


    } // namespace serialization
} // namespace boost

BOOST_GEOMETRY_REGISTER_POINT_2D(point2d, float, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_3D(point3d, float, cs::cartesian, x(), y(), z())
BOOST_GEOMETRY_REGISTER_LINESTRING(edge)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(std::vector<edge>)


template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        // Note that it is oblivious to the storage order of Eigen matrix (column- or
        // row-major). It will give you the same hash value for two different matrices if they
        // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

string to_string(const point3d& p);

void fromEdgeStringToIndixes(string, vector<int>&);

string fromIndicesToEdgeString(int, int);

edgeIndex createEdge(int , int);

inline bool StlFileHasASCIIFormat(const char* filename)
{
    using namespace std;
    ifstream in(filename);
    STL_READER_COND_THROW(!in, "Couldnt open file " << filename);

    char chars[256];
    in.read(chars, 256);
    string buffer(chars, in.gcount());
    transform(buffer.begin(), buffer.end(), buffer.begin(), ::tolower);
    return buffer.find("solid") != string::npos &&
        buffer.find("\n") != string::npos &&
        buffer.find("facet") != string::npos &&
        buffer.find("normal") != string::npos;
};

void readBinarySTL(const char* filename, vector<point3d>& , vector<point3d>& , vector<faceIndex>& , unordered_map<edgeIndex, vector<int>, matrix_hash<edgeIndex>> & , std::vector<set<int>>&);

class Solid {
public:
    Solid() {};

    Solid(const char* filename) {
        readBinarySTL(filename, vertices, normals, faceIndices, edgeFaceAdj, shells);
    }

    void print() {
        for (const auto p : vertices) {
            cout << p;
        }
    }

    std::vector<edge> getEdges() {
        vector<edge> edges;
        for (auto edgeFaceEle : edgeFaceAdj) {
            edgeIndex edge = edgeFaceEle.first;
            edges.push_back(vector<point3d> {vertices[edge[0]], vertices[edge[1]]});
        }
        return edges;
    }

private:
    std::vector<point3d> vertices; 
    std::vector<point3d> normals;
    std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>> edgeFaceAdj;
    std::vector<faceIndex> faceIndices;
    std::vector<set<int>> shells;
    bgi::rtree<pair<box, unsigned int>, bgi::rstar<16, 4>> aabb;
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& vertices;
        ar& normals;
        ar& faceIndices;
        ar& shells;
        ar& edgeFaceAdj;
    }

};

template <typename T>
void Append(std::vector<T>& a, const std::vector<T>& b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
}

//eigen primitives
void slicingPlaneIntersection(const vector3d&, const point3d&, vector<edge>& , vector<point3d>& );

point3d segSegIntersectEigen(const point3d&, const point3d&, const point3d&, const point3d&);

char segPlaneIntersectEigen(const vector3d&, const point3d&, const point3d&, const point3d&, point3d&);
//old primitives
vector3d unitVec(const vector3d&);

vector3d unitVec(const point3d&, const point3d&);

void findSolids(std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>&, vector<faceIndex>&, vector<set<int>>&);

void bfs(std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>&, vector<faceIndex>&, set<int>&, vector<set<int>>&, vector<int>&);