#pragma once

#include <vector>
#include <set>
#include <unordered_set>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <exception>
#include <map>
#include <queue>
#include <unordered_map>
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
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/io/svg/write.hpp>
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

# define M_PI           3.14159265358979323846
static double PIbyTwo= 0.0;

using point2d = Eigen::Vector2f;
using point3d = Eigen::Vector3f;
using vector3d = Eigen::Vector3f;
using face =  Eigen::Matrix3f;
using faceIndex = Eigen::Vector3i;
using edge = std::vector<Eigen::Vector3f> ;
using edgeIndex = Eigen::Vector2i;
using box = Eigen::AlignedBox<float,3>;
using rtree = bgi::rtree<pair<box, unsigned int>, bgi::rstar<2, 1>>;
using ring = vector<point3d>;
using polygon3d = bg::model::polygon<point3d>;
using polygon2d = bg::model::polygon<point2d>;

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
BOOST_GEOMETRY_REGISTER_BOX(box, point3d, min(), max())



template<typename T>
struct matrix_hash : std::function<size_t(T)> {
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
        readBinarySTL(filename);
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

    bool StlFileHasASCIIFormat(const char*);

    void readBinarySTL(const char*);

    void findShells();

    box findShellHelper(set<int>&, vector<int>&);

    void findInternalGeometry();

    //friend class
    
    friend class Contour;
    //friend methods
    friend void segSolidIntersection(const point3d&, const point3d&, Solid&);

    friend vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> segSolidIntersection(const vector3d&, Solid&);


private:
    std::vector<point3d> vertices; 
    std::vector<point3d> normals;
    std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>> edgeFaceAdj;
    std::vector<faceIndex> faceIndices;
    std::vector<set<int>> shells;
    std::vector<bool> internalGeometry;
    rtree aabbTree;
    
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

class Contour {
public:
    Contour() {};
    Contour(const vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> edges) {

        for (auto& exLoop : edges) {
            //create a copy
            unordered_set<edgeIndex, matrix_hash<edgeIndex>> temp{ exLoop.begin(), exLoop.end() };

            while (!temp.empty()) {
                loops.push_back(loopThrough(temp));
            }
        }
    }

    Contour(const vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> edges, Solid& s) {
        
        for (auto& exLoop : edges) {
            //create a copy
            unordered_set<edgeIndex, matrix_hash<edgeIndex>> temp{exLoop.begin(), exLoop.end()};

            while (!temp.empty()) {
                loops.push_back(loopThrough(temp));
            }
        }
        setRings(s);
        findInternalGeometry();
    }

    vector<int> loopThrough(unordered_set<edgeIndex, matrix_hash<edgeIndex>>& edgeGroup) {
        vector<int> loop;
        auto edgeOne = *(edgeGroup.begin());
        loop.push_back(edgeOne[0]);
        loop.push_back(edgeOne[1]);
        int toFind = loop.back();
        edgeGroup.erase(edgeOne);
        while (loop.front() != loop.back() && !edgeGroup.empty()) {
            for (auto& edgeI : edgeGroup) {
                if (edgeI[0] == toFind) {
                    toFind = edgeI[1];
                    loop.push_back(toFind);
                    edgeGroup.erase(edgeI);
                    break;
                }
                if (edgeI[1] == toFind) {
                    toFind = edgeI[0];
                    loop.push_back(toFind);
                    edgeGroup.erase(edgeI);
                    break;
                }
            }
        }
        return loop;
    }
    
    vector<vector<int>> getLoops() {
        return this->loops;
    }

    void setRings(Solid& s) {
        for (auto loop : loops) {
            rings.push_back(ring{});
            point3d minC{ (float)INT_MAX ,(float)INT_MAX ,(float)INT_MAX }, maxC{ (float)INT_MIN ,(float)INT_MIN ,(float)INT_MIN };
            for (auto pIndex : loop) {
                point3d vert = s.vertices[pIndex];
                minC[0] = min(minC[0], vert[0]);
                maxC[0] = max(maxC[0], vert[0]);
                minC[1] = min(minC[1], vert[1]);
                maxC[1] = max(maxC[1], vert[1]);
                minC[2] = min(minC[2], vert[2]);
                maxC[2] = max(maxC[2], vert[2]);
                rings.back().push_back(vert);
            }
            aabbTree.insert(make_pair(box(minC, maxC),rings.size()-1));
            internalGeometry.push_back(false);
        }
    }

    void findInternalGeometry() {
        for (auto const& v : aabbTree) {

            std::vector<std::pair<box, unsigned>> returned_values;
            std::copy(aabbTree.qbegin(bgi::contains(v.first)), aabbTree.qend(), std::back_inserter(returned_values));
            if (returned_values.size() % 2 == 0) {
                internalGeometry[v.second] = true;
            }
        }
    }

    vector<ring> getRings() {
        return this->rings;
    }

    bool isInternalRing(int i) {
        if (i < rings.size()) {
            return internalGeometry[i];
        }
        return false;
    }

private:
    vector<ring> rings;
    vector<vector<int>> loops;
    rtree aabbTree;
    std::vector<bool> internalGeometry;

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

void bfs_intersection(const vector3d&, std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>&, vector<faceIndex>&, std::vector<point3d>&, set<int>&, vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>>&, int);

vector<ring> project(const vector3d&, const point3d&, const vector3d&, Contour&);

//old primitives
vector3d unitVec(const vector3d&);

vector3d unitVec(const point3d&, const point3d&);

vector3d getPlaneNormal(float, float, float);

void findSolids(std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>&, vector<faceIndex>&, vector<set<int>>&);

void bfs(std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>&, vector<faceIndex>&, set<int>&, vector<set<int>>&, vector<int>&);


template <typename Geometry>
void create_svg(std::string const& file_path, Geometry const& geometry, std::string const& style) {
    std::ofstream svg_file(file_path.c_str());
    bg::svg_mapper<point2d> mapper(svg_file, 400,400);
    mapper.add(geometry);
   mapper.map(geometry, style);
}