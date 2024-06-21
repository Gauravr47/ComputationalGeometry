#pragma once
#include "primitives.h"

//constants and type defs
#ifdef STL_READER_NO_EXCEPTIONS
#define STL_READER_THROW(msg) return false;
#define STL_READER_COND_THROW(cond, msg) if(cond) return false;
#else
/// Throws an std::runtime_error with the given message.
#define STL_READER_THROW(msg) {std::stringstream ss; ss << msg; throw(std::runtime_error(ss.str()));}

/// Throws an std::runtime_error with the given message, if the given condition evaluates to true.
#define STL_READER_COND_THROW(cond, msg)  if(cond){std::stringstream ss; ss << msg; throw(std::runtime_error(ss.str()));}
#endif

//classes
class Solid {
public:
    //constructors
    Solid() {};

    Solid(const char* );

    //Getter - Setter
    void print();

    std::vector<edge> getEdges();

    std::vector<point3d> getVertices();

    box getBoundingBox();

    //Mesh reading functions
    bool StlFileHasASCIIFormat(const char*);

    void readBinarySTL(const char*);

    void findShells();

    box findShellHelper(set<int>&, vector<int>&);

    void findInternalGeometry();

    void loadMesh(const char* filename);
    //friend methods

    friend vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> segSolidIntersection(const vector3d&, Solid&);

    friend vector<ring> project(const vector3d&, const point3d&, const vector3d&, vector<ring>&, Solid&);

private:
    std::vector<point3d> vertices; 
    std::vector<point3d> normals;
    std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>> edgeFaceAdj;
    std::vector<faceIndex> faceIndices;
    std::vector<set<int>> shells;
    std::vector<bool> internalGeometry;
    rtree aabbTree;
    rtree octree;
    
    friend class boost::serialization::access;
    friend class Contour;
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

