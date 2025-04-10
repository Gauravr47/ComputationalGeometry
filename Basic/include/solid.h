#ifndef SOLID_H	
#define SOLID_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>
#include <algorithm>
#include <vector>
#include "basic_data_structures.h"
#include "geometry.h"
#include <memory>
using namespace cg;
using namespace std;

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
    Solid();

    Solid(const char*);

    //Getter - Setter
    void print();

    List<shared_ptr<Triangle3D>>* getTriangles();

    int getNumberTriangles();

    Edge3D getBoundingBox();

    //Mesh reading functions
    bool stlFileHasASCIIFormat(const char*);

    void readBinarySTL(const char*);

    void readASCIISTL(const char*);

    ~Solid();
private:
    List<shared_ptr<Triangle3D>>* tris;
    int num_tri;
    Edge3D _boundingBox;
    friend class Polygon;
};



#endif