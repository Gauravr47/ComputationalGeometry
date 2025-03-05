#pragma once

#include "primitives.h"
#include "solid.h"

class Contour {
public:
    Contour() {};

    Contour(const vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>>);

    Contour(const vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>>, Solid&);

    vector<int> loopThrough(unordered_set<edgeIndex, matrix_hash<edgeIndex>>&);
    
    vector<vector<int>> getLoops();

    void setRings(Solid&);

    void findInternalGeometry();

    vector<ring> getRings();

    bool isInternalRing(int);

    vector<ring> project(const vector3d&, const point3d& , const vector3d& , Solid& );
    
private:
    vector<ring> rings;
    vector<ring> ProjRings;
    vector<vector<int>> loops;
    rtree aabbTree;
    std::vector<bool> internalGeometry;
    friend class Solid;
};




