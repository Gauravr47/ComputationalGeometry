#include "contour.h"

Contour::Contour(const vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> edges) {

    for (auto& exLoop : edges) {
        //create a copy
        unordered_set<edgeIndex, matrix_hash<edgeIndex>> temp{ exLoop.begin(), exLoop.end() };

        while (!temp.empty()) {
            loops.push_back(loopThrough(temp));
        }
    }
}

Contour::Contour(const vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> edges, Solid& s) {

    for (auto& exLoop : edges) {
        //create a copy
        unordered_set<edgeIndex, matrix_hash<edgeIndex>> temp{ exLoop.begin(), exLoop.end() };

        while (!temp.empty()) {
            loops.push_back(loopThrough(temp));
        }
    }
    setRings(s);
    findInternalGeometry();
}


vector<int> Contour::loopThrough(unordered_set<edgeIndex, matrix_hash<edgeIndex>>& edgeGroup) {
    vector<int> loop;
    auto edgeOne = *(edgeGroup.begin());
    loop.push_back(edgeOne[0]);
    loop.push_back(edgeOne[1]);
    int toFind = loop.back();
    edgeGroup.erase(edgeOne);
    int run = edgeGroup.size();
    while (loop.front() != loop.back() && !edgeGroup.empty() && run >= 0) {
        run--;
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

vector<vector<int>> Contour::getLoops() {
    return this->loops;
}

void Contour::setRings(Solid& s) {
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
            while (rings.back().size() > 2) {
                point3d backOne = rings.back().back();
                rings.back().pop_back();
                point3d backtwo = rings.back().back();
                if (!isCollinear(backtwo, backOne, vert)) {
                    rings.back().push_back(backOne);
                    break;
                }
            }
            rings.back().push_back(vert);
        }
        aabbTree.insert(make_pair(box(minC, maxC), rings.size() - 1));
        internalGeometry.push_back(false);
    }
}


void Contour::findInternalGeometry() {
    for (auto const& v : aabbTree) {

        std::vector<std::pair<box, unsigned>> returned_values;
        std::copy(aabbTree.qbegin(bgi::contains(v.first)), aabbTree.qend(), std::back_inserter(returned_values));
        if (returned_values.size() % 2 == 0) {
            internalGeometry[v.second] = true;
        }
    }
}

vector<ring> Contour::getRings() {
    return this->rings;
}


bool Contour::isInternalRing(int i) {
    if (i < rings.size()) {
        return internalGeometry[i];
    }
    return false;
}

vector<ring> Contour::project(const vector3d& planeNormal, const point3d& planePoint, const vector3d& lightUnitVec, Solid& solid) {
    vector<ring> projRings;
    for (int i = 0; i < rings.size(); i++) {
        ring temp;
        for (auto p : rings[i]) {
            point3d found;
            char f = segPlaneIntersectEigen(planeNormal, planePoint, p, p + 1 * lightUnitVec, found);
            if (f != '0') {
                //ray trace from found to p with distance d, 
                //intersect with shells external, if distance is  <d, then collision, do not add

                temp.push_back(found);
            }
        }
        if (!temp.empty()) {
            projRings.push_back(temp);
        }
    }
    return projRings;
}