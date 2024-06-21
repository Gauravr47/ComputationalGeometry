#include "primitives.h"
#include "solid.h"
#include "contour.h"

//helpers

edgeIndex createEdge(int one, int two) {
    if (one > two) {
        int temp = std::move(one);
        one = std::move(two);
        two = std::move(temp);
    }
    return edgeIndex{one, two};
};

bool isCollinear(const point3d& p, const point3d& q, const point3d& r) {
    vector3d pq = q - p;
    vector3d pr = r - p;
    vector3d zero = vector3d::Zero();
    if (isApproxEqual(pq.cross(pr), zero)) {
        return true;
    }
    return false;
}

vector3d unitVec(const point3d& p, const point3d& q)
{
    return unitVec(p - q);
}

vector3d unitVec(const vector3d& pq)
{
    return pq / pq.norm();
}

vector3d getPlaneNormal(float alpha, float beta) {
    vector3d result;
    result[0] = cos(alpha * M_PI /180)* cos(beta * M_PI / 180);
    result[1] = sin(alpha * M_PI / 180)*cos(beta * M_PI / 180);
    result[2] = sin(beta * M_PI / 180);
    return result;
}

bool isApproxEqual(const vector3d& u, const vector3d& v) {
    return (fabs(u[0] - v[0]) < EPSILON) && (fabs(u[1] - v[1]) < EPSILON) && (fabs(u[2] - v[2]) < EPSILON);
}

void updateMinMax(const point3d& vert, point3d& minC, point3d& maxC) {

    minC[0] = min(minC[0], vert[0]);
    maxC[0] = max(maxC[0], vert[0]);
    minC[1] = min(minC[1], vert[1]);
    maxC[1] = max(maxC[1], vert[1]);
    minC[2] = min(minC[2], vert[2]);
    maxC[2] = max(maxC[2], vert[2]);
    return;
}


//Intersections
point3d segSegIntersectEigen(const point3d& p, const point3d& q, const point3d& r, const point3d& s) {
    using Line2 = Eigen::Hyperplane<float, 2>;
    Line2 pq = Line2::Through(p.head<2>(), q.head<2>());
    Line2 rs = Line2::Through(r.head<2>(), s.head<2>());
    point3d ans;
    ans << pq.intersection(rs), 0;
    return ans;
}

char segPlaneIntersectEigen(const vector3d& planeNormal, const point3d& planePoint, const point3d& p, const point3d& q, point3d& found) {

    float denom = planeNormal.dot(p - q);
    vector3d p0l0 = planePoint - p;
    float num = p0l0.dot(planeNormal);
    if (denom > 1e-6 || denom < -1e-6)       
    {
        float t = num / denom;
        found = (t * unitVec(p - q)) + p;
        return 'i';
    }
    else {
        if (denom > 1e-6 || denom < -1e-6) {
            found = p;
            return 'p';
        }
    }
    return '0';
}

void slicingPlaneIntersection(const vector3d& planeNormal, const point3d& planePoint, vector<edge>& edges, vector<point3d>& intersections) {
    float planeZ = planePoint.z();
    for (auto edge : edges) {
        float minZ = min(edge[0].z(), edge[1].z());
        float maxZ = max(edge[0].z(), edge[1].z());
        if (minZ <= planeZ  && planeZ<= maxZ) {
            point3d found;
            char res = segPlaneIntersectEigen(planeNormal, planePoint, edge[0], edge[1], found);
            if ( res != '0') {
                    intersections.push_back(found);

            }           
        }
    }
}

vector<unordered_set<edgeIndex, matrix_hash<edgeIndex>>> segSolidIntersection(const vector3d& ray, Solid& solid) {
    //for shells != internal
    //dot product of face normal and planeNormal
    //if >90, then casting shadow
    //get the edges, check for triangles next 
    //if in seen, remove common edge from edges output
    //
    vector<unordered_set<edgeIndex,matrix_hash<edgeIndex>>> edgesToProj;
    vector3d backRay = -1*ray;
    for(int i = 0; i < solid.shells.size(); i++) {
        if (!solid.internalGeometry[i]) {
            set<int> seen;
            for (auto& faceI: solid.shells[i]) {
                if (seen.find(faceI) == seen.end()) {
                    bfs_intersection(backRay, solid.edgeFaceAdj, solid.faceIndices, solid.normals, seen, edgesToProj, faceI);
                }
            }
        }
    }
    return edgesToProj;
}

void bfs_intersection(const vector3d& ray, std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>>& edgeFaceAdj, std::vector<faceIndex>& faceIndices, std::vector<point3d>& normals, std::set<int>& seen, std::vector<std::unordered_set<edgeIndex, matrix_hash<edgeIndex>>>& edgesToProj, int startFace) {
    queue<int> queue;
    unordered_set<edgeIndex, matrix_hash<edgeIndex>> edgesToProjSet;
    queue.push(startFace);

    while (!queue.empty()) {
        int currentLength = queue.size();
        set<int> toAdd;
        for (int i = 0; i < currentLength; i++) {
            int FI = queue.front();
            queue.pop();
            if (seen.find(FI) == seen.end()) {
                seen.insert(FI);
                double t = ray.dot(normals[FI]);
                if ((std::truncf(t* 1e10)/ 1e-10) <=PIbyTwo) {
                    faceIndex fIndex = faceIndices[FI];
                    for (int i = 0; i < (int)fIndex.size(); i++)
                    {
                        edgeIndex edgeI = createEdge(fIndex[i % fIndex.size()], fIndex[(i + 1) % fIndex.size()]);
                        //get the two faces and add edge only if one face is lit
                        if (edgeFaceAdj.find(edgeI) != edgeFaceAdj.end()) {
                            vector<int> newFaces = edgeFaceAdj[edgeI];
                            bool fullLit = true;
                            for (auto& newFace : newFaces) {
                                if (newFace != FI) {
                                    double t_ = ray.dot(normals[newFace]);
                                    bool halfLit = ((std::truncf(t_ * 1e10) / 1e-10) <= PIbyTwo);
                                    fullLit = fullLit && halfLit;
                                    if (!fullLit) {

                                        edgesToProjSet.insert(edgeI);
                                    }
                                    else {
                                        toAdd.insert(newFace);
                                    }
                                }
                            }
                            
                        }

                    }
                }
            }

        }
        for (auto t : toAdd) {
            queue.push(t);
        }
    }
    if(!edgesToProjSet.empty())
        edgesToProj.push_back(edgesToProjSet);
    return;
}

vector<ring> project(const vector3d& planeNormal, const point3d& planePoint, const vector3d& lightUnitVec, Contour& contour) {
    vector<ring> projRings;
    auto rings = contour.getRings();
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

vector<ring> project(const vector3d& planeNormal, const point3d& planePoint, const vector3d& lightUnitVec, vector<ring>& rings, Solid &solid) {
    vector<ring> projRings;
    vector<vector<double>> distances;
    for (int i = 0; i < rings.size(); i++) {
        ring temp;
        vector<double> dist_temp;
        for (auto p : rings[i]) {
            point3d found;
            char f = segPlaneIntersectEigen(planeNormal, planePoint, p, p + 1 * lightUnitVec, found);
            if (f != '0') {
                //ray trace from found to p with distance d, 
                //intersect with shells external, if distance is  <d, then collision, do not add
                float distance = (found - p).norm();
                cout << "\n" << found << " distance" << to_string(distance);
                temp.push_back(found);
                dist_temp.push_back(distance);
            }
        }
        if (!temp.empty()) {
            projRings.push_back(temp);
            distances.push_back(dist_temp);
        }
    }
    return projRings;
}
