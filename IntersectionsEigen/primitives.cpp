#include "primitives.h"

string to_string(const point3d& p) {
    string s = to_string(p[0]) + "," + to_string(p[1]) + "," + to_string(p[2]);
    return s;
}

void fromEdgeStringToIndixes(string s, vector<int>& indices) {
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        indices.push_back(stoi(token));
        s.erase(0, pos + delimiter.length());
    }
    indices.push_back(stoi(s));
    return;
}

string fromIndicesToEdgeString(int one, int two)
{
    if (one < two) {
        return to_string(one) + "," + to_string(two);
    }
    else {
        return to_string(two) + "," + to_string(one);
    }
};

edgeIndex createEdge(int one, int two) {
    if (one > two) {
        int temp = std::move(one);
        one = std::move(two);
        two = std::move(temp);
    }
    return edgeIndex{one, two};
};

void readBinarySTL(const char* filename, vector<point3d>& vertices,  vector<point3d>& normals, vector<faceIndex>& faceIndices, unordered_map<edgeIndex, vector<int>, matrix_hash<edgeIndex>>& edgeFaceAdj, std::vector<set<int>>& solids)
{
    
    STL_READER_COND_THROW(StlFileHasASCIIFormat(filename), "Only binary files are supported " << filename);

    ifstream in(filename, ios::binary);
    STL_READER_COND_THROW(!in, "Couldnt open file " << filename);

    std::unordered_map<point3d, int, matrix_hash<point3d>> mp;
    char stl_header[80];
    in.read(stl_header, 80);
    STL_READER_COND_THROW(!in, "Error while parsing binary stl header in file " << filename);

    unsigned int numTris = 0;
    in.read((char*)&numTris, 4);
    STL_READER_COND_THROW(!in, "Couldnt determine number of triangles in binary stl file " << filename);

    unsigned int index = 0;
    
    for (unsigned int tri = 0; tri < numTris; ++tri) {
        float d[12];
        in.read((char*)d, 12 * 4);
        STL_READER_COND_THROW(!in, "Error while parsing trianlge in binary stl file " << filename);

        normals.push_back(point3d(d[0], d[1], d[2])); 
        faceIndex fIndex;
        for (size_t ivrt = 1; ivrt < 4; ++ivrt) {

            point3d vert(d[ivrt * 3 + 0], d[ivrt * 3 + 1], d[ivrt * 3 + 2]);
            //string vertS = to_string(vert);
            if (mp.find(vert) != mp.end()) {
                fIndex[ivrt - 1] = mp[vert];
            }
            else {
                vertices.push_back(vert);
                mp[vert] = index;
                fIndex[ivrt - 1] = index;
                index++;
            }
        }

        for (std::vector<int>::size_type i = 0; i < fIndex.size(); i++)
        {
            edgeIndex edge = createEdge(fIndex[i % fIndex.size()], fIndex[(i + 1) % fIndex.size()]);
            edgeFaceAdj[edge].push_back(tri);
        }

        faceIndices.push_back(fIndex);
        char addData[2];
        in.read(addData, 2);
        STL_READER_COND_THROW(!in, "Error while parsing additional triangle data in binary stl file " << filename);
    }

    findSolids(edgeFaceAdj, faceIndices, solids);
    return;
};

vector3d unitVec(const point3d& p, const point3d& q)
{
    return unitVec(p - q);
}

vector3d unitVec(const vector3d& pq)
{
    return pq / pq.norm();
}
//eigen primitives

point3d segSegIntersectEigen(const point3d& p, const point3d& q, const point3d& r, const point3d& s) {
    using Line2 = Eigen::Hyperplane<float, 2>;
    Line2 pq = Line2::Through(p.head<2>(), q.head<2>());
    Line2 rs = Line2::Through(r.head<2>(), s.head<2>());
    point3d ans;
    ans << pq.intersection(rs), 0;
    return ans;
}

char segPlaneIntersectEigen(const vector3d& planeNormal, const point3d& planePoint, const point3d& p, const point3d& q, point3d& found) {

    float denom = planeNormal.transpose() * (p - q);
    vector3d p0l0 = planePoint - p;
    float num = p0l0.dot(planeNormal);
    if (denom > 1e-6 || denom < -1e-6)       
    {
        float t = num / denom;
        found = (t * (p - q)) + p;
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

void findSolids(std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>> & edgeFaceAdj, vector<faceIndex>& faceIndices, vector<set<int>>& solids) {
    set<int> seen;
    int numSolids = 0;
    for (auto& edgefacesPair : edgeFaceAdj) {
        edgeIndex edge = edgefacesPair.first;
        vector<int> faces = edgefacesPair.second;
        if (seen.find(faces[0]) == seen.end() || seen.find(faces[1]) == seen.end()) {
            solids.push_back(set<int>{});
            numSolids++;
            bfs(edgeFaceAdj, faceIndices, seen, solids, faces);
        }
    }
    return;
}

void bfs(std::unordered_map<edgeIndex, std::vector<int>, matrix_hash<edgeIndex>> & edgeFaceAdj, vector<faceIndex>& faceIndices, set<int>& seen, vector<set<int>>& solids, vector<int>& currfaces) {
    queue<int> queue;
    for (auto FI : currfaces) {
        queue.push(FI);
    }
    while (!queue.empty()) {
        int currentLength = queue.size();
        set<int> toAdd;
        for (int i = 0; i < currentLength; i++) {
            int FI = queue.front();
            queue.pop();
            if (seen.find(FI) == seen.end()) {
                seen.insert(FI);
                solids.back().insert(FI);
                faceIndex fIndex = faceIndices[FI];
                for (int i = 0; i <(int) fIndex.size(); i++)
                {
                    edgeIndex edge = createEdge(fIndex[i % fIndex.size()], fIndex[(i+1) % fIndex.size()]);
                    if (edgeFaceAdj.find(edge) != edgeFaceAdj.end()) {
                        vector<int> newFace = edgeFaceAdj[edge];
                        for (auto newFI : newFace) {
                            if (seen.find(newFI) == seen.end()) {
                                toAdd.insert(newFI);
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
    return;
}