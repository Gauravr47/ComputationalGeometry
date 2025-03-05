#include "solid.h"

Solid::Solid(const char* filename) {
    readBinarySTL(filename);
}

void Solid::print() {
    for (const auto p : vertices) {
        cout << p;
    }
}

void Solid::readBinarySTL(const char* filename)
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
        point3d minC{ (float)INT_MAX ,(float)INT_MAX ,(float)INT_MAX }, maxC{ (float)INT_MIN ,(float)INT_MIN ,(float)INT_MIN };
        for (size_t ivrt = 1; ivrt < 4; ++ivrt) {

            point3d vert(d[ivrt * 3 + 0], d[ivrt * 3 + 1], d[ivrt * 3 + 2]);
            updateMinMax(vert, minC, maxC);
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

        //octree.insert(make_pair(box(minC, maxC), tri));
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

    findShells();
    findInternalGeometry();

    return;

}

void Solid::findShells() {
    set<int> seen;
    int numSolids = 0;
    for (auto& edgefacesPair : edgeFaceAdj) {
        edgeIndex edge = edgefacesPair.first;
        vector<int> faces = edgefacesPair.second;
        if (seen.find(faces[0]) == seen.end() || seen.find(faces[1]) == seen.end()) {
            shells.push_back(set<int>{});
            box aabb = findShellHelper(seen, faces);
            aabbTree.insert(make_pair(aabb, numSolids++));
            internalGeometry.push_back(false);
        }
    }
    return;
}

box Solid::findShellHelper(set<int>& seen, vector<int>& currfaces) {
    queue<int> queue;
    point3d minC{ (float)INT_MAX ,(float)INT_MAX ,(float)INT_MAX }, maxC{ (float)INT_MIN ,(float)INT_MIN ,(float)INT_MIN };
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
                shells.back().insert(FI);
                faceIndex fIndex = faceIndices[FI];
                for (int i = 0; i < (int)fIndex.size(); i++)
                {
                    edgeIndex edge = createEdge(fIndex[i % fIndex.size()], fIndex[(i + 1) % fIndex.size()]);
                    point3d vert = vertices[fIndex[i]];
                    //find mix max to create a box
                    updateMinMax(vert, minC, maxC);
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
    box aabb(minC, maxC);
    return aabb;
}

bool Solid::StlFileHasASCIIFormat(const char* filename)
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

void Solid::findInternalGeometry() {
    for (auto const& v : aabbTree) {

        std::vector<std::pair<box, unsigned>> returned_values;
        std::copy(aabbTree.qbegin(bgi::contains(v.first)), aabbTree.qend(), std::back_inserter(returned_values));
        if (returned_values.size() % 2 == 0) {
            internalGeometry[v.second] = true;
        }
    }
}

std::vector<edge> Solid::getEdges() {
    vector<edge> edges;
    for (auto edgeFaceEle : edgeFaceAdj) {
        edgeIndex edge = edgeFaceEle.first;
        edges.push_back(vector<point3d> {vertices[edge[0]], vertices[edge[1]]});
    }
    return edges;
}

box Solid::getBoundingBox() {

    auto result = aabbTree.bounds();
    auto minC = result.min_corner();
    auto maxC = result.max_corner();
    return box(point3d{ minC.get<0>(), minC.get<1>(), minC.get<2>() }, point3d{ maxC.get<0>(), maxC.get<1>(), maxC.get<2>() });
}
