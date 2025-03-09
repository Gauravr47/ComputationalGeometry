#include "solid.h"

Solid::Solid():num_tri(0), tris(new List<Triangle3D*>),_boundingBox(Point3D(DBL_MAX, DBL_MAX, DBL_MAX),Point3D(DBL_MIN, DBL_MIN, DBL_MIN)) {};

Solid::Solid(const char* filename):_boundingBox(Point3D(DBL_MAX, DBL_MAX, DBL_MAX), Point3D(DBL_MIN, DBL_MIN, DBL_MIN)) {
    tris = new List<Triangle3D*>;
    if (stlFileHasASCIIFormat(filename))
        readASCIISTL(filename);
    else
        readBinarySTL(filename);
}

void Solid::readBinarySTL(const char* filename)
{
    ifstream in(filename, ios::binary);
    STL_READER_COND_THROW(!in, "Couldnt open file " << filename);

    char stl_header[80];
    in.read(stl_header, 80);
    STL_READER_COND_THROW(!in, "Error while parsing binary stl header in file " << filename);

    in.read((char*)&num_tri, 4);
    STL_READER_COND_THROW(!in, "Couldnt determine number of triangles in binary stl file " << filename);

    unsigned int index = 0;
    for ( int tri = 0; tri < num_tri; ++tri) {
        float d[12];
        in.read((char*)d, 12 * 4);
        STL_READER_COND_THROW(!in, "Error while parsing trianlge in binary stl file " << filename);

        Point3D normal(d[0], d[1], d[2]);
        Point3D verts[3];
        for (size_t ivrt = 1; ivrt < 4; ++ivrt) {

            verts[ivrt-1] = Point3D(d[ivrt * 3 + 0], d[ivrt * 3 + 1], d[ivrt * 3 + 2]);
        }
        tris->append(new Triangle3D(verts[0], verts[1], verts[2], normal, tri));
        tris->next();
        _boundingBox.org.x = min(_boundingBox.org.x, tris->val()->boundingBox().org.x);
        _boundingBox.org.y = min(_boundingBox.org.y, tris->val()->boundingBox().org.y);
        _boundingBox.org.z = min(_boundingBox.org.z, tris->val()->boundingBox().org.z);
        _boundingBox.dest.x = max(_boundingBox.dest.x, tris->val()->boundingBox().dest.x);
        _boundingBox.dest.y = max(_boundingBox.dest.y, tris->val()->boundingBox().dest.y);
        _boundingBox.dest.z = max(_boundingBox.dest.z, tris->val()->boundingBox().dest.z);
        char addData[2];
        in.read(addData, 2);
        STL_READER_COND_THROW(!in, "Error while parsing additional triangle data in binary stl file " << filename);
    }
    tris->first();
    return;

}

void Solid::readASCIISTL(const char* filename)
{
    ifstream in(filename);
    STL_READER_COND_THROW(!in, "Couldn't open file " << filename);

    string buffer;
    vector<string> tokens;
    int lineCount = 1;
    int maxNumTokens = 0;

    //temp struct
    struct Triangle {
        List<Point3D*> vertices;
        Point3D normal;
    };
    Triangle* currT;
    while (!(in.eof() || in.fail()))
    {
        //  read the line and tokenize.
        //  In order to reuse memory in between lines, 'tokens' won't be cleared.
        //  Instead we count the number of tokens using 'tokenCount'.
        getline(in, buffer);
        istringstream line(buffer);
        int tokenCount = 0;
        while (!(line.eof() || line.fail())) {
            if (tokenCount >= maxNumTokens) {
                maxNumTokens = tokenCount + 1;
                tokens.resize(maxNumTokens);
            }
            line >> tokens[tokenCount];
            ++tokenCount;
        }

        if (tokenCount > 0)
        {
            string& tok = tokens[0];
            if (tok.compare("vertex") == 0) {
                if (tokenCount < 4) {
                    STL_READER_THROW("ERROR while reading from " << filename <<
                        ": vertex not specified correctly in line " << lineCount);
                }

                //  read the position
                Point3D* v = new Point3D(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
                currT->vertices.append(v);
            }
            else if (tok.compare("facet") == 0)
            {
                STL_READER_COND_THROW(tokenCount < 5,
                    "ERROR while reading from " << filename <<
                    ": triangle not specified correctly in line " << lineCount);

                STL_READER_COND_THROW(tokens[1].compare("normal") != 0,
                    "ERROR while reading from " << filename <<
                    ": Missing normal specifier in line " << lineCount);

                //  read the normal
                currT = new Triangle;
                currT->normal = Point3D(atof(tokens[2].c_str()), atof(tokens[3].c_str()), atof(tokens[4].c_str()));
                
            }
            else if (tok.compare("outer") == 0) {
                STL_READER_COND_THROW((tokenCount < 2) || (tokens[1].compare("loop") != 0),
                    "ERROR while reading from " << filename <<
                    ": expecting outer loop in line " << lineCount);
            }
            else if (tok.compare("endfacet") == 0) {
                STL_READER_COND_THROW(currT->vertices.length() != 3,
                    "ERROR while reading from " << filename <<
                    ": bad number of vertices specified for face in line " << lineCount);

                tris->append(new Triangle3D(*currT->vertices.val(), *currT->vertices.next(), *currT->vertices.next(), currT->normal, tris->length()+1));
                tris->next();
                _boundingBox.org.x = min(_boundingBox.org.x, tris->val()->boundingBox().org.x);
                _boundingBox.org.y = min(_boundingBox.org.y, tris->val()->boundingBox().org.y);
                _boundingBox.org.z = min(_boundingBox.org.z, tris->val()->boundingBox().org.z);
                _boundingBox.dest.x = max(_boundingBox.dest.x, tris->val()->boundingBox().dest.x);
                _boundingBox.dest.y = max(_boundingBox.dest.y, tris->val()->boundingBox().dest.y);
                _boundingBox.dest.z = max(_boundingBox.dest.z, tris->val()->boundingBox().dest.z);
                delete currT;
            }
            else if (tok.compare("solid") == 0) {
                //solidRangesOut.push_back(static_cast<index_t> (trisOut.size() / 3));
            }
        }
        lineCount++;
    }

    this->num_tri = tris->length();
    tris->first();
    return;
}

bool Solid::stlFileHasASCIIFormat(const char* filename)
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

List<Triangle3D*>* Solid::getTriangles() {
    return tris;
}

Edge3D Solid::getBoundingBox() {
    return _boundingBox;
}

int Solid::getNumberTriangles() {
    return num_tri;
}

Solid::~Solid() {
    delete tris;
}