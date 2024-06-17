#include "primitives.h"
// Copy the values from another tPointd

std::ostream& operator<<(std::ostream& os, const tPointd& p)
{
    // TODO: insert return statement here
    for(double var : p)
    {
        os << " " << var;
    }
    return os;
}

float areaTriangle2D(const tPointd& p, const tPointd& q, const tPointd& r) {
    return (q[X] - p[X]) * (r[Y] - p[Y]) - (q[Y] - p[Y]) * (r[X] - p[X]);
}

// Function to determine if a tPointd is on the left side of a directed line
bool isLeft(const tPointd& p, const tPointd& q, const tPointd& r) {
    return areaTriangle2D(p, q, r) > 0;
}

// Function to determine if a tPointd is on the left side of or on the directed line
bool isLeftOn(const tPointd& p, const tPointd& q, const tPointd& r) {
    return areaTriangle2D(p, q, r) >= 0;
}

// Function to determine if a tPointd is on the left side of a directed line
bool isCollinear(const tPointd& p, const tPointd& q, const tPointd& r) {
    return areaTriangle2D(p, q, r) == 0;
}

// Function to determine if a tPointd is on the right side of a directed line
bool isRight(const tPointd& p, const tPointd& q, const tPointd& r) {
    return areaTriangle2D(p, q, r) < 0;
}

// Function to determine if a tPointd is between two points. All three are collinear
bool isBetween(const tPointd& p, const tPointd& q, const tPointd& r) {
    if (!isCollinear(p, q, r)) {
        return false;
    }
    else if (p[X] != q[X]) {
        return ((p[X] <= r[X])&&(r[X] <=q[X])) || ((p[X] >= r[X]) && (r[X] >= q[X]));
    }
    else {
        return ((p[Y] <= r[Y]) && (r[Y] <= q[Y])) || ((p[Y] >= r[Y]) && (r[Y] >= q[Y]));
    }
}
// Function to determine if two lines intersect properly
bool intersectProp(const tPointd& p, const tPointd& q, const tPointd& r, const tPointd& s) {
    if (isCollinear(p, q, r) ||
        isCollinear(p, q, s) ||
        isCollinear(r, s, p) ||
        isCollinear(r, s, q)) {
        return false;
    }
    return Xor(isLeft(p, q, r), isLeft(p, q, s)) && Xor(isLeft(r, s, p), isLeft(r, s, q));

}

// Function to determine if two lines intersect properly
bool intersect(const tPointd& p, const tPointd& q, const tPointd& r, const tPointd& s) {
    if (intersectProp(p, q, r, s))
        return true;
    else if (isBetween(p, q, r) || 
            isBetween(p, q, s) || 
            isBetween(r, s, p) || 
            isBetween(r, s, q)) {
        return true;
    }
    else
        return false;
}

//intersection using parametric formula and gives intersection points. Here we can overload the intersect function using 
//return type and number of variables but since return types are different, it can lead misunderstandings at different points in code
//We can load it within another interest function to keep it familiar
char segSegIntersect(const tPointd& p, const tPointd& q, const tPointd& r, const tPointd& s, tPointd& found) {
    char intersectType = '?';
    double num, denom, A, B; // using double instead of float to handle overflow of float buffers

    //line representation in parameteric form p +A (q-p) & r + B (s-r) 
    denom = p[X] * (s[Y] - r[Y])
        + q[X] * (r[Y] - s[Y])
        + r[X] * (p[Y] - q[Y])
        + s[X] * (q[Y] - p[Y]);

    if (denom == 0.0f)
        return parallelSegInt(p, q, r, s, found);

    //num for U
    num = p[X] * (s[Y] - r[Y])
        + r[X] * (p[Y] - s[Y])
        + s[X] * (r[Y] - p[Y]);
    if (num == 0.0f || num == denom) intersectType = 'v';
    A = num / denom;

    //num for V
    num = p[X] * (r[Y] - q[Y])
        + q[X] * (p[Y] - r[Y])
        + r[X] * (q[Y] - p[Y]);
    if (num == 0.0f || num == denom) intersectType = 'v';
    B = num / denom;

    if (0.0f <= A && A <= 1.0 && 0.0f <= B && B <= 1.0)
        intersectType = '1';
    else if (0.0f > A || A > 1.0 || 0.0f > B || B > 1.0)
        intersectType = '0';

    found[X] = p[X] + A * (q[X] - p[X]);
    found[Y] = p[Y] + A * (q[Y] - p[Y]);
    return intersectType;
}

//xor function
bool Xor(bool first, bool second) {
    return !first ^ !second;
}

char 
parallelSegInt(const tPointd& p,  const tPointd& q, const tPointd& r, const tPointd& s, tPointd& found) {
    if (isBetween(p, q, r))
        assign(r,found); return 'e';
    if (isBetween(p, q, s))
        assign(s, found); return 'e';
    if (isBetween(r, s, p))
        assign(p, found); return 'e';
    if (isBetween(r, s, q))
        assign(q, found); return 'e';
    return '0';
}

int planecCoeff(const tPointd& p, const tPointd& q, const tPointd& r, tPointd& N, double& D) {

    int m = 0; //index of largest of normal vector
    double biggest = 0.0;
    double temp;
    //find normal and distance
    normalVec(p, q, r, N);
    D = Dot(p, N);
    for (int i = 0; i < DIM; i++) {
        temp = fabs(N[i]);
        if (temp > biggest) {
            biggest = temp;
            m = i;
        }
    }

    return m;
}

void normalVec(const tPointd& p, const tPointd& q, const tPointd& r, tPointd& N)
{
    N[X] = (r[Z] - p[Z]) * (q[Y] - p[Y]) - (q[Z] - p[Z]) * (r[Y] - p[Y]);
    N[Y] = (r[X] - p[X]) * (q[Z] - p[Z]) - (q[X] - p[X]) * (r[Z] - p[Z]);
    N[Z] = (r[Y] - p[Y]) * (q[X] - p[X]) - (q[Y] - p[Y]) * (r[X] - p[X]);
}

double Dot(const tPointd& p, const tPointd& q)
{
    double sum = 0.0f;
    for (int i = 0; i < DIM; i++) {
        sum += p[i] * q[i];
    }
    return sum;
}

void subVec(const tPointd& p, const tPointd& q, tPointd& r)
{
    for (int i = 0; i < DIM; i++) {
        r[i] = p[i] - q[i];
    }
}

char segPlaneIntersect(const tFace& T, const tPointd& q, const tPointd& r, tPointd& intersectP, int* m)
{
    double D, num, denom, t;
    tPointd N, qr;

    *m = planecCoeff(T[0], T[1], T[2], N, D);
    subVec(r, q, qr);
    num = D - Dot(q, N);
    denom = Dot(qr, N);

    if (denom == 0) {
        if (num == 0) {
            return 'p';
        }
        else {
            return '0';
        }
    }else
        t = num / denom;

    for (int i = 0; i < DIM; i++) {
        intersectP[i] = q[i] + t * qr[i];
    }
    if ((0.0f < t) && (t < 1.0)) {
        return '1';
    }
    else if (num == 0.0f) {
        return 'q';
    }
    else if (num == denom) {
        return 'r';
    }else
        return '0';
}

char intTri3D(const tFace& T, int m, const tPointd& p)
{
    tPointd pProj;
    tFace tProj;
    int i, j, k;

    //project both triangle and point on plane closest to plane of triangle
    j = 0;
    for (i = 0; i < DIM; i++) {
        if (i != m) {
            pProj[j] = p[i];
            for (k = 0; k < DIM; k++) {
                tProj[k][j] = T[k][i];
            }
            j++;
        }
    }

    return intTri2D(tProj, pProj);
}

char intTri2D(const tFace& T, const tPointd& p)
{
    double area0, area1, area2;
    area0 = areaTriangle2D(p, T[0], T[1]);
    area1 = areaTriangle2D(p, T[1], T[2]);
    area2 = areaTriangle2D(p, T[2], T[1]);

    if (((area0 == 0.0f) && (area1 > 0) && (area2 > 0)) || ((area1 == 0.0f) && (area2 > 0) && (area0 > 0)) || ((area2 == 0.0f) && (area1 > 0) && (area0 > 0)))
        return 'E';
    if (((area0 == 0.0f) && (area1 < 0) && (area2 > 0)) || ((area1 == 0.0f) && (area2 > 0) && (area0 < 0)) || ((area2 == 0.0f) && (area1 > 0) && (area0 < 0)))
        return 'E';
    if (((area0 > 0) && (area1 > 0) && (area2 > 0)) || ((area0 < 0) && (area1 < 0) && (area2 < 0)))
        return 'F';
    if (area0 == 0.0f && area1 == 0.0f && area2 == 0.0f)
        fprintf(stderr, "Error in intTri2D\n"), exit(EXIT_FAILURE);
    if ((area0 == 0.0f && area1 == 0.0f) || (area1 == 0.0f && area2 == 0.0f) || (area0 == 0.0f && area2 == 0.0f))
        return 'V';

    return '0';
}

void assign(const tPointd& first, tPointd& second)
{
    second[X] = first[X];
    second[Y] = first[Y];
    second[Z] = first[Z];
}

double volume(const tPointd& T0, const tPointd& T1, const tPointd& T2, const tPointd& vertex)
{
    double vol;
    double ax, ay, az, bx, by, bz, cx, cy, cz;

    ax = T0[X] - vertex[X];
    ay = T0[Y] - vertex[Y];
    az = T0[Z] - vertex[Z];
    bx = T1[X] - vertex[X];
    by = T1[Y] - vertex[Y];
    bz = T1[Z] - vertex[Z];
    cx = T2[X] - vertex[X];
    cy = T2[Y] - vertex[Y];
    cz = T2[Z] - vertex[Z];
    vol = ax * (by * cz - bz * cy)
        + ay * (bz * cx - bx * cz)
        + az * (bx * cy - by * cx);
    return vol;
}

int volumeSign(const tPointd& T0, const tPointd& T1, const tPointd& T2, const tPointd& vertex)
{
    double vol = volume(T0,T1, T2, vertex);
    if (vol > 0.5)
        return 1;
    else if (vol < -0.5)
        return -1;
    else
        return 0;
}


char segTriCross(const tFace& T, const tPointd& q, const tPointd& r)
{
    int vol0, vol1, vol2;
    vol0 = volumeSign(q, T[0], T[1], r);
    vol1 = volumeSign(q, T[1], T[2], r);
    vol2 = volumeSign(q, T[2], T[0], r);
    if (((vol0 < 0) && (vol1 < 0) && (vol2 < 0)) ||
        ((vol0 > 0) && (vol1 > 0) && (vol2 > 0)))
        return 'f';

    if (((vol0 < 0) || (vol1 < 0) || (vol2 < 0)) &&
        ((vol0 > 0) || (vol1 > 0) || (vol2 > 0)))
        return '0';
    else if ((vol0 == 0) && (vol1 == 0) && (vol2 == 0))
        fprintf(stderr, "Error 1 in segTriCross\n"), exit(EXIT_FAILURE);
    else if ((vol0 == 0) && (vol1 == 0) ||
        (vol0 == 0) && (vol2 == 0) ||
        (vol2 == 0) && (vol1 == 0))
        return 'v';
    else if ((vol0 == 0) || (vol1 == 0) || (vol2 == 0))
        return 'e';
    else
        fprintf(stderr, "Error 2 in segTriCross\n"), exit(EXIT_FAILURE);

}

char inPlane(const tFace& T, int m, const tPointd& q, const tPointd& r, tPointd& p)
{
    tPointd qProj, rProj;
    tFace tProj;
    int i, j, k;

    //project both triangle and point on plane closest to plane of triangle
    j = 0;
    for (i = 0; i < DIM; i++) {
        if (i != m) {
            qProj[j] = q[i];
            rProj[j] = r[i];
            for (k = 0; k < DIM; k++) {
                tProj[k][j] = T[k][i];
            }
            j++;
        }
    }

    char qT = intTri2D(T, qProj);
    if (qT != '0')
    {
        assign(q, p);
        return qT;

    }
    char rT = intTri2D(T, rProj);
    
    if ( rT != '0') {
        assign(r, p);
        return rT;
    }
    else {
        for (int i = 0; i < DIM; i++) {
            char seg_tri = segSegIntersect(qProj, rProj, tProj[i], tProj[(i + 1) % DIM], p);
            if (seg_tri != 0) {
                //p here is projected on X, Y plane. need to Proj Back to original which needs plane Normal information
                return seg_tri;
            }
        }
           
    }
}

char intSegTri(const tFace& T, int m, const tPointd& q, const tPointd& r, tPointd& p)
{
    int code;
    code = segPlaneIntersect(T, q, r, p , &m);

    if (code == 'q')
        return intTri3D(T, m, q);
    else if (code == 'r')
        return intTri3D(T, m, r);
    else if (code == 'p')
        return inPlane(T, m, q, r, p);
    else
        return segTriCross(T, q, r);
}



