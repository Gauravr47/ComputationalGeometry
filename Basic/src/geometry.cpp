#include "geometry.h"
using namespace cg;

cg::Point::Point(double _x, double _y) : x(_x), y(_y) {
};

Point cg::operator+(Point& a, Point& b) {
	return Point(a.x + b.x, a.y + b.y);
};

Point cg::operator-(Point& a, Point& b) {
	return Point(a.x - b.x, a.y - b.y);
};

Point cg::operator*(double s, Point& a) {
	return Point(s * a.x, s * a.y);
};

double cg::Point::operator[](int i) {
	return (i == 0) ? x : y;
};

int cg::operator==(Point& a, Point& b) {
	return (a.x == b.x && a.y == b.y);
};

int cg::operator!=(Point& a, Point& b) {
	return abs(~(a == b));
};

int cg::operator<(Point& a, Point& b) {
	return (a.x < b.x) || ((a.x == b.x) && (a.y < b.y));
};

int cg::operator>(Point& a, Point& b) {
	return (a.x > b.x) || ((a.x == b.x) && (a.y > b.y));
};

int cg::Point::classify(Point& a, Point& b) {
	Point c = *this;
	Point ab = b - a;
	Point ac = c - a;
	double sa = ab.x * ac.y - ab.y * ac.x;
	if (sa > 0.0) {
		return LEFT;
	}
	if (sa < 0.0) {
		return RIGHT;
	}
	if ((ab.x * ac.x < 0.0) || ( ab.y * ac.y < 0.0)) {
		return BEHIND;
	}
	if (ab.length() < ac.length()) {
		return BEYOND;
	}
	if (a == c) {
		return ORIGIN;
	}
	if (b == c) {
		return DESTINATION;
	}
	return BETWEEN;
};

int cg::Point::classify(Edge& e) {
	return classify(e.org, e.dest);
};

double cg::Point::polarAngle(void) {
	if (x == 0.0 && y == 0.0) {
		return -1.0;
	}
	if (x == 0.0) {
		return ((y > 0.0) ? 90 : 270);
	}
	double theta = atan(y / x);
	theta *= 360 / (2 * 3.1415926);
	if (x > 0.0) {
		return ((y >= 0.0) ? theta : theta + 360);
	}
	return (theta + 180);
};

double cg::Point::length(void) {
	return sqrt(x * x + y * y);
}

double cg::Point::distance(Edge& e)
{
	Edge ab = e;
	ab.flip().rot();
	Point n = (ab.dest - ab.org);
	n = (1.0 / n.length()) * n;
	Edge f(*this, *this + n);
	double t;
	f.intersect(e, t);
	return t;
}

double cg::dotProduct(Point& a, Point& b) {
	return a.x * b.x + a.y * b.y;
}

/****************************************** Vertex ****************************************/
cg::Vertex::Vertex(double x, double y) :Point(x,y) {
};

cg::Vertex::Vertex(Point& a) :Point(a) {
};

Vertex* cg::Vertex::cw(void) {
	return (Vertex*)_next;
};

Vertex* cg::Vertex::ccw(void) {
	return (Vertex*)_prev;
};

Vertex* cg::Vertex::neighbor(int rotation) {
	return (rotation == CLOCKWISE) ? cw() : ccw();
};

Point cg::Vertex::point(void) {
	return *((Point*)this);
};

Vertex* cg::Vertex::insert(Vertex* v) {
	return (Vertex*)Node::insert(v);
};

Vertex* cg::Vertex::remove(void) {
	return (Vertex*)Node::remove();
};

void cg::Vertex::splice(Vertex* v) {
	Node::splice(v);
}

Vertex* cg::Vertex::split(Vertex* b)
{
	Vertex* bp = b->ccw()->insert(new Vertex(b->point()));
	insert(new Vertex(point()));
	splice(bp);
	return bp;
}

/****************************************** Polygon ****************************************/
void cg::Polygon::resize(void)
{
	if (_v == nullptr) {
		_size = 0;
	}
	else {
		Vertex* v = _v->cw();
		for (_size = 1; v != _v; _size++, v = v->cw());
	}
	return;
}

cg::Polygon::Polygon(void): _v(nullptr), _size(0)
{
}

cg::Polygon::Polygon(Polygon& p)
{
	_size = p.size();
	if (_size == 0) {
		_v = nullptr;
	}
	else {
		_v = new Vertex(p.point());
		for (int i = 1; i < _size; i++) {
			p.advance(CLOCKWISE);
			_v = _v->insert(new Vertex(p.point()));
		}
		p.advance(CLOCKWISE);
		_v = _v->cw();
	}
}

cg::Polygon::Polygon(Vertex* v): _v(v)
{
	resize();
}

cg::Polygon::~Polygon(void)
{
	Vertex* w = _v->cw();
	while (_v != w) {
		delete w->remove();
		w = _v->cw();
	}
	delete _v;
}

Vertex* cg::Polygon::insert(Point& p)
{
	if (_size++ == 0) {
		_v = new Vertex(p);
	}
	else {
		_v = _v->insert(new Vertex(p));
	}
	return _v;
}

void cg::Polygon::remove(void)
{
	Vertex* v = _v;
	_v = (--_size == 0) ? nullptr : _v->ccw();
	delete v->remove();
}

Vertex* cg::Polygon::v(void)
{
	return _v;
}

Point cg::Polygon::point(void)
{
	return _v->point();
}

Edge cg::Polygon::edge(void)
{
	return Edge(point(), _v->cw()->point());
}

int cg::Polygon::size()
{
	return _size;
}

Vertex* cg::Polygon::cw(void)
{
	return _v->cw();
}

Vertex* cg::Polygon::ccw(void)
{
	return _v->ccw();
}

Vertex* cg::Polygon::neighbor(int rotation)
{
	return _v->neighbor(rotation);
}

Vertex* cg::Polygon::advance(int rotation)
{
	return _v = _v->neighbor(rotation);
}

Vertex* cg::Polygon::setV(Vertex* v)
{
	return _v = v;
}

Polygon* cg::Polygon::split(Vertex* b)
{
	Vertex* bp = _v->split(b);
	resize();
	return new Polygon(bp);
}

/****************************************** Edge ****************************************/
cg::Edge::Edge(Point& Org, Point& Dest):org(Org), dest(Dest)
{
}

cg::Edge::Edge() :org(Point(0,0)), dest(Point(1,0))
{
}

Edge& cg::Edge::rot(void)
{
	Point m = 0.5 * (org + dest);
	Point v = dest - org;
	Point n(v.y, -v.x);
	org = m - 0.5 * n;
	dest = m + 0.5 * n;
	return *this;
}

Edge& cg::Edge::flip(void)
{
	return rot().rot();
}

Point cg::Edge::point(double t)
{
	return org + t* (dest - org);
}

int cg::Edge::intersect(Edge& e, double& t)
{
	Point a = org;
	Point b = dest;
	Point c = e.org;
	Point d = e.dest;
	Point n((d - c).y, (c - d).x);
	double denom = dotProduct(n, (b - a));
	if (denom == 0.0) {
		int aclass = org.classify(e);
		if (aclass == RIGHT || aclass == LEFT)
			return PARALLEL;
		else
			return COLLINEAR;
	}
	double num = dotProduct(n, (a - c));
	t = -num / denom;
	return SKEW;
}

int cg::Edge::cross(Edge& e, double& t)
{
	double s;
	int crossType = e.intersect(*this, s);
	if ((crossType == PARALLEL) || (crossType == COLLINEAR))
		return crossType;
	if (s < 0.0 || s > 1.0)
		return SKEW_NO_CROSS;
	intersect(e, t);
	if ((t >= 0.0) && (t <= 1.0))
		return SKEW_CROSS;
	else
		return SKEW_NO_CROSS;
}

bool cg::Edge::isVertical(void)
{
	return (org.x == dest.x);
}

double cg::Edge::slope(void)
{
	if (org.x != dest.x) {
		return (dest.y - org.y) / (dest.x - org.x);
	}
	return std::numeric_limits<double>::max();
}

double cg::Edge::y(double x)
{
	return slope() * (x - org.x) + org.y;
}

/****************************************** Point3D ****************************************/
cg::Point3D::Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z)
{
}

double cg::Point3D::operator[](int i)
{
	return (i == 0) ? x : ((i == 1) ? y : z);
}

int cg::Point3D::classify(Triangle3D& p)
{
	Point3D v = *this - p[0];
	double len = v.length();
	if (len == 0)
		return ON;
	v = (1.0 / len) * v;
	double d = dotProduct(v, p.n());
	if (d > EPSILON1)
		return POSITIVE;
	else if (d < -EPSILON1)
		return NEGATIVE;
	else
		return ON;
}

double cg::Point3D::length(void)
{
	return sqrt(x * x + y * y + z * z);
}

Point3D cg::operator+(Point3D& a, Point3D& b)
{
	return Point3D(a.x + b.x, a.y + b.y, a.z + b.z);
}

Point3D cg::operator-(Point3D& a, Point3D& b)
{
	return Point3D(a.x - b.x, a.y - b.y, a.z - b.z);
}

Point3D cg::operator*(double s, Point3D& a)
{
	return Point3D(s * a.x, s * a.y, s * a.z);
}

int cg::operator==(Point3D& a, Point3D& b)
{
	return  (a.x == b.x && a.y == b.y && a.z == b.z);
}

int cg::operator!=(Point3D& a, Point3D& b)
{
	return abs(~(a == b)) ;
}

int cg::operator<(Point3D& a, Point3D& b)
{
	return (a.x < b.x) || ((a.x == b.x) && (a.y < b.y)) || ((a.x == b.x) && (a.y == b.y) && (a.z < b.z));
}

int cg::operator>(Point3D& a, Point3D& b)
{
	return (a.x > b.x) || ((a.x == b.x) && (a.y > b.y)) || ((a.x == b.x) && (a.y == b.y) && (a.z > b.z));
}

double cg::dotProduct(Point3D& a, Point3D& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3D cg::crossProduct(Point3D& a, Point3D& b) {
	return Point3D(a.y * b.z + a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y + a.y * b.x
	);
}

/****************************************** Triangle3D ****************************************/
cg::Triangle3D::Triangle3D(Point3D& v0, Point3D& v1, Point3D& v2, int _id)
{
	id = _id;
	mark = 0;
	_v[0] = v0;
	_v[1] = v1;
	_v[2] = v2;
	_boundingBox.org.x = min3(v0.x, v1.x, v2.x);
	_boundingBox.org.y = min3(v0.y, v1.y, v2.y);
	_boundingBox.org.z = min3(v0.z, v1.z, v2.z);
	_boundingBox.dest.x = max3(v0.x, v1.x, v2.x);
	_boundingBox.dest.y = max3(v0.y, v1.y, v2.y);
	_boundingBox.dest.z = max3(v0.z, v1.z, v2.z);
	_n = crossProduct(v1 - v0, v2 - v0);
	_n = (1.0 / _n.length()) * _n;
}

cg::Triangle3D::Triangle3D(Point3D& v0, Point3D& v1, Point3D& v2, Point3D& n, int _id)
{
	id = _id;
	mark = 0;
	_v[0] = v0;
	_v[1] = v1;
	_v[2] = v2;
	_n = n;
	_boundingBox.org.x = min3(v0.x, v1.x, v2.x);
	_boundingBox.org.y = min3(v0.y, v1.y, v2.y);
	_boundingBox.org.z = min3(v0.z, v1.z, v2.z);
	_boundingBox.dest.x = max3(v0.x, v1.x, v2.x);
	_boundingBox.dest.y = max3(v0.y, v1.y, v2.y);
	_boundingBox.dest.z = max3(v0.z, v1.z, v2.z);
}

cg::Triangle3D::Triangle3D(void)
{
}

Point3D cg::Triangle3D::operator[](int i)
{
	return _v[i];
}

Edge3D cg::Triangle3D::boundingBox(void)
{
	return _boundingBox;
}

Point3D cg::Triangle3D::n(void)
{
	return _n;
}

/****************************************** Edge3D ****************************************/
cg::Edge3D::Edge3D(Point3D& _org, Point3D& _dest): org(_org), dest(_dest)
{
}

cg::Edge3D::Edge3D(void)
{
}

Point3D cg::Edge3D::point(double t)
{
	return org + t * (dest - org);
}

int cg::Edge3D::intersect(Triangle3D& p, double& t)
{
	Point3D a = org;
	Point3D b = dest;
	Point3D c = p[0];
	Point3D n = p.n();
	double denom = dotProduct(n, b - a);
	if (denom == 0.0) {
		int aclass = a.classify(p);
		if (aclass != ON)
			return PARALLEL;
		else
			return COLLINEAR;
	}
	double num = dotProduct(n, a - c);
	t = -num / denom;
	return SKEW;
}

/****************************************** Rectangle ****************************************/

Rectangle::Rectangle(Point& _sw, Point& _ne, int _id) : sw(_sw), ne(_ne), id(_id) {

}

bool cg::pointInRectangle(Point& p, Rectangle& R) {
	return ((R.sw.x <= p.x) && (p.x <= R.ne.x) &&
		(R.sw.y <= p.y) && (p.y <= R.ne.y));
}

bool cg::overlappingExtent(Rectangle& a, Rectangle& b, int i) {
	return ((a.sw[i] <= b.sw[i]) && (b.sw[i] <= a.ne[i])) || 
		((b.sw[i] <= a.sw[i]) && (a.sw[i] <= b.ne[i]));
}
bool cg::intersect(Rectangle& a, Rectangle& b) {
	return cg::overlappingExtent(a, b, X) && cg::overlappingExtent(a, b, Y);
}