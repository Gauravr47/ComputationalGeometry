#ifndef GEOMETRY_H	
#define GEOMETRY_H

#include <limits>
using namespace std;
#include "basic_data_structures.h"
using namespace cg;


namespace cg {

	#define min3(A,B,C) \
		((A)<(B)?((A)<(C)?(A):(C)):((B)<(C)?(B):(C))) 
	#define max3(A,B,C) \
		((A)>(B)?((A)>(C)?(A):(C)):((B)>(C)?(B):(C)))
	#define EPSILON1 1E-12
	#define EPSILON2 1E-10
	#define EPSILON3 1E-10

	enum {
		LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION
	};

	enum {
		CLOCKWISE, COUNTER_CLOCKWISE
	};

	enum {
		COLLINEAR, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS
	};

	enum {
		POSITIVE, NEGATIVE, ON
	};

	enum {
		INSIDE, OUTSIDE, BOUNDARY
	}; // point in polygon classification

	enum {
		TOUCHING, CROSSING, INESSENTIAL
	}; //edge classification

	enum {
		UPPER, LOWER
	}; //Vertex chain of a polygon

	enum {
		X, Y, Z
	};

	class Point;
	class Edge;
	class Vertex;
	class Polygon;
	class Point3D;
	class Edge3D;
	class Triangle3D;
	class rectangle;

	class Point {
	public:
		double x;
		double y;
		Point(double _x = 0.0, double _y = 0.0);
		friend Point operator+(Point&, Point&);
		friend Point operator-(Point&, Point&);
		friend Point operator*(double, Point&);
		double operator[](int);
		friend int operator==(Point&, Point&);
		friend int operator!=(Point&, Point&);
		friend int operator<(Point&, Point&);
		friend int operator>(Point&, Point&);
		int classify(Point&, Point&);
		int classify(Edge&);
		double polarAngle(void);
		double length(void);
		double distance(Edge&);
	};

	double dotProduct(Point&, Point&);

	class Vertex : public Node, public Point {
	public:
		Vertex(double, double);
		Vertex(Point&);
		Vertex* cw(void);
		Vertex* ccw(void);
		Vertex* neighbor(int);
		Point point(void);
		Vertex* insert(Vertex*);
		Vertex* remove(void);
		void splice(Vertex*);
		Vertex* split(Vertex*);
		friend class polygon;
	};

	class Polygon {
	private:
		Vertex* _v;
		int _size;
		void resize(void);
	public:
		Polygon(void);
		Polygon(Polygon&);
		Polygon(Vertex*);
		~Polygon(void);
		Vertex* insert(Point&);
		void remove();
		Vertex* v(void);
		Point point(void);
		Edge edge(void);
		int size();
		Vertex* cw(void);
		Vertex* ccw(void);
		Vertex* neighbor(int);
		Vertex* advance(int);
		Vertex* setV(Vertex*);
		Polygon* split(Vertex*);

	};

	class Edge {
	public:
		Point org;
		Point dest;
		Edge(Point&, Point&);
		Edge(void);
		Edge& rot(void);
		Edge& flip(void);
		Point point(double);
		int intersect(Edge&, double&);
		int cross(Edge&, double&);
		bool isVertical(void);
		double slope(void);
		double y(double);
	};

	class Point3D {
	public:
		double x;
		double y;
		double z;
		Point3D(double _x = 0.0, double _y = 0.0, double _z =0.0);
		friend Point3D operator+(Point3D&, Point3D&);
		friend Point3D operator-(Point3D&, Point3D&);
		friend Point3D operator*(double, Point3D&);
		double operator[](int);
		friend int operator==(Point3D&, Point3D&);
		friend int operator!=(Point3D&, Point3D&);
		friend int operator<(Point3D&, Point3D&);
		friend int operator>(Point3D&, Point3D&);
		int classify(Triangle3D&);
		double length(void);
	};

	double dotProduct(Point3D&, Point3D&);
	Point3D crossProduct(Point3D&, Point3D&);
	
	class Edge3D {
	public:
		Point3D org;
		Point3D dest;
		Edge3D(Point3D&, Point3D&);
		Edge3D(void);
		Point3D point(double);
		int intersect(Triangle3D&, double&);
	};

	class Triangle3D {
	private:
		Point3D _v[3];
		Edge3D _boundingBox;
		Point3D _n;
	public:
		int id;
		int mark;
		Triangle3D(Point3D&, Point3D&, Point3D&, int);
		Triangle3D(Point3D&, Point3D&, Point3D&, Point3D& n, int _id);
		Triangle3D(void);
		~Triangle3D(void);
		Point3D operator[](int);
		Edge3D boundingBox(void);
		Point3D n(void);
	};

	class Rectangle {
	public:
		Point sw; //bottom right corner
		Point ne; //top left corner
		int id; //some reference
		Rectangle(Point &_sw, Point &_ne, int _id=-1);
		~Rectangle(void) {};
	};

	bool pointInRectangle(Point& , Rectangle& );
	bool overlappingExtent(Rectangle&, Rectangle&, int);
	bool intersect(Rectangle&, Rectangle&);
}

#endif GEOMETRY_H