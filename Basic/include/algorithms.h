#pragma once
#include "geometry.h"
#include "list.h"
#include "stack.h"
#include "tree.h"
#include "utilFuncs.h"
using namespace cg;

namespace cg {

	namespace poly {
		enum { UNKNOWN, P_IS_INSIDE, Q_IS_INSIDE };
		enum { ACTIVE_POINT, ACTIVE_EDGE };
		enum { START_TYPE, BEND_TYPE, END_TYPE };
		enum { LEFT_SIDE, RIGHT_SIDE, BOTTOM_SIDE, TOP_SIDE };
		enum { LEFT_TO_RIGHT, RIGHT_TO_LEFT };
		extern Point hullCmpPoint;
		extern Point polarCmpPoint;
		extern double slCurrx;
		extern int monoSweepDirection;
		extern double monoCurrx;
		extern int monoCurrType;

		class EventPoint {
		public:
			Point p;
			virtual void handleTransition(Dictionary<Edge*>&, Dictionary<EventPoint*>&, List<EventPoint*>*) = 0;
		};

		class LeftEndpoint : public EventPoint {
		public:
			Edge e;
			LeftEndpoint(Edge* E) :e(*E) {
				p = (e.org < e.dest) ? e.org : e.dest;
			};
			void handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<EventPoint*>& schedule, List<EventPoint*>* result);
		};

		class RightEndpoint : public EventPoint {
		public:
			Edge e;
			RightEndpoint(Edge* E) :e(*E) {
				p = (e.org > e.dest) ? e.org : e.dest;
			};
			void handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<EventPoint*>& schedule, List<EventPoint*>* result);
		};

		class CrossingPoint : public EventPoint {
		public:
			Edge e1, e2;
			CrossingPoint(Edge* E1, Edge* E2, Point& P) :e1(*E1), e2(*E2) {
				p = P;
			};
			void handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<EventPoint*>& schedule, List<EventPoint*>* result);
		};

		class AxesParallelEdge {
		public:
			Rectangle* r;
			int type;
			int count;
			double m;
			double pos(void);
			double min(void);
			double max(void);
			AxesParallelEdge(Rectangle* _r, int _type);
			void setMin(double);
			void handleLeftEdge(Dictionary<AxesParallelEdge*>&, List<Edge*>*);
			void handleRightEdge(Dictionary<AxesParallelEdge*>&, List<Edge*>*);
		};

		class ActiveElement {
		public:
			int type;
			ActiveElement(int _type) :type(_type) {};
			virtual double y() = 0;
			virtual Edge edge() { return Edge(); };
			virtual double slope() { return 0.0; };
		};

		class ActiveEdge : public ActiveElement {
		public:
			Vertex* v;
			Vertex* w;
			int rotation;
			ActiveEdge(Vertex* _v, int _rotation, Vertex* _w) : v(_v), rotation(_rotation), w(_w), ActiveElement(ACTIVE_EDGE) {};
			double y();
			Edge edge();
			double slope();
		};

		class ActivePoint : public ActiveElement {
		public:
			Point p;
			ActivePoint(Point& _p) : p(_p), ActiveElement(ACTIVE_POINT) {};
			double y();
		};

		//plane sweeps
		List<EventPoint*>* intersectSegments(Edge[], int);
		Dictionary<EventPoint*>& buildSchedule(Edge[], int);
		List<Edge*>* findContour(Rectangle[], int);
		AxesParallelEdge** buildSchedule(Rectangle[], int);
		int AxesParallelEdgeCmp(AxesParallelEdge*, AxesParallelEdge*);
		List<Polygon*>* regularize(Polygon&);
		void semiregulaize(Polygon&, int direction, List<Polygon*>* poly);
		Vertex** buildSchedule(Polygon&, int(*cmp)(Vertex*, Vertex*));
		int typeEvent(Vertex*, int(*cmp)(Vertex*, Vertex*));
		int activeElementCmp(ActiveElement*, ActiveElement*);
		void monoStartTransition(Vertex*, Dictionary<ActiveElement*>&);
		void monoBendTransition(Vertex*, Dictionary<ActiveElement*>&);
		void monoEndTransition(Vertex*, Dictionary<ActiveElement*>&, List<Polygon*>*);

		//single polygon algorithms
		bool pointInConvexPolygon(Point&, Polygon&);
		Vertex* leastVertex(Polygon&, int(*cmp)(Point*, Point*));
		Polygon* starPolygon(Point[], int);
		int polarCmp(Point*, Point*);
		Polygon* insertionHull(Point[], int);
		Polygon* insertionHull2(Point[], int);
		void supportingLine(Point&, Polygon*, int);
		int closestToPolygonCmp(Point*, Point*);
		int pointInPolygon(Point&, Polygon&);
		int edgeType(Point&, Edge&);
		double signedAngle(Point&, Edge&);
		int pointInPolygon2(Point&, Polygon&);
		bool clipLineSegment(Edge&, Polygon&, Edge&); //Cyrus-beck clipping algorithm, clip line segment to a polygon, result is line segment
		bool clipPolygon(Polygon&, Polygon&, Polygon*&); //clip one polygon to another one
		bool clipPolygonToEdge(Polygon&, Edge&, Polygon*&);
		List<Polygon*>* triangulateMonotonePolygon(Polygon& p); // triangulation of polygon which is already monotone.
		int advancePtr(Vertex*, Vertex*, Vertex*);
		bool adjacent(Vertex*, Vertex*);
		void triangulateFanPolygon(Polygon&, List<Polygon*>*);
		//incremental selection methods
		Polygon* giftWrapHull(Point[], int);
		Polygon* grahamScanHull(Point[], int);
		Polygon* convexPolygonIntersect(Polygon&, Polygon&);
		bool aimsAt(Edge&, Edge&, int, int);
		int crossingPoint(Edge&, Edge&, Point&);
		void advance(Polygon&, Polygon&, int);
		List<Polygon*>* delaunayTriangulate(Point[], int);
		Edge* hullEdge(Point[], int);
		int edgeCmp(Edge*, Edge*);
		int edgeCmp2(Edge*, Edge*);
		int eventCmp(EventPoint*, EventPoint*);
		bool findMate(Edge, Point[], int, Point&);
		Polygon* triangle(Point&, Point&, Point&);
		void updateFrontier(Dictionary<Edge*>&, Point&, Point&);
		bool isConvex(Vertex*);
		Polygon* halfPlaneIntersect(Edge[], int, Polygon&);
		Polygon* kernel(Polygon&);
		Polygon* voronoiRegion(Point&, Point[], int, Polygon&);
		List<Polygon*>* voronoiDiagram(Point[], int, Polygon&);
		Polygon* mergeHull(Point[], int);
		Polygon* mHull(Point* [], int);
		Polygon* merge(Polygon*, Polygon*);
		void bridge(Polygon*, Polygon*, Vertex*&, Vertex*&, int);
		double closestPoints(Point[], int, Edge&);
		double cPoints(Point* [], Point* [], int, Edge&);
		void splitY(Point* [], int, Point*, Point* [], Point* []);
		double checkStrip(Point* [], int, Point*, double, Edge&);
		List<Polygon*>* triangulate(Polygon*);
		void findConvexVertex(Polygon*);
		Vertex* findIntrudingVertex(Polygon*);;
		bool pointInTriangle(Vertex*, Vertex*, Vertex*, Vertex*);
	}
	namespace surface {
		List<Triangle3D*>* depthSort(Triangle3D* [], int);
		void shuffleList(List<Triangle3D*>*, Triangle3D*);
		void refineList(List<Triangle3D*>*, Triangle3D*);

	}

	int lineTriangleintersect(Edge3D&, Triangle3D&, double&);
	Polygon* project(Triangle3D&, int, int);
	bool overlapExtent(Triangle3D*, Triangle3D*, int);
	int triangleCmp(Triangle3D*, Triangle3D*);
	bool mayObscure(Triangle3D*, Triangle3D*);
	bool projectionOverlap(Triangle3D*, Triangle3D*);
	int splitTriangleByPlane(Triangle3D*, Triangle3D*, Triangle3D*&, Triangle3D*&, Triangle3D*&);

	//classes
	class QuadTree;
	class TwoDTree;
	class BspTree;

	enum { VERTICAL = 0, HORIZONTAL = 1 };

	class Grid {
		int m;
		double cellSize;
		List<Point*>*** g;
		void _grid(double domainSize, Point s[], int n);
	public:
		Grid(double domainSize, Point s[], int n, int _m = 10);
		Grid(double domainSize, Point s[], int n, double M = 1.0);
		~Grid(void);
		List<Point*>* rangeQuery(Rectangle& range);
		friend class QuadTree;
	};

	class QuadTreeNode {
		QuadTreeNode* child[4];
		List<Point*>* pts;
		int size;
		List<Point*>* rangeQuery(Rectangle& range, Rectangle& span);
		bool isExternal();
		Rectangle quadrant(Rectangle&, int);
	public:
		QuadTreeNode(List<Point*>*);
		QuadTreeNode(void);
		~QuadTreeNode(void);
		friend class QuadTree;
	};

	class QuadTree {
		QuadTreeNode* root;
		Rectangle domain;
		QuadTreeNode* buildQuadTree(Grid& G, int M, int D, int level, int, int, int, int);
	public:
		QuadTree(Grid& G, int M, int D);
		~QuadTree();
		List<Point*>* rangeQuery(Rectangle& range);
	};

	class TwoDTreeNode {
		Point* pnt;
		TwoDTreeNode* lchild;
		TwoDTreeNode* rchild;
		List<Point*>* rangeQuery(Rectangle&, int);
	public:
		TwoDTreeNode(Point*);
		~TwoDTreeNode();
		friend class TwoDTree;
	};

	class TwoDTree {
		TwoDTreeNode* root;
		TwoDTreeNode* buildTwoDTree(Point* x[], Point* y[], int n, int cutType);
	public:
		TwoDTree(Point[], int);
		~TwoDTree();
		List<Point*>* rangeQuery(Rectangle&);
	};

	class BspTreeNode {
	private:
		BspTreeNode* posChild;
		BspTreeNode* negChild;
		Triangle3D* tri;
	public:
		BspTreeNode(Triangle3D*);
		~BspTreeNode();
		List<Triangle3D*>* visibilitySort(Point3D);
		friend class BspTree;
	};

	class BspTree {
		BspTreeNode* root;
		BspTreeNode* buildBspTree(List<Triangle3D*>*);
	public:
		BspTree(Triangle3D* [], int);
		~BspTree();
		List<Triangle3D*>* visibiltySort(Point3D);
	};
}