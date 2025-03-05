#ifndef GADTS_H
#define GADTS_H

#include "basic_data_structures.h"
#include "geometry.h"
#include "utilFuncs.h"
using namespace cg;

namespace cg {
	enum { ACTIVE_POINT, ACTIVE_EDGE };
	enum { LEFT_SIDE, RIGHT_SIDE, BOTTOM_SIDE, TOP_SIDE };
	enum { LEFT_TO_RIGHT, RIGHT_TO_LEFT };
	enum { VERTICAL = 0, HORIZONTAL = 1 };

	class QuadTree;
	class TwoDTree;
	class BspTree;
	
	extern double slCurrx;

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
		void refineList(List<Triangle3D*>*, Triangle3D*);
		int splitTriangleByPlane(Triangle3D*, Triangle3D*, Triangle3D*&, Triangle3D*&, Triangle3D*&);
	public:
		BspTree(Triangle3D* [], int);
		~BspTree();
		List<Triangle3D*>* visibiltySort(Point3D);
	};
}
#endif GADTS_H