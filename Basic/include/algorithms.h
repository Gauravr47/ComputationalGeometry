#ifndef ALGORITHMS_H
#define ALGORITHMS_H


#include "geometry.h"
#include "basic_data_structures.h"
#include "gadts.h"
#include "utilFuncs.h"
#define CG_MAX_D 1e10
using namespace cg;

namespace cg {

	namespace poly {
		enum { UNKNOWN, P_IS_INSIDE, Q_IS_INSIDE };
		enum { START_TYPE, BEND_TYPE, END_TYPE };
		extern Point hullCmpPoint;
		extern Point polarCmpPoint;
		extern int monoSweepDirection;
		extern double monoCurrx;
		extern int monoCurrType;

		
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
		double closestPoints(List<Point*>, Edge&);
		double cPoints(Point* [], Point* [], int, Edge&);
		void splitY(Point* [], int, Point*, Point* [], Point* []);
		double checkStrip(Point* [], int, Point*, double, Edge&);
		List<Polygon*>* triangulate(Polygon*);
		void findConvexVertex(Polygon*);
		Vertex* findIntrudingVertex(Polygon*);;
		bool pointInTriangle(Vertex*, Vertex*, Vertex*, Vertex*);
	}
	namespace surface {
		List<Triangle3D*>* depthSort(Triangle3D* [], int, bool=true);
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

}

#endif // ALGORITHMS_H