#ifndef ADTS_H
#define ADTS_H

#ifdef _DEBUG
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type
#else
#define DBG_NEW new
#endif

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

#include "basic_data_structures.h"
#include "geometry.h"
#include "utilFuncs.h"
#include "algorithms.h"
#include <memory>
using namespace cg;
using namespace std;
	
class RRTStarTree;
	
class RRTStarNode {
public:
	Point pnt;
	shared_ptr<RRTStarNode> parent;
	double cost;
	RRTStarNode(Point, shared_ptr<RRTStarNode>, double);
	RRTStarNode(Point, shared_ptr<RRTStarNode>);
	RRTStarNode(Point);
	~RRTStarNode(void);
	friend class RRTStarTree;
};

class RRTStarTree {
	shared_ptr<RRTStarNode> start, goal;
	double stepSize, searchRadius;
	const double goalThreshold = 0.5;
	long iterations;
	shared_ptr<RRTStarNode> closestToGoal;
	double closestDist;
	cg::Edge mapBoundingBox;
	cg::List< shared_ptr<RRTStarNode>>* nodes;
	cg::List< shared_ptr<RRTStarNode>>* path;
	cg::List<shared_ptr<Polygon>>* obstacles;

	shared_ptr<RRTStarNode> getRandomNode();
	shared_ptr<RRTStarNode> getNearestNode(Point);
	shared_ptr<RRTStarNode> RRTStarTree::steer(shared_ptr<RRTStarNode>, shared_ptr<RRTStarNode>);
	List<shared_ptr<RRTStarNode>>* findNearNodes(shared_ptr<RRTStarNode>);
	shared_ptr<RRTStarNode> chooseBestParent(List<shared_ptr<RRTStarNode>>*, shared_ptr<RRTStarNode>, shared_ptr<RRTStarNode>);
	void rewire(List<shared_ptr<RRTStarNode>>*, shared_ptr<RRTStarNode>);
	double distance(shared_ptr<RRTStarNode>, shared_ptr<RRTStarNode>);
	bool goalReached(shared_ptr<RRTStarNode> node);
	bool isInObstacle(shared_ptr<RRTStarNode> node);
public:
	RRTStarTree(Point, Point, double, double, Edge, List<shared_ptr<Polygon>>*);
	~RRTStarTree();
	void updateStart(Point);
	void updateGoal(Point);
	void run(int iterations);
	bool generatePath();
	cg::List< shared_ptr<RRTStarNode>>* getPath();
};


#endif ADTS_H