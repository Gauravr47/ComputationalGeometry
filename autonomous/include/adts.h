#ifndef ADTS_H
#define ADTS_H

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
	RRTStarNode* parent;
	double cost;
	RRTStarNode(Point, RRTStarNode*, double);
	RRTStarNode(Point, RRTStarNode*);
	RRTStarNode(Point);
	~RRTStarNode(void);
	friend class RRTStarTree;
};

class RRTStarTree {
	RRTStarNode *start, *goal;
	double stepSize, searchRadius;
	const double goalThreshold = 0.5;
	long iterations;
	RRTStarNode* closestToGoal;
	double closestDist;
	cg::Edge mapBoundingBox;
	cg::List< RRTStarNode*>* nodes;
	cg::List< RRTStarNode*>* path;
	cg::List< Polygon*>* obstacles;

	RRTStarNode* getRandomNode();
	RRTStarNode* getNearestNode(Point);
	RRTStarNode* RRTStarTree::steer(RRTStarNode*, RRTStarNode*);
	List<RRTStarNode*>* findNearNodes(RRTStarNode*);
	RRTStarNode* chooseBestParent(List<RRTStarNode*>*, RRTStarNode*, RRTStarNode*);
	void rewire(List<RRTStarNode*>*, RRTStarNode*);
	double distance(RRTStarNode*, RRTStarNode*);
	bool goalReached(RRTStarNode* node);
	bool isInObstacle(RRTStarNode* node);
public:
	RRTStarTree(Point, Point, double, double, Edge, List<Polygon*>*);
	~RRTStarTree();
	void updateStart(Point);
	void updateGoal(Point);
	void run(int iterations);
	bool generatePath();
	cg::List< RRTStarNode*>* getPath();
};


#endif ADTS_H