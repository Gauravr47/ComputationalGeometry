#include "adts.h"

///////////////////////////////////////////////////////RTTSTARTREE//////////////////////////////////////////////////////////

RRTStarNode::RRTStarNode(cg::Point _p, RRTStarNode* _parent, double _cost) : pnt(_p), parent(_parent), cost(_cost) {
}

RRTStarNode::RRTStarNode(cg::Point _p, RRTStarNode* _parent) : pnt(_p), parent(_parent){
	cost = 0.0;
}

RRTStarNode::RRTStarNode(cg::Point _p) : pnt(_p), parent(nullptr) {
	cost = 0.0;
}

RRTStarNode::~RRTStarNode(void) {
}

RRTStarNode* RRTStarTree::getRandomNode() {
	double x = mapBoundingBox.org.x + static_cast<double>(rand()) / RAND_MAX * (mapBoundingBox.dest.x - mapBoundingBox.org.x);
	double y = mapBoundingBox.org.y + static_cast<double>(rand()) / RAND_MAX * (mapBoundingBox.dest.y - mapBoundingBox.org.y);
	return new RRTStarNode(Point(x,y));
}

RRTStarNode* RRTStarTree::getNearestNode(Point p) {
	RRTStarNode* nearest = nullptr;
	double minDist = std::numeric_limits<double>::max();
	nodes->first();
	while(!nodes->isHead()) {
		double dist = (nodes->val()->pnt - p).length();
		if (dist < minDist) {
			minDist = dist;
			nearest = nodes->val();
		}
		nodes->next();
	}
	return nearest;
}

RRTStarNode* RRTStarTree::steer(RRTStarNode* nearest, RRTStarNode* random) {
	double theta = atan2(random->pnt.y - nearest->pnt.y, random->pnt.x - nearest->pnt.x);
	double newX = nearest->pnt.x + stepSize * cos(theta);
	double newY = nearest->pnt.y + stepSize * sin(theta);
	if (newX < mapBoundingBox.org.x || newX > mapBoundingBox.dest.x || newY < mapBoundingBox.org.y || newY > mapBoundingBox.dest.y) return nullptr;
	return new RRTStarNode(Point(newX, newY), nearest, nearest->cost + stepSize);
}

List<RRTStarNode*>* RRTStarTree::findNearNodes(RRTStarNode* newNode) {
	List<RRTStarNode*>* neighbors = new List< RRTStarNode*>;
	nodes->first();
	while (!nodes->isHead()) {
		double dist = distance(nodes->val(),newNode);
		if (dist < searchRadius) {
			neighbors->insert(nodes->val());
		}
		nodes->next();
	}
	return neighbors;
}

RRTStarNode* RRTStarTree::chooseBestParent(List<RRTStarNode*>* neighbors, RRTStarNode* nearest, RRTStarNode* newNode) {
	RRTStarNode* best = nearest;
	double minCost = nearest->cost + distance(nearest, newNode);
	neighbors->first();
	while (neighbors->length()>0 && !neighbors->isHead()) {
		double cost = neighbors->val()->cost + distance(neighbors->val(), newNode);
		if (cost < minCost) {
			minCost = cost;
			best = neighbors->val();
		}
		neighbors->next();
	}
	return best;
}

void RRTStarTree::rewire(List<RRTStarNode*>* neighbors, RRTStarNode* newNode) {
	neighbors->first();
	while (!neighbors->isHead()) {
		double newCost = newNode->cost + distance(newNode, neighbors->val());
		if (newCost < neighbors->val()->cost) {
			neighbors->val()->parent = newNode;
			neighbors->val()->cost = newCost;
		}
		neighbors->next();
	}
}

double RRTStarTree::distance(RRTStarNode* a, RRTStarNode* b) {
	return (a->pnt - b->pnt).length();
}

RRTStarTree::RRTStarTree(cg::Point _start, cg::Point _goal, double _stepSize, double _searchRadius, cg::Edge boundingBox = Edge(Point(0,100), Point(100,100)), cg::List<cg::Polygon*>* obs = nullptr)
	: stepSize(_stepSize), searchRadius(_searchRadius), closestDist(std::numeric_limits<double>::max()), mapBoundingBox(boundingBox){
	start = new RRTStarNode(_start);
	goal = new RRTStarNode(_goal);
	closestToGoal = start;
	nodes = new cg::List<RRTStarNode*>();
	path = new cg::List<RRTStarNode*>();
	nodes->insert(start);
	iterations = 0;
	if (obs == nullptr) {
		obstacles = new List<Polygon*>();
	}
	else {
		obstacles = obs;
	}
}

RRTStarTree::~RRTStarTree() {
	obstacles = nullptr;
	delete start;
	delete goal;
	delete closestToGoal;
	delete nodes;
	delete path;
	
}

void RRTStarTree::run(int iterations) {
	for (int i = 0; i < iterations; ++i) {
		RRTStarNode* randomNode = getRandomNode();
		RRTStarNode* nearest = getNearestNode(randomNode->pnt);
		RRTStarNode* newNode = steer(nearest, randomNode);
		List<RRTStarNode*>* neighbors;
		RRTStarNode* bestParent;
		if (newNode && !isInObstacle(newNode)) {
			neighbors = findNearNodes(newNode);
			bestParent = chooseBestParent(neighbors, nearest, newNode);
			newNode->parent = bestParent;
			newNode->cost = bestParent->cost + distance(bestParent, newNode);
			nodes->insert(newNode);
			rewire(neighbors, newNode);

			double distToGoal = distance(newNode, goal);
			if (distToGoal < closestDist) {
				closestDist = distToGoal;
				closestToGoal = newNode;
			}

			if (goalReached(newNode)) {
				this->iterations = i;
				return;
			}
			
		}
	}
}

void RRTStarTree::updateStart(Point _start) {
	start = new RRTStarNode(_start);
}

void RRTStarTree::updateGoal(Point _goal) {
	goal = new RRTStarNode(_goal);
}

bool RRTStarTree::generatePath() {
	int numRuns = 4;
	run(1000);
	while (!goalReached(closestToGoal) && numRuns >0) {
		run(1000);
		numRuns--;
	}
	path->insert(closestToGoal);
	RRTStarNode* curr = closestToGoal;
	while (!nodes->isFirst() && curr != start) {
		path->prepend(curr->parent);
		curr = curr->parent;
	}
	return goalReached(path->last());
}

bool RRTStarTree::goalReached(RRTStarNode* node) {
	return distance(node, this->goal) < goalThreshold;
}

cg::List< RRTStarNode*>* RRTStarTree::getPath() {
	return path;
}

bool  RRTStarTree::isInObstacle(RRTStarNode* node) {
	Point curr = node->pnt;
	if (obstacles->length() < 1) {
		return false;
	}
	obstacles->first();
	while (!obstacles->isHead()) {
		if(cg::poly::pointInPolygon(curr, *obstacles->val()))
			return true;
		obstacles->next();
	}
	return false;
}
