

#include "utilFuncs.h"
#include "basic_data_structures.h"
#include "algorithms.h"
#include "solid.h"
#include "adts.h"
#include <string>
#include <iostream>


using namespace std;
using namespace cg;

int cmpInt(int val, int nVal) {
	if (nVal<val) {
		return -1;
	}
	else if (nVal > val) {
		return 1;
	}
	else {
		return 0;
	}
}

void printTree(int val) {
	cout << "//n" << val;
	return;
}

int cmpChar(char* val, char* nVal) {
	return strcmp(val, nVal);
}

int main() {

	try {
		//string fileName = "C://Users//gaura//OneDrive//Documents//github//ComputationalGeometry//Basic//testFiles//cube.stl";
	
		Polygon p;
		p.insert(Point(1.0, 1.0));
		p.insert(Point(-3.0, 5.0));
		p.insert(Point(1.0, 8.0));
		p.insert(Point(3.0, 5.0));
		List<shared_ptr<Polygon>>* obs = new List<shared_ptr<Polygon>>();
		obs->insert(make_shared<Polygon>(p));
		cg::Point start(0.0, 10.0);
		cg::Point goal(10.0, 0.0);
		Edge boundingBox(Point(-11, -11.0), Point(11.0, 11.0));
		RRTStarTree test(start, goal, 0.5, 2.0, boundingBox,obs);
		test.generatePath();
		List< shared_ptr<RRTStarNode>>* path = test.getPath();
		path->first();
		while (!path->isHead()) {
			cout << "\n" << path->val()->pnt.x << "  " << path->val()->pnt.y;
			delete path->val();
			path->next();
		}
		delete obs;
	}
	catch (exception& e) {
		std::cerr<<e.what();
	}
	_CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_DEBUG);
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetBreakAlloc(105);
	_CrtDumpMemoryLeaks();
}