
#include "utilFuncs.h"
#include "basic_data_structures.h"
#include "algorithms.h"
#include "solid.h"
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
	
		Polygon p, * r;
		p.insert(Point(1.0, 1.0));
		p.insert(Point(-3.0, 5.0));
		p.insert(Point(1.0, 8.0));
		p.insert(Point(3.0, 5.0));
		cg::Point start(3.0, 5.0);
		bool isIn = cg::poly::pointInPolygon(start, p);
		r = cg::poly::kernel(p);
		for (int i = 0; i < r->size(); i++, r->advance(CLOCKWISE)) {
			Point pt = r->point();
			cout << "\n" << pt.x << "\t" << pt.y;
		}
		delete r;
		_CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_DEBUG);
		_CrtDumpMemoryLeaks();
	}
	catch (exception& e) {
		std::cerr<<e.what();
	}
}