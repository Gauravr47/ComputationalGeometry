
#include "utilFuncs.h"
#include "basic_data_structures.h"
#include "algorithms.h"
#include "solid.h"
#include <string>;
#include <iostream>
#include <memory>
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
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	try {
		string fileName = "C://Users//gaura//OneDrive//Documents//github//ComputationalGeometry//Basic//testFiles//cube.stl";
		Solid s(fileName.c_str());
		List<shared_ptr<Triangle3D>> * l = s.getTriangles();
		//List<Triangle3D*>* dep = cg::surface::depthSort(tri, s.getNumberTriangles());;
		Polygon p;
		p.insert(Point(4, 5));
		p.insert(Point(5, 4));
		p.insert(Point(5, 2));
		p.insert(Point(4, 2));
		Point t(3, 3);
		bool isIn = poly::pointInConvexPolygon(t, p);
	}
	catch (exception& e) {
		std::cerr<<e.what();
	}
	_CrtDumpMemoryLeaks();
}