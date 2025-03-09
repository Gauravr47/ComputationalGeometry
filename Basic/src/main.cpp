
#include "utilFuncs.h"
#include "basic_data_structures.h"
#include "algorithms.h"
#include "solid.h"
#include <string>;
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
		string fileName = "C://Users//gaura//OneDrive//Documents//github//ComputationalGeometry//Basic//testFiles//Stanford_Bunny.stl";
		Solid s(fileName.c_str());
		Triangle3D** tri = cg::ListToArray(s.getTriangles());
		List<Triangle3D*>* dep = cg::surface::depthSort(tri, s.getNumberTriangles());
	}
	catch (exception& e) {
		std::cerr<<e.what();
	}
}