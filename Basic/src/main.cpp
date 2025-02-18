
#include "utilFuncs.h"
#include "tree.h"
#include "algorithms.h"
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
	cout << "\n" << val;
	return;
}

int cmpChar(char* val, char* nVal) {
	return strcmp(val, nVal);
}

int main() {
	
	Point s[6] = { {3,3},{8,7},{7,3},{5,9},{2,7},{5,5}};	
	int size = 6;
	Edge ans;
	double closest = poly::closestPoints(s, size, ans);
	return 0;

}