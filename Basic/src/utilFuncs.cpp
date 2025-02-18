#include "utilFuncs.h"

void reverseString(char* a[], int n)
{
	Stack<char*> s;
	for (int i = 0; i < n; i++) {
		s.push(a[i]);
	}
	for (int i = 0; i < n; i++) {
		a[i] = s.pop();
	}
}



int cg::leftToRightCmp(Point* a, Point* b) {
	if (*a < *b)return -1;
	if (*a > *b)return 1;
	return 0;
}

int cg::rightToLeftCmp(Point* a, Point* b) {
	return leftToRightCmp(b, a);
}

int cg::bottomToTopCmp(Point* a, Point* b) {
	if ((a->y < b->y) || ((a->y == b->y) && (a->x < b->x)))
		return -1;
	else if ((a->y > b->y) || ((a->y == b->y) && (a->x > b->x)))
		return 1;
	else
		return 0;
}

int cg::leftToRightCmp(Vertex* a, Vertex* b) {
	Point pa = a->point();
	Point pb = b->point();
	return leftToRightCmp(&pa, &pb);
};

int cg::rightToLeftCmp(Vertex* a, Vertex* b) {
	Point pa = a->point();
	Point pb = b->point();
	return rightToLeftCmp(&pa, &pb);
};

void cg::splitPointSet(Point* y[], int n, Point* x, Point* yL[], Point* yR[], int(*cmp)(Point*, Point*)) {
	int lidx = 0, ridx = 0;
	for (int i = 0; i < n; i++) {
		if ((*cmp)(y[i], x) < 0) {
			yL[lidx++] = y[i];
		}
		else if((*cmp)(y[i], x)>0) {
			yR[ridx++] = y[i];
		}
	}
}