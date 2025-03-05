#include "gadts.h"

double cg::slCurrx;

/////////////////////////////////////////////////////// EVENTPOINT//////////////////////////////////////////////////////
void LeftEndpoint::handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<EventPoint*>& schedule, List<EventPoint*>* result) {
	Edge* b = sweepLine.find(&e);
	Edge* c = sweepLine.next();
	sweepLine.prev();
	Edge* a = sweepLine.prev();
	double t;
	if (a && c && (a->cross(*c, t) == SKEW_CROSS)) {
		Point pt = a->point(t);
		if (pt.x > slCurrx) {
			CrossingPoint cev(a, c, pt);
			schedule.remove(&cev);
		}
	}
	if (a && (b->cross(*a, t) == SKEW_CROSS)) {
		schedule.insert(new CrossingPoint(a, b, b->point(t)));
	}
	if (c && (b->cross(*c, t) == SKEW_CROSS)) {
		schedule.insert(new CrossingPoint(b, c, b->point(t)));
	}
}

void RightEndpoint::handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<EventPoint*>& schedule, List<EventPoint*>* result) {
	Edge* b = sweepLine.find(&e);
	Edge* c = sweepLine.next();
	sweepLine.prev();
	Edge* a = sweepLine.prev();
	double t;
	if (a && c && (a->cross(*c, t) == SKEW_CROSS)) {
		Point pt = a->point(t);
		if (pt.x > slCurrx) {
			schedule.insert(new CrossingPoint(a, c, pt));
		}
	}
}

void CrossingPoint::handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<EventPoint*>& schedule, List<EventPoint*>* result) {
	Edge* b = sweepLine.find(&e1);
	Edge* a = sweepLine.prev();
	Edge* c = sweepLine.find(&e2);
	Edge* d = sweepLine.next();
	double t;
	if (a && c && (a->cross(*c, t) == SKEW_CROSS)) {
		Point pt = a->point(t);
		if (pt.x > slCurrx) {
			schedule.insert(new CrossingPoint(a, c, pt));
		}
	}

	if (b && d && (b->cross(*d, t) == SKEW_CROSS)) {
		Point pt = b->point(t);
		if (pt.x > slCurrx) {
			schedule.insert(new CrossingPoint(b, d, pt));
		}
	}

	if (a && b && (a->cross(*b, t) == SKEW_CROSS)) {
		Point pt = a->point(t);
		if (pt.x > slCurrx) {
			CrossingPoint cev(a, b, pt);
			schedule.remove(&cev);
		}
	}

	if (d && c && (d->cross(*c, t) == SKEW_CROSS)) {
		Point pt = d->point(t);
		if (pt.x > slCurrx) {
			CrossingPoint cev(c, d, pt);
			schedule.remove(&cev);
		}
	}

	sweepLine.remove(b);
	slCurrx += EPSILON3;
	sweepLine.insert(b);
	slCurrx -= EPSILON3;
	result->append(this);
}

///////////////////////////////////////////////////AXISPARALLELEDGE//////////////////////////////////////////////////////
AxesParallelEdge::AxesParallelEdge(Rectangle* _r, int _type) :r(_r), type(_type) {
	m = -DBL_MAX;
	count = 0;
}

double AxesParallelEdge::pos(void) {
	switch (type)
	{
	case LEFT_SIDE:
		return r->sw.x; break;
	case RIGHT_SIDE:
		return r->ne.x; break;
	case TOP_SIDE:
		return r->ne.y; break;
	case BOTTOM_SIDE:
	default:
		return r->sw.y;
		break;
	}
}

double AxesParallelEdge::min(void) {
	if (m > -DBL_MAX)
		return m;
	switch (type)
	{
	case LEFT_SIDE:
	case RIGHT_SIDE:
		return r->sw.y; break;
	case TOP_SIDE:
	case BOTTOM_SIDE:
	default:
		return r->sw.x;
		break;
	}
}

double AxesParallelEdge::max(void) {
	switch (type)
	{
	case LEFT_SIDE:
	case RIGHT_SIDE:
		return r->ne.y; break;
	case TOP_SIDE:
	case BOTTOM_SIDE:
	default:
		return r->ne.x;
		break;
	}
}

void AxesParallelEdge::setMin(double f) {
	m = f;
}

void AxesParallelEdge::handleLeftEdge(Dictionary<AxesParallelEdge*>& sweepline, List<Edge*>* segs) {
	//insert top and bottom edges of the new rectangle
	sweepline.insert(new AxesParallelEdge(r, TOP_SIDE));
	AxesParallelEdge* u = sweepline.val(); //save reference to top edge
	sweepline.insert(new AxesParallelEdge(r, BOTTOM_SIDE));
	AxesParallelEdge* l = sweepline.val(); //reference to bottom edge
	AxesParallelEdge* p = sweepline.prev(); //reference to edge below bottom edge
	l->count = p->count + 1;
	p = sweepline.next(); //p now points to bottom edge
	l = sweepline.next(); //l now points to edge above bottom edge
	double curx = pos();
	for (; l != u; p = l, l = sweepline.next()) { //loop from edge above bottom to top. Add 1 to each edges count
		//process bottom edge
		if ((l->type == BOTTOM_SIDE) && (l->count++ == 1)) {
			//add the vertical line joining previous edge and current edge. Here pos will return y based on vertical edge type
			segs->append(new Edge(Point(curx, p->pos()), Point(curx, l->pos())));
			//add the horizontal line joining previous min x cordinate of l and the currx
			segs->append(new Edge(Point(l->min(), l->pos()), Point(curx, l->pos())));
		}
		else if ((l->type == TOP_SIDE) && (l->count++ == 0)) {//process top edge
			//add the horizontal line joining previous min x cordinate of l and the currx
			segs->append(new Edge(Point(l->min(), l->pos()), Point(curx, l->pos())));
		}
	}
	//process top edge as l now points to it
	if ((l->count = p->count - 1) == 0) {
		//add the vertical line joining previous edge and current edge. Here pos will return y based on vertical edge type
		segs->append(new Edge(Point(curx, p->pos()), Point(curx, l->pos())));
	}

}

void AxesParallelEdge::handleRightEdge(Dictionary<AxesParallelEdge*>& sweepline, List<Edge*>* segs) {
	//get reference to top and bottom edges
	AxesParallelEdge uEdge(r, TOP_SIDE);
	AxesParallelEdge* u = sweepline.find(&uEdge);
	AxesParallelEdge lEdge(r, BOTTOM_SIDE);
	AxesParallelEdge* l = sweepline.find(&lEdge);
	double curx = pos();
	//process bottom edge
	if (l->count == 1) {
		segs->append(new Edge(Point(l->min(), l->pos()), Point(curx, l->pos())));
	}

	//process top edge
	if (u->count == 0) {
		segs->append(new Edge(Point(u->min(), u->pos()), Point(curx, u->pos())));
	}
	AxesParallelEdge* initl = l; //reference to l
	AxesParallelEdge* p = l;
	l = sweepline.next();
	for (; l != u; p = l, l = sweepline.next()) { //loop from edge above bottom to top. remove 1 to each edges count
		//process bottom edge
		if ((l->type == BOTTOM_SIDE) && (--l->count == 1)) {
			//Add vertical line from previous edge to current edge
			segs->append(new Edge(Point(curx, p->pos()), Point(curx, l->pos())));
			//snap the edge to a new min
			l->setMin(curx);
		}
		else if ((l->type == TOP_SIDE) && (--l->count == 0)) {//process top edge
			//set the new min which can be used by next bottom or top edge of the current rectangle
			l->setMin(curx);
		}
	}
	//remove edges from sweep lines and add vertical if needed
	if (l->count == 0) {
		segs->append(new Edge(Point(curx, p->pos()), Point(curx, l->pos())));
	}
	sweepline.remove(u);
	sweepline.remove(initl);
}

///////////////////////////////////////////////////////ACTIVEEDGE//////////////////////////////////////////////////////

double ActiveEdge::y() {
	return v->y;
}

Edge ActiveEdge::edge() {
	return Edge(v->point(), v->neighbor(rotation)->point());
}

double ActiveEdge::slope() {
	return edge().slope();
}

double ActivePoint::y() {
	return p.y;
}
///////////////////////////////////////////////////////ACTIVEPOINT//////////////////////////////////////////////////////


/////////////////////////////////////////////////////// GRID //////////////////////////////////////////////////////////
void Grid::_grid(double domainSize, Point s[], int n) {
	cellSize = domainSize / m;
	g = new List<Point*>**[m];
	for (int i = 0; i < m; i++) {
		g[i] = new List<Point*>*[m];
		for (int j = 0; j < m; j++)
		{
			g[i][j] = new List<Point*>;
		}
	}
	for (int i = 0; i < n; i++) {
		int a = int(s[i].x / cellSize);
		int b = int(s[i].x / cellSize);
		g[a][b]->append(new Point(s[i]));
	}
}

Grid::Grid(double domainSize, Point s[], int n, int _m) : m(_m) {
	_grid(domainSize, s, n);
}

Grid::Grid(double domainSize, Point s[], int n, double M) : m(int(ceil(sqrt(n / M)))) {
	_grid(domainSize, s, n);
}

Grid::~Grid() {
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < m; j++) {
			g[i][j]->last();
			while (g[i][j]->length() > 0) {
				delete g[i][j]->remove();
			}
			delete g[i][j];
		}
		delete g[i];
	}
	delete g;
}

List<Point*>* Grid::rangeQuery(Rectangle& r) {
	List<Point*>* result = new List<Point*>;
	int iLimit = int(r.ne.x / cellSize);
	int jLimit = int(r.ne.y / cellSize);
	for (int i = int(r.sw.x / cellSize); i < iLimit; i++) {
		for (int j = int(r.sw.y / cellSize); j < jLimit; j++) {
			List<Point*>* pts = g[i][j];
			for (pts->first(); !pts->isHead(); pts->next()) {
				Point* p = pts->val();
				if (pointInRectangle(*p, r)) {
					result->append(p);
				}
			}
		}
	}
	return result;
}

/////////////////////////////////////////////////////// QUADTREE/////////////////////////////////////////////////////////
List<Point*>* QuadTreeNode::rangeQuery(Rectangle& range, Rectangle& span) {
	List<Point*>* result = new List<Point*>();
	if (!intersect(range, span)) {
		return result;
	}
	else if (isExternal()) {
		for (pts->first(); !pts->isHead(); pts->next()) {
			Point* p = pts->val();
			if (pointInRectangle(*p, range)) {
				result->append(p);
			}
		}
	}
	else {
		for (int i = 0; i < 4; i++) {
			List<Point*>* l = child[i]->rangeQuery(range, quadrant(span, i));
			result->append(l);
		}
	}
	return result;
}

bool QuadTreeNode::isExternal() {
	return (pts != nullptr);
}

Rectangle QuadTreeNode::quadrant(Rectangle& quad, int i) {
	Point c = 0.5 * (quad.sw + quad.ne);
	switch (i)
	{
	case 0:
		return Rectangle(c, quad.ne);
		break;
	case 1:
		return Rectangle(Point(c.x, quad.sw.y), Point(quad.ne.x, c.y));
		break;
	case 2:
		return Rectangle(quad.sw, c);
		break;
	case 3:
		return Rectangle(Point(quad.sw.x, c.y), Point(c.x, quad.ne.y));
		break;
	}
}

QuadTreeNode::QuadTreeNode(List<Point*>* _pts) : pts(_pts), size(pts->length()) {
	for (int i = 0; i < 4; i++) {
		child[i] = nullptr;
	}
}

QuadTreeNode::QuadTreeNode(void) : pts(nullptr), size(0) {
	for (int i = 0; i < 4; i++) {
		child[i] = nullptr;
	}
}

QuadTreeNode::~QuadTreeNode(void) {
	if (isExternal()) {
		pts->last();
		while (pts->length() > 0) {
			delete pts->remove();
		}
		delete pts;
	}
	else {
		for (int i = 0; i < 4; i++) {
			delete child[i];
		}
	}
}

QuadTreeNode* QuadTree::buildQuadTree(Grid& G, int M, int D, int level, int imin, int imax, int jmin, int jmax) {
	if (imin == imax) {
		QuadTreeNode* q = new QuadTreeNode(G.g[imin][imax]);
		G.g[imin][imax] = new List<Point*>();
		return q;
	}
	else {
		QuadTreeNode* q = new QuadTreeNode;
		int imid = 0.5 * (imin + imax);
		int jmid = 0.5 * (jmin + jmax);
		q->child[0] = buildQuadTree(G, M, D, level + 1, imid + 1, imax, jmid + 1, jmax);
		q->child[1] = buildQuadTree(G, M, D, level + 1, imin + 1, imax, jmin, jmid);
		q->child[2] = buildQuadTree(G, M, D, level + 1, imin, imid, jmin, jmid);
		q->child[3] = buildQuadTree(G, M, D, level + 1, imin, imid, jmid + 1, jmax);
		for (int i = 0; i < 4; i++)
			q->size += q->child[i]->size;
		if ((q->size <= M) || (level >= D)) {
			q->pts = new List<Point*>;
			for (int i = 0; i < 4; i++) {
				q->pts->append(q->child[i]->pts);
				delete q->child[i]->pts;
				q->child[i] = nullptr;
			}
		}
		return q;
	}
}

QuadTree::QuadTree(Grid& G, int M, int D) : domain(Rectangle(Point(0.0, 0.0), Point(G.m* G.cellSize, G.m* G.cellSize))) {
	root = buildQuadTree(G, M, D, 0, 0, G.m - 1, 0, G.m - 1);
}

QuadTree::~QuadTree() {
	delete root;
}

List<Point*>* QuadTree::rangeQuery(Rectangle& range) {
	return root->rangeQuery(range, domain);
}

///////////////////////////////////////////////////////TWODTREE//////////////////////////////////////////////////////////
List<Point*>* TwoDTreeNode::rangeQuery(Rectangle& range, int cutType) {
	List<Point*>* result = new List<Point*>;
	if (pointInRectangle(*pnt, range))
		result->append(pnt);
	int (*cmp)(Point*, Point*);
	if (cutType == VERTICAL) {
		cmp = leftToRightCmp;
	}
	else {
		cmp = bottomToTopCmp;
	}
	if (lchild != nullptr && ((*cmp)(&range.sw, pnt) < 0)) {
		result->append(lchild->rangeQuery(range, 1 - cutType));
	}
	if (rchild != nullptr && ((*cmp)(&range.ne, pnt) > 0)) {
		result->append(rchild->rangeQuery(range, 1 - cutType));
	}
	return result;
}

TwoDTreeNode::TwoDTreeNode(Point* p) :pnt(p), lchild(nullptr), rchild(nullptr) {

}

TwoDTreeNode::~TwoDTreeNode() {
	if (lchild) delete lchild;
	if (rchild) delete rchild;
	delete pnt;
}

TwoDTreeNode* TwoDTree::buildTwoDTree(Point* x[], Point* y[], int n, int cutType) {
	if (n == 0) {
		return nullptr;
	}
	if (n == 1) {
		return new TwoDTreeNode(x[0]);
	}
	int m = n / 2;
	TwoDTreeNode* p = new TwoDTreeNode(x[m]);
	Point** yL = new Point * [m];
	Point** yR = new Point * [n - m];
	int(*cmp)(Point*, Point*);
	if (cutType == VERTICAL) cmp = cg::leftToRightCmp;
	else cmp = cg::bottomToTopCmp;
	splitPointSet(y, n, x[m], yL, yR, cmp);
	p->lchild = buildTwoDTree(yL, x, m, 1 - cutType);
	p->rchild = buildTwoDTree(yR, x + m + 1, n - m, 1 - cutType);
	delete[] yL;
	delete[] yR;
	return p;
}

TwoDTree::TwoDTree(Point p[], int n) {
	Point** x = new Point * [n];
	Point** y = new Point * [n];
	for (int i = 0; i < n; i++) {
		x[i] = y[i] = &p[i];
	}
	mergeSort(x, n, leftToRightCmp);
	mergeSort(y, n, bottomToTopCmp);
	this->root = buildTwoDTree(x, y, n, VERTICAL);
}

TwoDTree::~TwoDTree() {
	delete root;
}

List<Point*>* TwoDTree::rangeQuery(Rectangle& range) {
	return root->rangeQuery(range, VERTICAL);
}

///////////////////////////////////////////////////////BSPTREE//////////////////////////////////////////////////////////
BspTreeNode::BspTreeNode(Triangle3D* _tri) : tri(_tri), posChild(nullptr), negChild(nullptr) {

}

BspTreeNode::~BspTreeNode() {
	if (posChild != nullptr) delete posChild;
	if (negChild != nullptr) delete negChild;
	delete tri;
}

List<Triangle3D*>* BspTreeNode::visibilitySort(Point3D p) {
	List < Triangle3D*>* s = new List<Triangle3D*>;
	if (p.classify(*tri) == POSITIVE) {
		if (negChild) s->append(negChild->visibilitySort(p));
		s->append(tri);
		if (posChild) s->append(posChild->visibilitySort(p));
	}
	else {
		if (posChild) s->append(posChild->visibilitySort(p));
		s->append(tri);
		if (negChild) s->append(negChild->visibilitySort(p));
	}
	return s;
}

void cg::BspTree::refineList(List<Triangle3D*>* t, Triangle3D* p) {
	Triangle3D* q1, * q2, * q3;
	int nbrTris = splitTriangleByPlane(t->val(), p, q1, q2, q3);
	if (nbrTris > 1) {
		t->remove();
		t->insert(q1);
		t->insert(q2);
	}
	if (nbrTris == 3) {
		t->insert(q3);
	}
	return;
}

int cg::BspTree::splitTriangleByPlane(Triangle3D* q, Triangle3D* p, Triangle3D*& q1, Triangle3D*& q2, Triangle3D*& q3) {
	Point3D splitPoints[2];
	int cl[3], edgeIds[2];
	int numTri = 0;
	for (int i = 0; i < 3; i++) {
		cl[i] = (*q)[i].classify((*p));
	}
	for (int i = 0; i < 3; i++) {
		if ((cl[i] == NEGATIVE && cl[(i + 1) / 3] == POSITIVE) || (cl[i] == POSITIVE && cl[(i + 1) / 3] == NEGATIVE))
		{
			Edge3D qEdge((*q)[i], (*q)[(i + 1) / 3]);
			double t;
			qEdge.intersect(*p, t);
			edgeIds[numTri] = i;
			splitPoints[numTri++] = qEdge.point(t);
		}
	}
	if (numTri == 0)
		return 1;
	Point3D a = (*q)[edgeIds[0]];
	Point3D b = (*q)[(edgeIds[0] + 1) % 3];
	Point3D c = (*q)[(edgeIds[0] + 2) % 3];

	if (numTri == 1) {
		Point3D d = splitPoints[0];
		q1 = new Triangle3D(d, b, c, (*q).id);
		q2 = new Triangle3D(a, d, c, (*q).id);
	}
	else {
		Point3D d = splitPoints[0];
		Point3D e = splitPoints[1];
		if (edgeIds[1] == (edgeIds[0] + 1) % 3)
		{
			q1 = new Triangle3D(d, b, e, (*q).id);
			q2 = new Triangle3D(a, d, e, (*q).id);
			q2 = new Triangle3D(a, e, c, (*q).id);
		}
		else {
			q1 = new Triangle3D(a, d, e, (*q).id);
			q2 = new Triangle3D(b, e, d, (*q).id);
			q2 = new Triangle3D(c, e, b, (*q).id);
		}
	}
	return (numTri + 1);
}

BspTreeNode* BspTree::buildBspTree(List<Triangle3D*>* s) {
	if (s->length() == 0)
		return nullptr;
	if (s->length() == 1)
		return new BspTreeNode(s->first());
	List<Triangle3D*>* sp = new List<Triangle3D*>;
	List<Triangle3D*>* sn = new List<Triangle3D*>;
	Triangle3D* p = s->first();
	for (s->next(); !s->isHead(); s->next()) {
		Triangle3D* q = s->val();
		int cl[3];
		for (int i = 0; i < 3; i++) {
			cl[i] = (*q)[i].classify(*p);
		}
		if (cl[0] != NEGATIVE && cl[1] != NEGATIVE && cl[2] != NEGATIVE) sp->append(q);
		else if (cl[0] != POSITIVE && cl[1] != POSITIVE && cl[2] != POSITIVE) sn->append(p);
		else cg::BspTree::refineList(s, q);
	}
	BspTreeNode* n = new BspTreeNode(s->first());
	n->posChild = buildBspTree(sp);
	n->negChild = buildBspTree(sn);
	delete[] sp;
	delete[] sn;
	return n;
}

BspTree::BspTree(Triangle3D* t[], int n) {
	List<Triangle3D*>* tris = new List<Triangle3D*>;
	for (int i = 0; i < n; i++) {
		tris->append(new Triangle3D(*t[i]));
	}
	root = buildBspTree(tris);
}

BspTree::~BspTree() {
	delete root;
}

List<Triangle3D*>* BspTree::visibiltySort(Point3D p) {
	return root->visibilitySort(p);
}