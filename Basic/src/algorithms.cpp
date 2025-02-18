#include "algorithms.h"
using namespace cg;

//global variables
Point cg::poly::hullCmpPoint;
Point cg::poly::polarCmpPoint;
double cg::poly::slCurrx;
int cg::poly:: monoSweepDirection;
double cg::poly::monoCurrx;
int cg::poly::monoCurrType;

//functions
bool cg::poly::pointInConvexPolygon(Point& a, Polygon& p) {
	if (p.size() == 1) {
		Point temp = p.point();
    	return (a == temp);
	}
	if (p.size() == 2) {
		int c = a.classify(p.edge());
		return ((c == BETWEEN) || (c == ORIGIN) || (c == DESTINATION));
	}
	Vertex* org = p.v();
	for (int i = 0; i < p.size(); i++, p.advance(CLOCKWISE)) {
		if (a.classify(p.edge()) == LEFT) {
			p.setV(org);
			return false;
		}
	}
	return true;
}

Vertex* cg::poly::leastVertex(Polygon& p, int(*cmp)(Point*, Point*))
{
	Vertex* bestV = p.v();
	p.advance(CLOCKWISE);
	for (int i = 1; i < p.size(); p.advance(CLOCKWISE), i++) {
		int result = (*cmp)(p.v(), bestV);
		if (result < 0)
			bestV = p.v();
	}
	p.setV(bestV);
	return bestV;
}

int cg::lineTriangleintersect(Edge3D& e, Triangle3D& p, double& t)
{
	Point3D q;
	int aclass = e.intersect(p, t);
	if (aclass == COLLINEAR || aclass == PARALLEL)
		return aclass;
	q = e.point(t);
	int h, v;
	Point3D n = p.n();
	Point3D z(0.0, 0.0, 1.0);
	Point3D x(1.0, 0.0, 0.0);
	
	if (dotProduct(n, z) != 0.0) {
		h = 0;
		v = 1;
	}
	else if (dotProduct(n, x) != 0.0) {
		h = 1;
		v = 2;
	}
	else {
		h = 2;
		v = 0;
	}
	Polygon* pp = project(p, h, v);
	Point pq = Point(q[h], q[v]);
	int answer = poly::pointInConvexPolygon(pq, *pp);
	delete pp;
	return (answer ? SKEW_CROSS : SKEW_NO_CROSS);

}

Polygon* cg::project(Triangle3D& p, int h, int v)
{
	Point3D a;
	Point pts[3];
	for (int i = 0; i < 3; i++) {
		a = p[i];
		pts[i] = Point(a[h], a[v]);
	}
	Polygon* pp = new Polygon;
	for (int i = 0; i < 2; i++) {
		pp->insert(pts[i]);
	}
	if (pts[2].classify(pts[0], pts[1]) == LEFT)
		pp->advance(CLOCKWISE);
	pp->insert(pts[2]);
	return nullptr;
}

Polygon* cg::poly:: starPolygon(Point s[], int n) {
	Polygon* p = new Polygon;
	p->insert(s[0]);
	Vertex* origin = p->v();
	cg::poly::polarCmpPoint = origin->point();
	for (int i = 1; i < n; i++) {
		p->setV(origin);
		p->advance(CLOCKWISE);
		while (poly::polarCmp(&(s[i]), &(p->v()->point())) < 0) {
			p->advance(CLOCKWISE);
		}
		p->advance(COUNTER_CLOCKWISE);
		p->insert(s[i]);
	}
	p->setV(origin);
	return p;
}

int cg::poly::polarCmp(Point* p, Point* q) {
	Point po = *p - cg::poly::polarCmpPoint;
	Point qo = *q - cg::poly::polarCmpPoint;
	double pPolar = po.polarAngle();
	double qPolar = qo.polarAngle();
	if (pPolar < qPolar) 
		return -1;
	if (pPolar > qPolar) 
		return 1;
	if (po.length() < qo.length()) 
		return -1;
	if (po.length() > qo.length()) 
		return 1;
	return 0;
}

Polygon* cg::poly::insertionHull(Point s[], int n) {
	Polygon* p = new Polygon;
	p->insert(s[0]);
	hullCmpPoint = p->point();
	for (int i = 1; i < n; i++) {
		if (cg::poly::pointInConvexPolygon(s[i], *p))
			continue;
		cg::poly::leastVertex(*p, cg::poly::closestToPolygonCmp);
		cg::poly::supportingLine(s[i], p, LEFT);
		Vertex* l = p->v();
		supportingLine(s[i], p, RIGHT);
		delete p->split(l);
		p->insert(s[i]);
	}
	return p;
}

Polygon* cg::poly::insertionHull2(Point pts[], int n) {
	Polygon* p = new Polygon;
	Point **s = new Point*[n];
	for (int i = 0; i < n; i++) {
		s[i] = &pts[i];
	}
	selectionSort<Point*>(s, n, leftToRightCmp);
	p->insert(*s[0]);
	for (int i = 1; i < n; i++) {
		if (*s[i] == *s[i+1])
			continue;
		cg::poly::supportingLine(*s[i], p, LEFT);
		Vertex* l = p->v();
		supportingLine(*s[i], p, RIGHT);
		delete p->split(l);
		p->insert(*s[i]);
	}
	return p;
}

void cg::poly::supportingLine(Point& s, Polygon* p, int side) {
	int rotation = (side == LEFT) ? CLOCKWISE : COUNTER_CLOCKWISE;
	Vertex* a = p->v();
	Vertex* b = p->neighbor(rotation);
	int c = b->classify(s, *a);
	while ((c == side) || (c == BEYOND) || (c == BETWEEN)) {
		p->advance(rotation);
		a = p->v();
		b = p->neighbor(rotation);
		c = b->classify(s, *a);
	}
}

int cg::poly::closestToPolygonCmp(Point* a, Point* b) {
	double distA = (hullCmpPoint - *a).length();
	double distB = (hullCmpPoint - *b).length();
	if (distA < distB)return -1;
	else if (distA > distB) return 1;
	else return 0;
}

int cg::poly::pointInPolygon(Point& a, Polygon& p) {
	int parity = 0;
	for (int i = 0; i < p.size(); i++) {
		Edge e = p.edge();
		switch (cg::poly::edgeType(a, e)) {
		case TOUCHING:
			return BOUNDARY;
		case CROSSING:
			parity = 1 - parity;
		}
	}
	return (parity ? INSIDE : OUTSIDE);
}

int cg::poly::edgeType(Point& a, Edge& e) {
	Point v = e.org;
	Point w = e.dest;

	switch (a.classify(e)) {
		case LEFT:
			return ((v.y < a.y) && (a.y <= w.y)) ? TOUCHING : INESSENTIAL;
		case RIGHT:
			return ((w.y < a.y) && (a.y <= v.y)) ? TOUCHING : INESSENTIAL;
		case BETWEEN:
		case ORIGIN:
		case DESTINATION:
			return TOUCHING;
		default:
			return INESSENTIAL;
	}
}

double cg::poly::signedAngle(Point& a, Edge& e) {
	//create ab and ac segments
	Point v = e.org - a; 
	Point w = e.dest - a;
	double va = v.polarAngle();
	double wa = w.polarAngle();
	//Handle case when either org or destination is the same as point a
	if (va == -1.0 || wa == -1.0)
		return 180;
	//find the signed angle difference
	double x = wa - va;
	//if Horizontal, it means that the point lies on the edge (BOUNDARY), return 180 to be handled as Boundary
	if ((x == 180) || (x == -180))
		return 180.0;
	else if (x < -180) //angle between -180 to 180
		return x + 360;
	else if (x > 180)
		return x - 360;
	else
		return x;
}

int cg::poly::pointInPolygon2(Point& a, Polygon& p) {
	//if point lies outside, the total will be 0, if it lies inside the total will be -360
	double total = 0.0;
	for (int i = 0; i < p.size(); i++, p.advance(CLOCKWISE)) {
		double angle = cg::poly::signedAngle(a, p.edge());
		if (angle == 180.0) {
			return BOUNDARY;
		}
		total += angle;
	}
	return ((total < -180) ? INSIDE : OUTSIDE);
}

 //Cyrus-beck clipping algorithm, clip line segment to a polygon, result is line segment
bool cg::poly::clipLineSegment(Edge& s, Polygon& p, Edge& r) {
	double t0 = 0.0;
	double t1 = 1.0;
	double t;
	Point v = s.dest - s.org;
	for (int i = 0; i < p.size(); i++, p.advance(CLOCKWISE)) {
		Edge e = p.edge();
		if (s.intersect(e, t) == SKEW) {
			//Edges intersect, whcih means either t0 is increased or t1 is decresed. 
			Edge f = e;
			f.rot();
			Point n = f.dest - f.org;
			if (dotProduct(n, v) > 0.0) {
				if (t > t0) {
					t0 = t;
				}
			}
			else {
				if (t < t1) {
					t1 = t;
				}
			}
		}
		else {  //Edge s is parallel or collinear to edge e
				//Check if s lies to the left of e, which means outside the polygon,
				//function returns false else it ignores this edge and continues
			if (s.org.classify(e) == LEFT)
				return false;
		}
	}
	//return result
	if (t0 <= t1) {
		r = Edge(s.point(t0), s.point(t1));
		return true;
	}
	return false;
}
 //clip one polygon to another one, result is polygon/s
bool cg::poly::clipPolygon(Polygon& p, Polygon& clip, Polygon*&	result) {
	Polygon* q = new Polygon(p); // make a pointer to the copy of og polygon to be clipped
	Polygon* r;
	bool flag = true;
	for (int i = 0; i < clip.size(); i++, clip.advance(CLOCKWISE)) {
		Edge e = clip.edge();
		if (cg::poly::clipPolygonToEdge(*q, e, r)) { //if result is positive, delete old polygon and refer to the new result for next edge
			delete q;
			q = r;
		}
		else { // if no result, then stop the loop
			delete q;
			flag = false;
			break;
		}
	}
	if (flag) {
		result = q;
		return true;
	}
	return false;
}

List<Polygon*>* cg::poly::triangulateMonotonePolygon(Polygon& p)
{
	Stack<Vertex*> s; //stack to store vertices in montone fashion in increasing x
	Vertex* v, *vu, *vl;
	List<Polygon*> *triangles = new List<Polygon*>;
	leastVertex(p, leftToRightCmp); //find the leftmost vertex and set as first point
	v = vu = vl = p.v();
	s.push(v); //push first
	int chain = advancePtr(vu, vl, v); //advance to next vertex in x. Chain will represent if it was from upper chain or lower chain
	s.push(v); //push second
	while (true) {
		chain = advancePtr(vu, vl, v); //find next consecitive vertex
		if (adjacent(s.top(), v) && !adjacent(s.bottom(), v)) {
			Vertex* a = s.top();
			Vertex* b = s.nextToTop();
			int side = (chain == UPPER) ? LEFT : RIGHT;
			while (s.size() > 0 && b->classify(*v, *a) == side) {
				if (chain == UPPER) { //Split works in next fashion and polygon is arranged in clockwise fashion 
					//so take care of spliting based on Upper or Lower chain
					p.setV(b);
					triangles->append(p.split(v));
				}
				else {
					p.setV(v);
					triangles->append(p.split(b));
				}
				s.pop();
				a = b;
				b = s.nextToTop();
			}
			s.push(v);
		}
		else if (!adjacent(s.top(), v)) {
			Polygon* q;
			Vertex* t = s.pop();
			if (chain == UPPER) {
				p.setV(t); //set vertex to most recent
				q = p.split(v); 
			}
			else {
				p.setV(v);
				q = p.split(t);
				q->advance(CLOCKWISE); //set apex back to v 
			}
			triangulateFanPolygon(*q, triangles);
			while (!s.empty())
				s.pop(); //clear stack as these vertices are now part of polygon q and will be triangulated
			//push t and v to stack for next operations
			s.push(t);
			s.push(v);
		}
		else {
			p.setV(v);
			triangulateFanPolygon(p, triangles);
			break;
		}
	}
	return triangles;
}

int cg::poly::advancePtr(Vertex* vu, Vertex* vl, Vertex* v) {
	Vertex* vun = vu->cw(); //next v with higher x along upper chain
	Vertex* vln = vl->ccw(); //next v with higher x along lower chain
	if (vun->point() < vln->point()) { //if vun.x < vln.x, then next x is from upper chain
		v = vu = vun;
		return UPPER;
	}
	else {//if vun.x > vln.x, then next x is from lower chain
		v = vu = vun;
		v = vl = vln;
		return LOWER;
	}
}

bool cg::poly::adjacent(Vertex* v, Vertex* w) {
	return ((v->cw() == w) || (v->ccw() == w));
}

void cg::poly::triangulateFanPolygon(Polygon& p, List<Polygon*>* triangles) {
	Vertex* t = p.v()->cw()->cw();
	int size = p.size();
	for (int i = 3; i < size; i++) {
		triangles->append(p.split(t));
		t = t->cw();
	}
	triangles->append(&p);
	return;
}

 //clip polygon to an edge, result is polygon/s
bool cg::poly::clipPolygonToEdge(Polygon& s, Edge& e, Polygon* &result) {
	Polygon* p = new Polygon; //polygon to store result in temp
	Point crossingPoint; // point to store intersection point
	for (int i = 0; i < s.size(); i++, s.advance(CLOCKWISE)) {
		Point org = s.point();
		Point dest = s.cw()->point();
		bool orgIsInside = (org.classify(e) != LEFT); //If left, point lies outside
		bool destIsInside = (dest.classify(e) != LEFT); //If left, point lies outside
		if (orgIsInside != destIsInside) { //if org is inside and dest is outside, add vice versa, find intersection point
			double t;
			e.intersect(s.edge(), t);
			crossingPoint = e.point(t);
		}
		if (orgIsInside && destIsInside) { //if both inside, add dest
			p->insert(dest);
		}
		else if (orgIsInside && !destIsInside) { //if org is inside and destination outside, add crossing point
			if (org != crossingPoint)
				p->insert(crossingPoint);
		}
		else if (!orgIsInside && !destIsInside) { //if both outside, ignore
			;
		}
		else {									//if org is outside and destination inside, add crossing point and dest
			p->insert(crossingPoint);
			if (dest != crossingPoint)
				p->insert(dest);
		}
	}
	result = p;
	return(result->size() > 0);
}

//incremental selection methods
Polygon* cg::poly::giftWrapHull(Point a[], int n) {
	int l = 0;
	//find the leftmost point
	for (int i = 1; i < n; i++) {
		if (a[i]<a[l])
			l = i;
	}
	//the leftmost point serves as sentinal point. Here the actaul array has n-1 elements. During using this
	//algorithm make sure to have a extra element space in the input array
	a[n] = a[l];
	Polygon* p = new Polygon;
	for (int i = 0; i < n; i++) {
		//All points of convex hull are stored from [0,i] and rest of the points [l+1, n] may or may not belomg to hull
		swap(a[i], a[l]);
		p->insert(a[i]);
		l = i + 1;
		for (int j = i + 2; j <=n; j++) {
			int aclass = a[j].classify(a[i], a[l]);
			if ( aclass == LEFT || aclass == BEYOND)
				l = j;
		}
		//if the hull reaches the last a.k.a first element, then we can terminate the algorithm
		if (l == n)
			return p;
	}
	return nullptr;
}

Polygon* cg::poly::grahamScanHull(Point a[], int n) {
	int m = 0;
	int i;
	//find extreme point
	for (i = 1; i < n; i++) {
		if((a[i].y<a[m].y)||((a[i].y==a[m].y)&&(a[i].x>a[m].x)))
			m = i;
	}
	swap(a[0], a[m]);
	polarCmpPoint = a[0];
	//create a pointer copy array
	Point** pts = new Point * [n];
	for (i = 0; i < n; i++) {
		pts[i] = &a[i];
	}
	/*sort based on polarCmp function.Presorting simplifies the hull finding phase : Each point
	processed during the hull finding phase gets inserted into the current hull, no questions asked;
	moreover, the vertices to be removed from the current hull are easy to find.*/
	selectionSort(&pts[1], n-1, polarCmp);
	for (int j = 0; j < n; j++) {
		Point test = *pts[j];
		test = test;
	}
	//find the next point
	for (i = 1; pts[i + 1]->classify(*pts[0], *pts[i]) == BEYOND; i++)
		;

	//create a stack to store points. This algorithm finds the hull in counter clockwise manner. When emptying the stack,
	//resulting polygon is then obtained in clockwise manner
	Stack<Point*> s;
	s.push(pts[0]);
	s.push(pts[i]);

	//Add consecutive points. Remove points until the point does not lie to the left of the edge formed by s top, s top-1
	for (i = i + 1; i < n; i++) {
		while (pts[i]->classify(*s.nextToTop(), *s.top()) != LEFT)
			s.pop();
		s.push(pts[i]);
	}
	Polygon* p = new Polygon;
	while (!s.empty()) {
		p->insert(*s.pop());
	}
	delete pts;
	return p;
}

List<Triangle3D*>* cg::surface::depthSort(Triangle3D* tri[], int n)
{
	List<Triangle3D*> *result = new List<Triangle3D*>;
	//create a copy of triangle array
	Triangle3D** t = new Triangle3D*[n]; 
	for (int i = 0; i < n; i++) {
		t[i] = new Triangle3D(*tri[i]);
	}
	//sort based in descending z direction from far to close
	cg::insertionSort(t, n, triangleCmp);
	List<Triangle3D*> *triSort = arrayToList(t, n);
	delete t; //??
	while (triSort->length() > 0) {
		Triangle3D* p = triSort->first();
		Triangle3D* q = triSort->next();
		bool hasShuffled = false;
		for (; !triSort->isHead() && !overlapExtent(p, q, 2); q = triSort->next()) {
			//check if there is overlap. If so then do we need to refine the list by splitting triangles of simply shuffle the list
			if (cg::mayObscure(p, q)) {
				if (hasShuffled || q->mark) {
					refineList(triSort, p);
				}
				else {
					shuffleList(triSort, p);
					hasShuffled = true;
					break;
				}

			}
			if (!hasShuffled) {
				result->insert(p);
				triSort->first();
				triSort->remove();
			}
		}
	}
	delete triSort;
	return result;
	}

	int cg::triangleCmp(Triangle3D * p, Triangle3D * q) {
		Edge3D pBox = p->boundingBox();
		Edge3D	qBox = q->boundingBox();
		if (pBox.dest.z > qBox.dest.z)
			return -1;
		else if (qBox.dest.z > pBox.dest.z)
			return 1;
		else
			return 0;
	}

	bool cg::overlapExtent(Triangle3D* p, Triangle3D* q, int i) {
		Edge3D pBox = p->boundingBox();
		Edge3D	qBox = q->boundingBox();
		if(((pBox.org[i] <= qBox.org[i]) && (qBox.org[i] <= pBox.dest[i])) ||
			((qBox.org[i] <= pBox.org[i]) && (pBox.org[i] <= qBox.dest[i])))
			return true;
		else
			return false;
	}

	void cg::surface::shuffleList(List<Triangle3D*>* t, Triangle3D* p) {
		Triangle3D* q = t->val();
		q->mark = true;
		t->val(p);
		t->first();
		t->val(q);
		return;
	}

	bool cg::mayObscure(Triangle3D* p, Triangle3D* q) {
		//Does x entent of triangles not overlap ?
		int i;
		if (!overlapExtent(p, q, 0))
			return false;
		//Does y extent of triangles not overlap ?
		if (!overlapExtent(p, q, 1))
			return false;
		//Is p entirely behind or on the plane of q ? This is with the assumption that the triangles here 
		// are oriented with thier normals pointing towards the center. If the convetion is that the normals
		//are oriented away from the center like 3D printing STL files, then this check the points should classify as positive
		for (int i = 0; i < 3; i++) {
			if ((*p)[i].classify(*q) == NEGATIVE)
				break;
		}
		if (i == 3)
			return false;
		// Is q entirely front or on the plane of p ? This is with the assumption that the triangles here 
		// are oriented with thier normals pointing towards the center. If the convetion is that the normals
		//are oriented away from the center like 3D printing STL files, then this check the points should classify as negative
		for (int i = 0; i < 3; i++) {
			if ((*q)[i].classify(*p) == POSITIVE)
				break;
		}
		if (i == 3)
			return false;
		//Do projections of p & q not overlap ?
		if (!projectionOverlap(p, q))
			return false;
		return true;
	}

	bool cg::projectionOverlap(Triangle3D* p, Triangle3D* q) {
		bool answer = true;
		Polygon *pProj = project(*p, 0, 1);
		Polygon* qProj = project(*q, 0, 1);
		//Does either vertex of projected p lies within q ?
		for (int i = 0; i < 3; i++, pProj->advance(CLOCKWISE)) {
			if(cg::poly::pointInConvexPolygon(pProj->point(), (*qProj)))
				goto FINISH;
		}
		//Does either vertex of projected q lies within p ?
		for (int i = 0; i < 3; i++, qProj->advance(CLOCKWISE)) {
			if (cg::poly::pointInConvexPolygon(qProj->point(), (*pProj)))
				goto FINISH;
		}
		//Does any edge of p intersects any edge of q ?
		for (int i = 0; i < 3; i++, pProj->advance(CLOCKWISE)) {
			Edge pEdge = pProj->edge();
			for (int j = 0; j < 3; j++, qProj->advance(CLOCKWISE)) {
				Edge qEdge = qProj->edge();
				double t;
				if (pEdge.cross(qEdge, t) == SKEW_CROSS)
					goto FINISH;
			}
		}
		answer = false;
	FINISH:
		delete pProj;
		delete qProj;
		return true;
	}

	void cg::surface::refineList(List<Triangle3D*>* t, Triangle3D* p) {
		Triangle3D *q1, *q2, *q3;
		int nbrTris = splitTriangleByPlane(t->val(),p, q1,q2,q3);
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

	int cg::splitTriangleByPlane(Triangle3D* q, Triangle3D* p, Triangle3D* &q1, Triangle3D* &q2, Triangle3D* &q3) {
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
			q1 = new Triangle3D(d,b,c, (*q).id);
			q2 = new Triangle3D(a,d,c, (*q).id);
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

	bool cg::poly::aimsAt(Edge& a, Edge& b, int aclass, int crossingType) {
		Point va = a.dest - a.org;
		Point vb = b.dest - b.org;
		if (crossingType != COLLINEAR) {
			if ((va.x * vb.y - vb.x * va.y) >= 0)
			{
				return (aclass != RIGHT);
			}
			else {
				return (aclass != LEFT);
			}
		}
		else {
			/*If edges a and b are collinear, a aims at b if endpoint a. dest does not lie beyond b. This
			is used to ensure that a is advanced, rather than b, when the two edges intersect degenerately 
			in more than one point. By allowing a to "catch up" with b, we ensure that no intersection
			points are skipped over.*/
			return (aclass != BEYOND);
		}
		
	}

	Polygon* cg::poly::convexPolygonIntersect(Polygon& p, Polygon& q) {
		Polygon* R;
		int phase = 1;
		Point intPt, startPt;
		int iFlag = UNKNOWN;
		
		int maxIts = 2 * (p.size() + q.size());
		for (int i = 0; (i < maxIts) || (phase == 2); i++)
		{
			Edge pEdge = p.edge();
			Edge qEdge = q.edge();
			int pClass = pEdge.dest.classify(qEdge);
			int qClass = qEdge.dest.classify(pEdge);
			int crossingType = crossingPoint(pEdge, qEdge, intPt);
			if (crossingType == SKEW_CROSS) {
				if (phase == 1) {
					R = new Polygon();
					R->insert(intPt);
					phase = 2;
				}
				else if (R->point() != intPt) {
					if (intPt != startPt) {
						R->insert(intPt);
					}
					else {
						return R;
					}

				}
				if (pClass == RIGHT)iFlag = P_IS_INSIDE;
				else if (qClass == RIGHT) iFlag = Q_IS_INSIDE;
				else iFlag = UNKNOWN;
			}
			else if (crossingType == COLLINEAR && pClass != BEYOND && qClass!=BEYOND) {
				iFlag = UNKNOWN;
			}
			int pAimsQ = aimsAt(pEdge,qEdge, pClass, crossingType);
			int qAimsP = aimsAt(qEdge, pEdge, qClass, crossingType);
			if (pAimsQ && qAimsP) {
				if (iFlag == Q_IS_INSIDE ||(iFlag==UNKNOWN && pClass==LEFT) ) {
					advance(p, *R, false);
				}
				else {
					advance(q, *R, false);
				}
			}
			else if (pAimsQ && !qAimsP) {
				advance(p, *R, iFlag == P_IS_INSIDE);
			}
			else if (!pAimsQ && qAimsP) {
				advance(q, *R, iFlag == Q_IS_INSIDE);
			}
			else {
				if (iFlag == Q_IS_INSIDE || (iFlag == UNKNOWN && pClass == LEFT)) {
					advance(p, *R, false);
				}
				else {
					advance(q, *R, false);
				}
			}
		}//for
		if (pointInConvexPolygon(p.point(), q))return new Polygon(p);
		else if (pointInConvexPolygon(q.point(), p))return new Polygon(q);
		else return new Polygon();
	}

	void cg::poly::advance(Polygon& A, Polygon& R, int inside) {
		A.advance(CLOCKWISE);
		if (inside && R.point() != A.point()) {
			R.insert(A.point());
		}
		return;
	}

	int cg::poly::crossingPoint(Edge& a, Edge& b, Point& r ) {
		double s, t;
		int crossType = a.intersect(b, s);
		if ((crossType == PARALLEL) || (crossType == COLLINEAR))
			return crossType;
		double lena = (a.dest - a.org).length();
		if (s < -lena*EPSILON2 || s >1.0+lena * EPSILON2)
			return SKEW_NO_CROSS;
		b.intersect(a, t);
		double lenb = (b.dest - b.org).length();
		if ((t >= -lenb * EPSILON2) && (t <= 1.0 + lenb * EPSILON2)) {
			if (t <= EPSILON2 * lenb) r = b.org;
			else if (t >= 1.0 - EPSILON2 * lenb) r = b.dest;
			else if (s <= EPSILON2 * lena) r = a.org;
			else if (s >= 1.0 - EPSILON2 * lena) r = b.dest;
			else  r = b.point(t);
			return SKEW_CROSS;
		}
		else
			return SKEW_NO_CROSS;
	}

	List<Polygon*>* cg::poly::delaunayTriangulate(Point s[], int n) {
		List<Polygon*>* triangles = new List<Polygon*>;
		Dictionary<Edge*> frontier(edgeCmp);
		Edge* e= hullEdge(s, n);
		frontier.insert(e);
		while (!frontier.isEmpty()) {
			e = frontier.removeMin();
			Point p;
			if (findMate(*e, s, n, p)) {
				updateFrontier(frontier, p, e->org);
				updateFrontier(frontier, e->dest, p);
				triangles->insert(triangle(e->org, e->dest, p));
			}
			delete e;
		}
		return triangles;
	}

	int cg::poly::edgeCmp(Edge* a, Edge* b) {
		if (a->org < b->org) return -1;
		else if (a->org > b->org) return 1;
		else if (a->dest < b->dest)	return -1;
		else if (a->dest > b->dest)return 1;
		else return 0;
	}

	Edge* cg::poly::hullEdge(Point s[], int n) {
		int m = 0;
		int i;
		for (i = 1; i < n; i++) {
			if (s[i] < s[m])
			{
				m = i;
			}
		}
		swap(s[m], s[0]);
		for (m = 1, i = 2; i < n; i++) {
			int c = s[i].classify(s[0], s[m]);
			if ((c == LEFT) || (c == BETWEEN)) {
				m = i;
			}
		}
		return new Edge(s[0], s[m]);
	}

	bool cg::poly::findMate(Edge e, Point s[], int n, Point& p) {
		double t, tBest = FLT_MAX;
		Point* pBest = nullptr;
		Edge f = e;
		f.rot(); 
		for (int i = 0; i < n; i++) {
			int c = s[i].classify(e);
			if (c == RIGHT) {
				Edge g(e.org, s[i]);
				g.rot();
				f.intersect(g, t);
				if (t < tBest) {
					tBest = t;
					pBest = &s[i];
				}
			}
		}
		if (pBest != nullptr) {
			p = *pBest;
			return true;
		}
		return false;
	}

	Polygon* cg::poly::triangle(Point& a, Point& b, Point& c) {
		Polygon* p = new Polygon;
		p->insert(a);
		p->insert(b);
		p->insert(c);
		return p;
	}

	void cg::poly::updateFrontier(Dictionary<Edge*>& frontier, Point& org, Point& dest) {
		Edge* e = new Edge(org, dest);
		if (frontier.find(e)) {
			frontier.remove(e);
		}
		else {
			e->flip();
			frontier.insert(e);
		}
	}

	List<poly::EventPoint*>* poly::intersectSegments(Edge edges[], int n) {
		Dictionary<poly::EventPoint*> schedule = buildSchedule(edges, n);
		Dictionary<Edge*> sweepline(edgeCmp2);
		List<poly::EventPoint*>* result = new List<poly::EventPoint*>;
		while (!schedule.isEmpty()) {
			EventPoint* ev = schedule.removeMin();
			slCurrx = ev->p.x;
			ev->handleTransition(sweepline, schedule, result);
		}
		return result;
	}
	
	
	Dictionary<poly::EventPoint*>& poly::buildSchedule(Edge edges[], int n) {
		Dictionary<poly::EventPoint*> *schedule = new Dictionary<poly::EventPoint*>(eventCmp);
		for (int i = 0; i < n; i++) {
			schedule->insert(new LeftEndpoint(&edges[i]));
			schedule->insert(new RightEndpoint(&edges[i]));
		}
		return *schedule;
	}

	int poly::edgeCmp2(Edge* a, Edge* b) {
		double ya = a->y(poly::slCurrx- EPSILON3);
		double yb = b->y(poly::slCurrx - EPSILON3);
		if (ya < yb)return -1;
		else if (ya > yb)return 1;
		double ma = a->slope();
		double mb = b->slope();
		if (ma > mb) return -1;
		else if (ma < mb)return 1;
		return 0;
	}

	int poly::eventCmp(poly::EventPoint* a, poly::EventPoint* b) {
		if (a->p < b->p) return -1;
		else if (a->p > b->p)return 1;
		return 0;
	}

	void poly::LeftEndpoint::handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<poly::EventPoint*>& schedule, List<poly::EventPoint*>* result) {
		Edge* b = sweepLine.find(&e);
		Edge* c = sweepLine.next();
		sweepLine.prev();
		Edge* a = sweepLine.prev();
		double t;
		if (a && c && (a->cross(*c, t) == SKEW_CROSS)) {
			Point pt = a->point(t);
			if (pt.x > poly::slCurrx) {
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

	void poly::RightEndpoint::handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<poly::EventPoint*>& schedule, List<poly::EventPoint*>* result) {
		Edge* b = sweepLine.find(&e);
		Edge* c = sweepLine.next();
		sweepLine.prev();
		Edge* a = sweepLine.prev();
		double t;
		if (a && c && (a->cross(*c, t) == SKEW_CROSS)) {
			Point pt = a->point(t);
			if (pt.x > poly::slCurrx) {
				schedule.insert(new CrossingPoint(a,c,pt));
			}
		}
	}

	void poly::CrossingPoint::handleTransition(Dictionary<Edge*>& sweepLine, Dictionary<poly::EventPoint*>& schedule, List<poly::EventPoint*>* result) {
		Edge* b = sweepLine.find(&e1);
		Edge* a = sweepLine.prev();
		Edge* c = sweepLine.find(&e2);
		Edge* d = sweepLine.next();
		double t;
		if (a && c && (a->cross(*c, t) == SKEW_CROSS)) {
			Point pt = a->point(t);
			if (pt.x > poly::slCurrx) {
				schedule.insert(new CrossingPoint(a, c, pt));
			}
		}

		if (b && d && (b->cross(*d, t) == SKEW_CROSS)) {
			Point pt = b->point(t);
			if (pt.x > poly::slCurrx) {
				schedule.insert(new CrossingPoint(b, d, pt));
			}
		}

		if (a && b && (a->cross(*b, t) == SKEW_CROSS)) {
			Point pt = a->point(t);
			if (pt.x > poly::slCurrx) {
				CrossingPoint cev(a, b, pt);
				schedule.remove(&cev);
			}
		}

		if (d && c && (d->cross(*c, t) == SKEW_CROSS)) {
			Point pt = d->point(t);
			if (pt.x > poly::slCurrx) {
				CrossingPoint cev(c, d, pt);
				schedule.remove(&cev);
			}
		}

		sweepLine.remove(b);
		poly::slCurrx += EPSILON3;
		sweepLine.insert(b);
		poly::slCurrx -= EPSILON3;
		result->append(this);
	}

	poly::AxesParallelEdge::AxesParallelEdge(Rectangle* _r, int _type):r(_r),type(_type) {
		m = -DBL_MAX;
		count = 0;
	}

	double poly::AxesParallelEdge::pos(void) {
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

	double poly::AxesParallelEdge::min(void) {
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

	double poly::AxesParallelEdge::max(void) {
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

	void poly::AxesParallelEdge::setMin(double f) {
		m = f;
	}

	void poly::AxesParallelEdge::handleLeftEdge(Dictionary<poly::AxesParallelEdge*>& sweepline, List<Edge*>* segs) {
		//insert top and bottom edges of the new rectangle
		sweepline.insert(new AxesParallelEdge(r, TOP_SIDE));
		AxesParallelEdge* u = sweepline.val(); //save reference to top edge
		sweepline.insert(new AxesParallelEdge(r, BOTTOM_SIDE));
		AxesParallelEdge * l = sweepline.val(); //reference to bottom edge
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
			}else if ((l->type == TOP_SIDE) && (l->count++ == 0)) {//process top edge
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

	void poly::AxesParallelEdge::handleRightEdge(Dictionary<poly::AxesParallelEdge*>& sweepline, List<Edge*>* segs) {
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
	
	List<Edge*>* poly::findContour(Rectangle recs[], int n) {
		//initialise data structures
		poly::AxesParallelEdge** schedule = buildSchedule(recs, n);
		Dictionary<AxesParallelEdge*> sweepline(AxesParallelEdgeCmp);
		List<Edge*>* segs = new List<Edge*>;
		Rectangle sentinal(Point(-DBL_MAX, -DBL_MAX), Point(DBL_MAX, DBL_MAX), -1);
		sweepline.insert(new AxesParallelEdge(&sentinal, BOTTOM_SIDE));
		for (int i = 0; i < 2 * n; i++) {
			AxesParallelEdge* curr = schedule[i];
			switch (curr->type) {
			case LEFT_SIDE:
				curr->handleLeftEdge(sweepline, segs); break;
			case RIGHT_SIDE:
				curr->handleRightEdge(sweepline, segs); break;
			}
		}
		return segs;
	}

	poly::AxesParallelEdge** poly::buildSchedule(Rectangle recs[], int n) {
		poly::AxesParallelEdge** schedule = new poly::AxesParallelEdge* [2*n];
		for (int i = 0; i < n; i++) {
			schedule[2 * i] = new AxesParallelEdge(&recs[i], BOTTOM_SIDE);
			schedule[2 * i +1] = new AxesParallelEdge(&recs[i], TOP_SIDE);
		}
		insertionSort(schedule, 2 * n, poly::AxesParallelEdgeCmp);
		return schedule;
	}

	int poly::AxesParallelEdgeCmp(poly::AxesParallelEdge* a, poly::AxesParallelEdge* b) {
		if (a->pos() < b->pos()) return -1;
		else if (a->pos() > b->pos()) return 1;
		else if (a->type < b->type) return -1;
		else if (a->type > b->type)return 1;
		else if (a->r->id < b->r->id)return -1;
		else if (a->r->id < b->r->id)return 1;
		return 0;
	}

	double poly::ActiveEdge::y() {
		return v->y;
	}

	Edge poly::ActiveEdge::edge() { 
		return Edge(v->point(), v->neighbor(rotation)->point()); 
	}

	double poly::ActiveEdge::slope() { 
		return edge().slope();
	}

	double poly::ActivePoint::y() {
		return p.y;
	}

	List<Polygon*>* poly::regularize(Polygon& p) {
		List<Polygon*>* poly1 = new List<Polygon*>();
		semiregulaize(p, LEFT_TO_RIGHT, poly1);
		List<Polygon*>* poly2 = new List<Polygon*>();
		poly1->last();
		while (!poly1->isHead()) {
			semiregulaize(*poly1->remove(), RIGHT_TO_LEFT, poly2);
		}
		return poly2;
	}

	 void poly::semiregulaize(Polygon& p, int direction, List<Polygon*>* poly) {
		 int (*cmp)(Vertex*, Vertex*);
		 monoSweepDirection = direction;
		 if (monoSweepDirection == LEFT_TO_RIGHT) {
			 cmp = leftToRightCmp;
		 }
		 else {
			 cmp = rightToLeftCmp;
		 }
		 Vertex** schedule = buildSchedule(p, cmp);
		 Dictionary<ActiveElement*> sweepLine(activeElementCmp);
		 sweepLine.insert(new ActivePoint(Point(0.0, -DBL_MAX)));
		 for (int i = 0; i < p.size(); i++) {
			 Vertex* currV = schedule[i];
			 monoCurrx = currV->x;
			 monoCurrType = typeEvent(currV, cmp);
			 switch (monoCurrType) 
			 {
			 case START_TYPE:
				 break;
			 case END_TYPE:
				 break;
			 case BEND_TYPE:
			 default:
				 break;
			 }
		 }
		 return;
	}

	Vertex** poly::buildSchedule(Polygon& p, int(*cmp)(Vertex*, Vertex*)) {
		Vertex** schedule = new Vertex * [p.size()];
		for (int i = 0; i < p.size(); i++, p.advance(CLOCKWISE)) {
			schedule[i] = p.v();
		}
		insertionSort(schedule, p.size(), cmp);
		return schedule;
	}

	int poly::typeEvent(Vertex* v, int(*cmp)(Vertex*, Vertex*)) {
		int a = cmp(v->cw(), v);
		int b = cmp(v->ccw(), v);
		if ((a > 0) && (b > 0)) return START_TYPE;
		else if ((a <= 0) && (b <= 0))return END_TYPE;
		else return BEND_TYPE;
	}

	int poly::activeElementCmp(poly::ActiveElement* a, poly::ActiveElement* b) {
		double ya = a->y();
		double yb = b->y();
		if (ya < yb)return -1;
		else if (ya > yb)return 1;
		if (a->type == ACTIVE_POINT && b->type == ACTIVE_EDGE)
			return -1;
		else if (a->type == ACTIVE_EDGE && b->type == ACTIVE_POINT)
			return 1;
		double aSlope = a->slope();
		double bSlope = b->slope();
		int rval = 1;
		if ((monoCurrType == START_TYPE && monoSweepDirection == LEFT_TO_RIGHT) || (monoCurrType == END_TYPE && monoSweepDirection == RIGHT_TO_LEFT)) {
			rval = -1;
		}
		if (aSlope < bSlope) return rval;
		else if (aSlope > bSlope)return -rval;
		return 0;
	}

	void poly::monoStartTransition(Vertex* v, Dictionary<poly::ActiveElement*>& sweepline) {
		ActivePoint ve(v->point());
		ActiveEdge* a = (ActiveEdge*)sweepline.locate(&ve);
		Vertex* w = a->w;
		if (!isConvex(v)) {
			Vertex* wp = v->split(w);
			sweepline.insert(new ActiveEdge(wp, CLOCKWISE, wp->cw()));
			sweepline.insert(new ActiveEdge(v->ccw(), COUNTER_CLOCKWISE, v));
			a->w = (monoSweepDirection == LEFT_TO_RIGHT) ? wp->ccw() : v;
		}
		else {
			sweepline.insert(new ActiveEdge(v->ccw(), COUNTER_CLOCKWISE, v));
			sweepline.insert(new ActiveEdge(v, CLOCKWISE, v->cw()));
			a->w = v;
		}
	}

	void poly::monoBendTransition(Vertex* v, Dictionary<poly::ActiveElement*>& sweepline) {
		ActivePoint ve(v->point());
		ActiveEdge* a = (ActiveEdge*)sweepline.locate(&ve);
		ActiveEdge* b = (ActiveEdge*)sweepline.next();
		a->w = v;
		b->w = v;
		b->v = b->v->neighbor(b->rotation);

	}

	void poly::monoEndTransition(Vertex* v, Dictionary<poly::ActiveElement*>& sweepline, List<Polygon*>* polys) {
		ActivePoint ve(v->point());
		ActiveEdge* a = (ActiveEdge*)sweepline.locate(&ve);
		ActiveEdge* b = (ActiveEdge*)sweepline.next();
		ActiveEdge* c = (ActiveEdge*)sweepline.next();
		if (!isConvex) {
			polys->append(new Polygon(v));
		}
		else {
			((ActiveEdge*)a)->w = v;
		}
		sweepline.remove(b);
		sweepline.remove(c);
	};

	bool poly::isConvex(Vertex* v) {
		Vertex* u = v->ccw();
		Vertex* w = v->cw();
		int c = w->classify(u->point(), v->point());
		if ((c == BEYOND) || (c == RIGHT))
			return true;
		return false;
	}

	Polygon* poly::halfPlaneIntersect(Edge H[], int n, Polygon& box) {
		Polygon* c;
		if (n == 1) {
			poly::clipPolygonToEdge(box, H[0], c);
		}
		else {
			int m = n / 2;
			Polygon* a = halfPlaneIntersect(H, m, box);
			Polygon* b = halfPlaneIntersect(H + m, n-m,box);
			c = convexPolygonIntersect(*a, *b);
			delete a;
			delete b;
		}
		return c;
	}

	Polygon* poly::kernel(Polygon& p) {
		Edge *edges = new Edge[p.size()];
		for(int i = 0; i<p.size(); i++, p.advance(CLOCKWISE)){
			edges[i] = p.edge();
		}
		Polygon Box;
		Box.insert(Point(-DBL_MAX, -DBL_MAX));
		Box.insert(Point(-DBL_MAX, DBL_MAX));
		Box.insert(Point(DBL_MAX, DBL_MAX));
		Box.insert(Point(DBL_MAX, -DBL_MAX));
		Polygon* r = poly::halfPlaneIntersect(edges, p.size(), Box);
		delete[] edges;
		return r;
	}

	Polygon* poly::voronoiRegion(Point &p, Point s[], int n, Polygon& box) {
		Edge* edges = new Edge[n];
		for (int i = 0; i < n; i++) {
			edges[i] = Edge(p, s[i]);
			edges[i].rot();
		}
		Polygon* r = poly::halfPlaneIntersect(edges, n, box);
		delete edges;
		return r;
	}

	List<Polygon*>* poly::voronoiDiagram(Point s[], int n, Polygon &box) {
		List<Polygon*>* r = new List<Polygon*>;
		for (int i = 0; i < n; i++) {
			Point p = s[i];
			s[i] = s[n - 1];
			r->append(voronoiRegion(p, s, n, box));
			s[i] = p;
		}
		return r;
	}

	Polygon* poly::mergeHull(Point pts[], int n) {
		Point** p = new Point * [n];
		for (int i = 0; i < n; i++) {
			p[i] = &pts[i];
		}
		mergeSort(p, n, leftToRightCmp);
		return mHull(p, n);
	}

	Polygon* poly::mHull(Point* pts[], int n) {
		if (n == 1) {
			Polygon* q = new Polygon;
			q->insert(*pts[0]);
			return q;
		}
		else {
			int m = n / 2;
			Polygon* L = mHull(pts, m);
			Polygon* R = mHull(pts+m, n - m);
			return merge(L, R);
		}
	}

	Polygon* poly::merge(Polygon* L, Polygon* R) {
		Vertex *l1, *l2, *r1, *r2;
		Vertex *vl = poly::leastVertex(*L, rightToLeftCmp);
		Vertex *vr = poly::leastVertex(*R, leftToRightCmp);
		bridge(L, R, l1, r1, UPPER);
		L->setV(vl);
		R->setV(vr);
		bridge(L, R, l2, r2, LOWER);
		L->setV(l1);
		L->split(r1);
		R->setV(r2);
		delete R->split(l2);
		return R;
	}

	void poly::bridge(Polygon *L, Polygon *R, Vertex *&vl, Vertex *&vr, int type) {
		int sides[2] = { LEFT, RIGHT };
		int index = (type == UPPER) ? 0 : 1;
		do {
			vl = L->v();
			vr = R->v();
			supportingLine(L->point(), R, sides[index]);
			supportingLine(R->point(), L, sides[1-index]);
		} while ((vl != L->v()) || (vr != R->v()));

	}

	double poly::closestPoints(Point pts[], int n, Edge &c) {
		Point** X = new Point* [n];
		Point** Y = new Point* [n];
		for (int i = 0; i < n; i++) {
			X[i] = Y[i] = &pts[i];
		}
		mergeSort(X, n, leftToRightCmp);
		mergeSort(Y, n, bottomToTopCmp);
		return cPoints(X, Y, n, c);
	}

	double poly::cPoints(Point* X[], Point* Y[], int n, Edge &c) {
		if (n == 1) {
			return DBL_MAX;
		}
		else {
			double delta;
			int m = n / 2;
			Point** YL = new Point * [m];
			Point** YR = new Point * [n-m];
			poly::splitY(Y, n, X[m], YL, YR);
			Edge a, b;
			double deltaL = cPoints(X, YL, m, a);
			double deltaR = cPoints(X + m, YR, n - m, b);
			delete[] YL;
			delete[] YR;
			if (deltaL < deltaR) {
				delta = deltaL;
				c = a;
			}
			else {
				delta = deltaR;
				c = b;
			}
			return checkStrip(Y, n, X[m], delta, c);
		}
	}

	void poly::splitY(Point* Y[], int n, Point* p, Point* YL[], Point* YR[]) {
		int i, indexL, indexR;
		i = indexL = indexR = 0;
		while (i < n) {
			if (*Y[i] < *p) {
				YL[indexL++] = Y[i++];
			}
			else {
				YR[indexR++] = Y[i++];
			}
		}
	}

	double poly::checkStrip(Point* Y[], int n, Point *p, double delta, Edge &c) {
		int i, striplen;
		Point* strip = new Point[n];
		for (i = striplen = 0; i < n; i++) {
			if ((p->x - delta < Y[i]->x) && (p->x + delta > Y[i]->x)) {
				strip[striplen++] = *Y[i];
			}
		}
		for (i = 0; i < striplen; i++) {
			for (int j = i + 1; j < striplen; j++) {
				if (strip[j].y - strip[i].y) break;
				if ((strip[j] - strip[i]).length() < delta) {
					delta = (strip[j] - strip[i]).length();
					c = Edge(strip[i], strip[j]);
				}
			}
		}
		delete[] strip;
		return delta;
	}

	List<Polygon*>* poly::triangulate(Polygon* p) {
		List<Polygon*>* triangles = new List<Polygon*>;
		if (p->size() ==3) {
			triangles->append(p);
		}
		else {
			findConvexVertex(p);
			Vertex* d = findIntrudingVertex(p);
			if (d == nullptr) {
				Vertex* c = p->neighbor(CLOCKWISE);
				p->advance(COUNTER_CLOCKWISE);
				Polygon* q = p->split(c);
				triangles->append(triangulate(p));
				triangles->append(q);
			}
			else {
				Polygon* q = p->split(d);
				triangles->append(triangulate(p));
				triangles->append(triangulate(q));
			}
		}
		return triangles;
	}

	void poly::findConvexVertex(Polygon* p) {
		Vertex* a = p->neighbor(COUNTER_CLOCKWISE);
		Vertex* b = p->v();
		Vertex* c = p->neighbor(CLOCKWISE);
		while (c->classify(*a, *b) != RIGHT) {
			a = b;
			b = p->advance(CLOCKWISE); 
			c = p->neighbor(CLOCKWISE);
		}
	}

	Vertex* poly::findIntrudingVertex(Polygon* p) {
		Vertex* a = p->neighbor(COUNTER_CLOCKWISE);
		Vertex* b = p->v();
		Vertex* c = p->neighbor(CLOCKWISE);
		Vertex* d = nullptr;
		double bestD = -1.0;
		Vertex* v = p->advance(CLOCKWISE);
		Edge ca = Edge(*c, *a);
		while (v != a) {
			if (poly::pointInTriangle(v, a, b, c)) {
				double dist = v->distance(ca);
				if (dist > bestD) {
					d = v;
					bestD = dist;
				}
			}
			v = p->advance(CLOCKWISE);
		}
		p->setV(b);
		return d;
	}

	bool poly::pointInTriangle(Vertex* v, Vertex* a, Vertex* b, Vertex* c) {
		return((v->classify(*a,*b) == LEFT) && (v->classify(*b, *c) == LEFT) && (v->classify(*c, *a) == LEFT));
	}

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
			else cg::surface::refineList(s, q);
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