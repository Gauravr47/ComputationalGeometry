#include "points.h"



int checkOrientation(point2D p, point2D q, point2D r)
{
	int value = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (value == 0) return 0;
	return (value > 0 ? 1 : 2);
};

bool doIntersect(point2D p1, point2D q1, point2D p2, point2D q2)
{
	int p1q1p2 = checkOrientation(p1, q1, p2);
	int p1q1q2 = checkOrientation(p1, q1, q2);
	int p2q2p1 = checkOrientation(p2, q2, p1);
	int p2q2q1 = checkOrientation(p2, q2, q1);
	
	if ((p1q1p2 != p1q1q2) && (p2q2p1 != p2q2q1)) {
		return true;
	}

	if (p1q1p2 == 0 && doCollinearIntersects(p1, q1, p2)) return true;
	if (p1q1q2 == 0 && doCollinearIntersects(p1, q1, q2)) return true;
	if (p2q2p1 == 0 && doCollinearIntersects(p2, q2, p1)) return true;
	if (p2q2q1 == 0 && doCollinearIntersects(p2, q2, q1)) return true;

	return false;
};

bool doCollinearIntersects(point2D p, point2D q, point2D r) {

	if ((r.x <= std::max(p.x, q.x) && r.x >= std::min(p.x, q.x)) && (r.y <= std:: max(p.y, q.y) && r.y >= std::min(p.y, q.y))) return true;
	return false;
}