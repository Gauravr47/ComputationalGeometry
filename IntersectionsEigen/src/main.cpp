#include "primitives.h"
#include "solid.h"
#include "contour.h"
//#include "OpenGLFunctions.h"
using namespace std;

int main() {
	try {
		Solid s("testFiles/cube_edge_hole.stl");

		box bounds = s.getBoundingBox();
		vector<point3d> intersections;
		point3d planeNormal;
		planeNormal = getPlaneNormal(90, 45);
		point3d planePoint(planeNormal* -100);
		point3d lightUnitVec;
		lightUnitVec = -1 * planeNormal;
		point3d zPlane;
		zPlane = getPlaneNormal(90, 90);


		Contour C(segSolidIntersection(lightUnitVec, s), s);
		vector<ring> rings = C.getRings();
		auto projRings = project(planeNormal, planePoint, lightUnitVec, rings , s);

		//Save to svg
		bg::model::polygon<point2d> hull2d;
		auto tranAngle = planeNormal.dot(zPlane);
		auto tranAxis = planeNormal.cross(zPlane);

		Eigen::Transform<float,3, Eigen::Affine> trans(Eigen::AngleAxis(acos(tranAngle), tranAxis));
		int i = 0;
		for (auto& loop : projRings) {
			for (auto const& p : loop) {
				point3d temp = trans* p;
				if(i == 0)
					bg::append(hull2d.outer(), point2d{temp.x(), temp.y()});
				else {
					bg::append(hull2d.inners().back(), point2d{ temp.x(), temp.y() });
				}
			}
			i++;
			hull2d.inners().resize(1);
		}
		
		string style = "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2";
		string svg_file = "hull.svg";
		create_svg(svg_file, hull2d, style);

	}
	catch (exception& e) {
		std::cerr << e.what();
	}

	return 0;
}