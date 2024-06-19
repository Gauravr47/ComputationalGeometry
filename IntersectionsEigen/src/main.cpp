#include "primitives.h"
//#include "OpenGLFunctions.h"
using namespace std;

int main() {
	try {
		Solid s("testFiles/cube_hole.stl");
		//std::ofstream ofs("test.log");
		//std::cout << "\n Archiving";
		//// save data to archive
		//{
		//	boost::archive::text_oarchive oa(ofs);
		//	// write class instance to archive
		//	oa << s;
		//	// archive and stream closed when destructors are called
		//}

		vector<point3d> intersections;
		point3d planeNormal;
		planeNormal = getPlaneNormal(0, 0, 45);
		point3d planePoint(50, 0.0, -50.0);
		point3d lightUnitVec;
		lightUnitVec = getPlaneNormal(0.0, 0.0, 45);
		point3d zPlane;
		zPlane = getPlaneNormal(0, 0, 90);
		vector<edge> edges = std::move(s.getEdges());
		slicingPlaneIntersection(planeNormal, planePoint, edges, intersections);
		std::cout << "\n Done intersecting";
		Contour C(segSolidIntersection(lightUnitVec, s), s);
		auto projRings = project(planeNormal, planePoint, lightUnitVec, C);

		vector<point3d> hull;
		boost::geometry::convex_hull(intersections, hull);

		bg::model::polygon<point2d> hull2d;
		auto tranAngle = planeNormal.dot(zPlane);
		auto tranAxis = planeNormal.cross(zPlane);
		Eigen::Transform<float,3, Eigen::Affine> trans(Eigen::AngleAxis(acos(tranAngle), tranAxis));
		for (auto& loop : projRings) {
			for (auto const& p : loop) {
				point3d temp = trans * p;
				bg::append(hull2d.outer(), point2d{temp.x(), temp.y()});
			}
		}

		string style = "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2";
		string svg_file = "hull.svg";
		create_svg(svg_file, hull2d, style);

		std::cout << "\n Hull";
		for (auto const intS : hull) {
			std::cout << "\n\n" << intS;
		}



		//// ... some time later restore the class instance to its orginal state
		//std::cout << "\n Retreiving archive";
		//Solid newg;
		//{
		//	// create and open an archive for input
		//	std::ifstream ifs("test.log");
		//	boost::archive::text_iarchive ia(ifs);
		//	// read class state from archive
		//	ia >> newg;
		//	// archive and stream closed when destructors are called
		//}


	}
	catch (exception& e) {
		std::cerr << e.what();
	}

	return 0;
}