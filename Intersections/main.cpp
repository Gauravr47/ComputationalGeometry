#include "primitives.h"
#include <set>
using namespace std;

double fRand(double fMin, double fMax) {
    return fMin + (static_cast<double>(rand()) / RAND_MAX) * (fMax - fMin);
}

std::vector<tPointd> getPointSet(int num_of_points) {
    // Generate random points and insert into the set to ensure uniqueness
    set<tPointd> pointSet;
    while (pointSet.size() < num_of_points) {
        tPointd temp;
        temp[0] = fRand(-1.0, 1.0);
        temp[1] = fRand(-1.0, 1.0);
        temp[2] = fRand(-1.0, 1.0);
        pointSet.insert(temp);
    }

    vector<tPointd> ans;
    for (const double* temp : pointSet) {
        ans.push_back(temp);
    }

    return ans;
}

int main()
{
    vector<tPointd> input;
    input = getPointSet(20);
    std::cout << "Generated Points:" << std::endl;
    for (const tPointd& point : input) {
        std::cout << point << std::endl;
    }
	return 0;
}