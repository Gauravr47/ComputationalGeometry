
#include "points.h"
#include "sweepLine.h"

using namespace std;



int main()
{
    segment arr[] = { {{1, 5}, {4, 5}}, {{2, 5}, {10, 1}},{{3, 2}, {10, 3}},{{6, 4}, {9, 4}},{{7, 1}, {8, 1}} };
    int n = sizeof(arr) / sizeof(arr[0]);
    cout <<"\n"<< checkInteresections(arr, n);
    return 0;
}