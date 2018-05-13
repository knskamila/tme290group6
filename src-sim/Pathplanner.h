
#ifndef OPENDLV_LOGIC_TEST_KIWI_PATHPLANNER_H
#define OPENDLV_LOGIC_TEST_KIWI_PATHPLANNER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include "Graph.h"
#include "stringtoolbox.hpp"

class Pathplanner
{
public:
    //Pathplanner();
    Pathplanner(std::string, float xBase, float yBase, int height, int width, float dx, float xBegin, float yBegin, float xGoal, float yGoal) noexcept;
    std::list<std::pair<float,float>> getPath();

private:
    //int nNodes;
    //listOfConnections *connections;
    bool cmpf(float, float);
    void swap(float*, float*);
    int pointToNode(float xbase, float ybase, float x, float y, int height, float dx);
    std::pair<float, float> nodeToPoint(int node, int width, float dx);
    std::list<std::pair<float, float>> nodeToPoint(std::vector<int> nodelist, int width, float dx);
    bool notPartOf(std::vector<int> vectorList, int elem);
    void createGraph(std::string filename, float xbase, float ybase, int height, int width, float dx);


    std::list<std::pair<float,float>> path;
    Graph g;
};

#endif //OPENDLV_LOGIC_TEST_KIWI_PATHPLANNER_H
