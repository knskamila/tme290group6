#include "Pathplanner.h"

using namespace std;

bool Pathplanner::cmpf(float a, float b) {
    float tol = 0.005f;
    return (fabs (a - b) < tol);
}

void Pathplanner::swap(float *a, float *b)
{
    if(*a > *b)
    {
        float temp = *a;
        *a = *b;
        *b = temp;
    }
}

int Pathplanner::pointToNode(float xbase, float ybase, float x, float y, int width, float dx)
{
    int xGrid = (int) ((x - xbase) / dx + 0.5);
    int yGrid = (int) ((y - ybase) / dx + 0.5);

    return yGrid + xGrid * (float) width / dx;
}

pair<float, float> Pathplanner::nodeToPoint(int node, int width, float xBase, float yBase, float dx)
{
    int nodesPerMeter = 1/dx;
    float xNode = xBase + node/(width*nodesPerMeter)*dx;
    float yNode = yBase + node%(width*nodesPerMeter)*dx;
    return std::make_pair(xNode, yNode);
}

list<pair<float, float>> Pathplanner::nodeToPoint(std::vector<int> nodelist, int width, float xBase, float yBase, float dx)
{
    list<pair<float, float>> pointList;
    for (auto v: nodelist)
    {
        pointList.push_back(nodeToPoint(v, width, xBase, yBase, dx));
    }

    return pointList;
}

bool Pathplanner::notPartOf(vector<int> vectorList, int elem) {
    return (find(vectorList.begin(), vectorList.end(), elem) == vectorList.end());
}


void Pathplanner::createGraph(string filename, float xbase, float ybase, int width, float dx) {
    string tmpstr;
    vector<string> lines;
    ifstream txtfile;

    float multiplier = 1/dx;

    int nrNodes = width*width*multiplier*multiplier;
    int sqrtNodes = width*multiplier;

    g = Graph(nrNodes);
    vector<pair<float, float>> wallPoints;
    vector<int> wallNodes;

    std::fstream input(filename);
    for (std::string str; getline(input, str);) {
        std::vector<std::string> coordinates = stringtoolbox::split(
                stringtoolbox::split(stringtoolbox::trim(str), ';')[0], ',');
        if (coordinates.size() == 4) {
            std::cout << "right size" << std::endl;
            vector<float> tp;
            for(int i = 0; i < 4; i++)
            {
                tp.push_back(std::stof(coordinates[i]));
            }
            swap(&tp.at(0), &tp.at(2));
            swap(&tp.at(1), &tp.at(3));

            if (cmpf(tp.at(1), tp.at(3)) ) {
                for (float i = tp.at(0); i <= tp.at(2); i += dx) {
                    float yval = tp.at(1);
                    wallPoints.push_back(make_pair(i, yval));
                }
            } else if (cmpf(tp.at(0), tp.at(2))) {

                for (float i = tp.at(1); i <= tp.at(3); i += dx) {
                    float xval = tp.at(0);
                    wallPoints.push_back(make_pair(xval, i));
                }

            } else {
                float slope = (tp.at(1) - tp.at(3))/ (tp.at(0) - tp.at(2));
                float c = tp.at(1) - slope*tp.at(0);
                for (float i = tp.at(0); i <= tp.at(2); i += dx) {
                    float yval = slope*i + c;
                    wallPoints.push_back(make_pair(i, yval));
                }
                for (float i = tp.at(1); i <= tp.at(3); i += dx) {
                    float xval = (i - c)/ slope;
                    wallPoints.push_back(make_pair(xval, i));
                }
            }
        }
    }


    for (int i = 0; i < wallPoints.size() ; i ++) {
        wallNodes.push_back(pointToNode(xbase, ybase, wallPoints[i].first, wallPoints[i].second, width, dx));
    }

    for (int i = 0; i < nrNodes; i++) {
        if (notPartOf(wallNodes, i)) {
            // nodes with no walls
            if ((i % sqrtNodes != (sqrtNodes - 1)) && (i % sqrtNodes != 0) && (i > sqrtNodes) &&
                (i < (nrNodes - sqrtNodes))) {
                if (notPartOf(wallNodes, i + 1)) {
                    g.addEdge(i, i + 1, 1);
                }
                if (notPartOf(wallNodes, i - 1)) {
                    g.addEdge(i, i - 1, 1);
                }
                if (notPartOf(wallNodes, i + sqrtNodes)) {
                    g.addEdge(i, i + sqrtNodes, 1);
                }
                if (notPartOf(wallNodes, i - sqrtNodes)) {
                    g.addEdge(i, i - sqrtNodes, 1);
                }
                if (notPartOf(wallNodes, i - sqrtNodes - 1)) {
                    g.addEdge(i, i - sqrtNodes - 1, sqrt(2));
                }
                if (notPartOf(wallNodes, i - sqrtNodes + 1)) {
                    g.addEdge(i, i - sqrtNodes + 1, sqrt(2));
                }
                if (notPartOf(wallNodes, i + sqrtNodes - 1)) {
                    g.addEdge(i, i + sqrtNodes - 1, sqrt(2));
                }
                if (notPartOf(wallNodes, i + sqrtNodes + 1)) {
                    g.addEdge(i, i + sqrtNodes + 1, sqrt(2));
                }
            }
        }
    }

    for (int i = 0; i < nrNodes; i++) {
        if (notPartOf(wallNodes, i)) {
            // nodes with no walls
            if ((i % sqrtNodes != (sqrtNodes-1)) && (i % sqrtNodes !=0) && (i > sqrtNodes) && (i < (nrNodes - sqrtNodes)) ) {
                if (!notPartOf(wallNodes, i + 1)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i - 1)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i + sqrtNodes)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i - sqrtNodes)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i - sqrtNodes - 1)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i - sqrtNodes + 1)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i + sqrtNodes - 1)) {
                    g.zeroNode(i);
                }
                if (!notPartOf(wallNodes, i + sqrtNodes + 1)) {
                    g.zeroNode(i);
                }
            }
        }
    }
}

std::list<std::pair<float,float>> Pathplanner::getPath()
{
    std::list<std::pair<float,float>> path2;
    path2 = path;
    path2.pop_front();
    return path2;
}

Pathplanner::Pathplanner(string filename, float xBase, float yBase, int width, float dx, float xBegin, float yBegin, float xGoal, float yGoal) noexcept {
    createGraph(filename, xBase, yBase, width, dx);
    int nodeBegin = pointToNode(xBase, yBase, xBegin, yBegin, width, dx);
    int nodeGoal = pointToNode(xBase, yBase, xGoal, yGoal, width, dx);
    std::vector<int> nodePath = g.dijkstra(nodeBegin, nodeGoal);
    g.printInfo(nodePath);
    nodePath = g.simplifyPath(nodePath, width);
    g.printMap(width * (1 / dx), nodePath);
    path = nodeToPoint(nodePath, width, xBase, yBase, dx);
    for (auto v:path)
    {
        std::cout << v.first << ", " << v.second << std::endl;
    }
}
