#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <list>
#include <queue>
#include <vector>
#include <algorithm>

#define INF 0x3f3f3f3f

typedef std::pair<int, float> connection;
typedef std::list<connection> listOfConnections;

class Graph
{
public:
    Graph();
    Graph(int);
    Graph(float*, int);
    void addEdge(int, int, float);
    std::vector<int> dijkstra(int, int);
    std::vector<int> simplifyPath(std::vector<int>, int);
    void printInfo(std::vector<int>);
    float getConnection(int, int);
    void printMap(int, std::vector<int>);
    int getLinearityType(int, int, int);
    void zeroNode(int);
    int nNodes;

private:
    //int nNodes;
    listOfConnections *connections;

};

#endif //GRAPH_H
