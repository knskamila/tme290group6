#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <list>
#include <queue>
#include <vector>

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
    std::list<int> dijkstra(int, int);
    void printInfo();
    int nNodes;

private:
    //int nNodes;
    listOfConnections *connections;

};

#endif //GRAPH_H
