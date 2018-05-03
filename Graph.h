#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <list>
#include <queue>
#include <vector>

#define INF 0x3f3f3f3f

typedef std::pair<int, int> connection;
typedef std::list<connection> listOfConnections;

class Graph
{
public:
    Graph();
    Graph(int);
    Graph(int*, int);
    void addEdge(int, int, int);
    std::list<int> dijkstra(int, int);
    void printInfo();

private:
    int nNodes;
    listOfConnections *connections;

};

#endif //GRAPH_H
