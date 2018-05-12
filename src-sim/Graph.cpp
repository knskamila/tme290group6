#include "Graph.h"

Graph::Graph(){}

Graph::Graph(int nNodes)
{
    this->nNodes = nNodes;
    this->connections = new listOfConnections[nNodes];
}

Graph::Graph(float* arr, int nNodes)
{
    //example
    // 0 1 0 0 5
    // 1 0 1 0 0
    // 0 1 0 1 0
    // 0 0 1 0 2
    // 5 0 0 2 0
    //
    // testarray[] = {0, 1, 0, 0, 5, 1, 0, 1, 0, 0 , 0, 1, 0, 1, 0, 0, 0, 1, 0, 2, 5, 0, 0, 2, 0};
    // Graph gh(testarray, 5);
    // g.dijkstra(0,3) ->
    // distance: 3
    // path:
    // 0
    // 1
    // 2
    // 3

    this->nNodes = nNodes;
    this->connections = new listOfConnections[nNodes];

    //iterate through the array and add nodes
    for(int px = 0; px < nNodes; px++) {
        for (int py = 0; py < nNodes; py++) {
            float weight = *(arr + nNodes * px + py);
            if (weight != 0) {
                connections[px].push_back(std::make_pair(py, weight));
            }
        }
    }
}

void Graph::addEdge(int src, int dest, float weight)
{
    connections[src].push_back(std::make_pair(dest, weight));
    connections[dest].push_back(std::make_pair(src, weight));
}

std::list<int> Graph::dijkstra(int src, int dest)
{
    std::priority_queue<connection, std::vector<connection>, std::greater<connection>> searchingQueue;

    std::vector<float> distance(nNodes, INF);

    searchingQueue.push(std::make_pair(0, src));
    distance[src] = 0;

    int previousList[nNodes];
    previousList[(int)searchingQueue.top().second] = src;
    std::list<int> path;

    while(!searchingQueue.empty())
    {
        int currentV = searchingQueue.top().second;
        searchingQueue.pop();

        for(listOfConnections::iterator i = connections[currentV].begin(); i != connections[currentV].end(); i++)
        {
            int nextV = (*i).first;
            float weight = (*i).second;

            if(distance[nextV] > distance[currentV] + weight)
            {
                distance[nextV] = distance[currentV] + weight;
                searchingQueue.push(std::make_pair(distance[nextV], nextV));
                previousList[nextV] = currentV;
            }
        }
    }

    int prev = dest;
    path.push_front(prev);
    while(prev != src)
    {
        prev = previousList[prev];
        path.push_front(prev);
    }
    std::cout << "distance: " << distance[dest] << std::endl;
    std::cout << "path: " << std::endl;
    for(auto v: path)
    {
        std::cout << v << std::endl;
    }

    return path;
}

void Graph::printInfo()
{
    std::cout << "size: " << this->nNodes << std::endl;
    std::cout << "connections:" << std::endl;
    for(int i = 0; i<nNodes; i++)
    {
        std::cout << i << " connects with:" << std::endl;
        listOfConnections pairs = connections[i];
        for(auto v: pairs)
        {
            std::cout << " " << v.first << " with weight " << v.second << std::endl;
        }
    }
}

void Graph::printMap(int width)
{
    int currentColumn = 1;
    for(int i = 0; i<nNodes; i++)
    {
        listOfConnections pairs = connections[i];
        if(pairs.size() == 0)
        {
            std::cout << "#";
        }
        else
        {
            std::cout << "_";
        }
        currentColumn++;
        if(currentColumn > width)
        {
            std::cout << std::endl;
            currentColumn = 1;
        }
    }
}

void Graph::zeroNode(int node)
{
    connections[node].clear();
}