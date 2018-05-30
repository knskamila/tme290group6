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
    // Graph g(testarray, 5);
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

std::vector<int> Graph::simplifyPath(std::vector<int> path, int width)
{
    //we can make these assumptions, since certain combinations (/\, |_) will never happen:

    int it = 0;
    std::list<int> toErase;
    while(it < path.size() - 2)
    {
        if(getConnection(path.at(it), path.at(it+1)) == getConnection(path.at(it+1), path.at(it+2)))
        {
            toErase.push_front(it + 1);
        }
        it++;
    }
    for(auto v: toErase)
    {
        path.erase(path.begin() + v);
    }
    return path;
}

std::vector<int> Graph::dijkstra(int src, int dest)
{
    std::priority_queue<connection, std::vector<connection>, std::greater<connection>> searchingQueue;

    std::vector<float> distance(nNodes, INF);

    searchingQueue.push(std::make_pair(0, src));
    distance[src] = 0;

    int previousList[nNodes];
    previousList[(int)searchingQueue.top().second] = src;
    std::vector<int> path;

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
    path.insert(path.begin(), prev);
    while(prev != src)
    {
        prev = previousList[prev];
        path.insert(path.begin(), prev);
    }
    //std::cout << "distance: " << distance[dest] << std::endl;
    //std::cout << "path: " << std::endl;
    //for(auto v: path)
    //{
    //    std::cout << v << std::endl;
    //}

    return path;
}
float Graph::getConnection(int nodeA, int nodeB)
{
    for(listOfConnections::iterator i = connections[nodeA].begin(); i != connections[nodeA].end(); i++)
    {
        if ((*i).first == nodeB)
        {
            return (*i).second;
        }
    }
    return 0;
}

int Graph::getLinearityType(int nodeA, int nodeB, int width)
{
    //4 linearity types: - | \ /
    if(getConnection(nodeA, nodeB) == 1)
    {
        if(nodeA%width == nodeB%width) return 1;
        else return 2;
    }
    else if(getConnection(nodeA, nodeB) > 1)
    {
        if(nodeA%width < nodeB%width) return 3;
        else return 4;
    }
    else return 0;
}

void Graph::printInfo(std::vector<int> path)
{
    std::cout << "connections:" << std::endl;
    for(int i = 0; i < path.size() - 1; i++)
    {
        int nodeA = path.at(i);
        int nodeB = path.at(i+1);
        std::cout << nodeA << " connects with " << nodeB << " through " << getConnection(nodeA, nodeB) << std::endl;
    }
}

void Graph::printMap(int width, std::vector<int> path)
{
    int currentColumn = 1;
    for(int i = 0; i<nNodes; i++)
    {
        listOfConnections pairs = connections[i];
        if(std::find(path.begin(), path.end(), i) != path.end())
        {
            std::cout << "*";
        }
        else if(pairs.size() == 0)
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