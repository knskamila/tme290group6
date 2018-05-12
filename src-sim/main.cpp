#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include "Graph.h"
#include "stringtoolbox.hpp"

using namespace std;

/*
vector<string> split(const string & str, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(str);
    while(getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}
*/
bool cmpf (float a, float b, float tol = 0.005f) {
    return (fabs (a - b) < tol);
}

void swap(float *a, float *b)
{
    if(*a > *b)
    {
        float temp = *a;
        *a = *b;
        *b = temp;
    }
}
/*
vector<pair<float, float>> coordToPoints(const string & coordinates) {
    vector<string> tmp;
    vector<string> stmp;
    vector<float> tp;
    vector<pair<float,float>> coord;

    stmp = split(coordinates, ';');
    tmp = split(stmp.front(), ',');

    for (string s: tmp) {
        tp.push_back(strtof(s.c_str(),0));
    }

    // shit, this assumes that neg always written before pos, else wont work
    if (tp.size() == 4) {
        if (cmpf(tp.at(1), tp.at(3)) ) {
            for (float i = tp.at(0); i <= tp.at(2); i += 1) {
                float yval = tp.at(1);
                coord.push_back(make_pair(i, yval));
            }
        } else if (cmpf(tp.at(0), tp.at(2))) {
            for (float i = tp.at(1); i <= tp.at(3); i += 1) {
                float yval = tp.at(0);
                coord.push_back(make_pair(i, yval));
            }
        } else {
            float m = (tp.at(1) - tp.at(3))/ (tp.at(0) - tp.at(2));
            float c = tp.at(1) - m* tp.at(0);
            for (float i = tp.at(0); i <= tp.at(2); i += 1) {
                float yval = m*i + c;
                coord.push_back(make_pair(i, yval));
            }
        }
    }

    return coord;
}

*/
int pointToNode(float xbase, float ybase, float x, float y, int height, float dx) {

    //xbase -= 1/(2*dx);
    //ybase -= 1/(2*dx);

    int xGrid = (int)((x - xbase)/dx + 0.5);
    int yGrid = (int)((y - ybase)/dx + 0.5);

    return yGrid + xGrid*(float)height/dx;

    //float sqrtNodes = (float) sqrt(nrNodes);
   /* int counter = 0;
    for (float i = xbase; i < xbase + width ; i++) {
        for (float j = ybase; j < ybase + height ; j++) {
            if ( cmpf(i, x) && cmpf (j, y) ) {
                return counter;
            } else {
                counter++;
            }
        }
    }
    return counter;*/
}


bool notPartOf(vector<int> vectorList, int elem) {
    return (find(vectorList.begin(), vectorList.end(), elem) == vectorList.end());
}


void createGraph(string filename, float xbase, float ybase, int height, int width, float dx) {
    string tmpstr;
    vector<string> lines;
    ifstream txtfile;
    //float xbase = -1;
    //float ybase = -1;
    //int nrNodes = 16;
    float multiplier = 1/dx;
    //int trim = multiplier - 1;
    int nrNodes = height*width*multiplier*multiplier;
    int sqrtNodes = height*multiplier;
    //int xlength = xbase + sqrtNodes;
    //int ylength = ybase + sqrtNodes;
    Graph g(nrNodes);
    vector<pair<float, float>> wallPoints;
    vector<int> wallNodes;

    std::ifstream input(filename.c_str());
    std::cout << input.is_open() << std::endl;
    for (std::string str; getline(input, str);) {
        std::vector<std::string> coordinates = stringtoolbox::split(
                stringtoolbox::split(stringtoolbox::trim(str), ';')[0], ',');
        if (coordinates.size() == 4) {
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
            //wallPoints.push_back();
        }
    }
/*
    txtfile.open(filename.c_str());
    while(!txtfile.eof())
    {
        getline(txtfile, tmpstr);
        lines.push_back(tmpstr);
    }
    txtfile.close();

    for (int i = 0;  i < lines.size() - 1; i ++) {
        vector<pair<float, float>> tmpVec = coordToPoints(lines.at(i));
        for (int j = 0; j < tmpVec.size(); j ++ ) {
            wallPoints.push_back(tmpVec.at(j));
        }
    }
*/

    // insert some kind of base value, ask how one can set these somewhere easily accessible?

    for (int i = 0; i < wallPoints.size() ; i ++) {
        wallNodes.push_back(pointToNode(xbase, ybase, wallPoints[i].first, wallPoints[i].second, height, dx));
    }

    std::cout << "wallNodes:" << std::endl;
    for(auto v: wallNodes)
    {
        std::cout << v << std::endl;
    }

    for (int i = 0; i < nrNodes; i++) {
        if (notPartOf(wallNodes, i)) {
            if ((i % sqrtNodes != (sqrtNodes-1)) && (i % sqrtNodes !=0) && (i > sqrtNodes) && (i < (nrNodes - sqrtNodes)) ) {
                if (notPartOf(wallNodes, i+1) )
                    g.addEdge(i, i+1, 1);
                if (notPartOf(wallNodes, i-1))
                    g.addEdge(i, i-1, 1);
                if (notPartOf(wallNodes, i + sqrtNodes))
                    g.addEdge(i, i+ sqrtNodes, 1);
                if (notPartOf(wallNodes, i - sqrtNodes))
                    g.addEdge(i, i - sqrtNodes, 1);
                if (notPartOf(wallNodes, i - sqrtNodes - 1) )
                    g.addEdge(i, i - sqrtNodes -1 , sqrt(2));
                if (notPartOf(wallNodes, i - sqrtNodes +1))
                    g.addEdge(i, i - sqrtNodes +1 , sqrt(2));
                if (notPartOf(wallNodes, i + sqrtNodes -1 ))
                    g.addEdge(i, i + sqrtNodes -1, sqrt(2));
                if (notPartOf(wallNodes, i + sqrtNodes +1))
                    g.addEdge(i, i + sqrtNodes +1, sqrt(2));

            } else if (i < sqrtNodes) {
                if (notPartOf(wallNodes, i + 1) && (i != sqrtNodes) )
                    g.addEdge(i, i+1, 1);
                if (notPartOf(wallNodes, i-1) && (i != 0))
                    g.addEdge(i, i-1, 1);
            } else if (i > (nrNodes - sqrtNodes)) {
                if (notPartOf(wallNodes, i + 1) && (i != nrNodes -1) )
                    g.addEdge(i, i+1, 1);
                if (notPartOf(wallNodes, i-1) && (i != (nrNodes - sqrtNodes -1)))
                    g.addEdge(i, i-1, 1);

            } else if (i % sqrtNodes == sqrtNodes -1) {
                if (notPartOf(wallNodes, i + sqrtNodes) && (i != nrNodes -1))
                    g.addEdge(i, i+ sqrtNodes, 1);
                if (notPartOf(wallNodes, i - sqrtNodes) && (i != sqrtNodes -1))
                    g.addEdge(i, i - sqrtNodes, 1);

            } else if (i % sqrtNodes == 0) {
                if (notPartOf(wallNodes, i + sqrtNodes) && (i != (nrNodes-sqrtNodes )))
                    g.addEdge(i, i+ sqrtNodes, 1);
                if (notPartOf(wallNodes, i - sqrtNodes) && (i != 0))
                    g.addEdge(i, i - sqrtNodes, 1);

            }
        }
    }
    g.printInfo();
    g.printMap(width*multiplier);
}



int main() {
    //coordToPoints("1.0, -1.0, 1.0, 1.0;");
    createGraph("simulation-map.txt", -3, -3, 7, 7, 0.1);
}

