#include <queue>
#include <stack>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "planning.h"

void printPath(std::vector<int>& path, Graph& g) {

    if (path.size() < 1)
    {
        std::cout << "No path found :(\n";
        return;
    }

    std::cout << "Path: ";
    for (int i = 0; i < path.size() - 1; i++)
    {
        std::cout << g.data[path[i]] << " -> ";
    }
    std::cout <<  g.data[path.back()] << "\n";
};

std::vector<int> tracePath(int n, Graph& g) {
    std::vector<int> path;
    int curr = n;
    do {
        path.push_back(curr);
        curr = getParent(curr, g);
    } while (curr != -1);

    // Since we built the path backwards, we need to reverse it.
    std::reverse(path.begin(), path.end());
    return path;
};

std::vector<int> getNeighbors(int n, Graph& g)
{
    return g.edges[n];
}

std::vector<float> getEdgeCosts(int n, Graph& g)
{
    return g.edge_costs[n];
}

int getParent(int n, Graph& g)
{
    // *** Task: Implement this function *** //
     Node node = g.nodes[n];
    std::cout<< "node index: " << node.parent << "\n";
    int index  = node.parent;
    return index;
    return -1;

    // *** End student code *** //
}

void initGraph(Graph& g)
{
    g.nodes.clear();
    for (int i = 0; i < g.data.size(); i++)
    {
        Node n;
        n.city = g.data[i];
        g.nodes.push_back(n);
    }
}

std::vector<int> bfs(int start, int goal, Graph& g)
{
    
    initGraph(g);
    std::vector<int> path;
    std::queue<int> visit_queue; 
    std::vector<bool> visited(g.nodes.size(), false);
    int current_node_index;
    std::vector<int> neighbors;
    int neighbor_index;
    bool isVisited;
    int node_index;

    // first location is automatically visited  
    visit_queue.push(start);
    g.nodes[start].visited = true; 


    // while there are still locations to be visited 
    while(visit_queue.size() > 0){
        // add current node to queue
        current_node_index = visit_queue.front();
        visit_queue.pop();

        // trace path when goal is reached 
        if (current_node_index == goal){
            path = tracePath(current_node_index, g);
            return path;
        }

        neighbors = getNeighbors(current_node_index, g);

        for (int i = 0; i < neighbors.size(); i++){
            neighbor_index = neighbors[i];
            isVisited = g.nodes[neighbor_index].visited;

            // if neighbor is not visited, visit it and add its neighbors to queue
            if (!isVisited){
                g.nodes[neighbor_index].visited = true;
                g.nodes[neighbor_index].parent = current_node_index;
                visit_queue.push(neighbor_index);
            }
        }
    }
    return path;
    // *** End student code *** //
}

std::vector<int> dfs(int start, int goal, Graph& g)
{
    initGraph(g);
    std::vector<int> path;
    std::stack<int> visit_stack; 
    // *** Task: Implement this function if completing the advanced extension *** //
    
    // *** End student code *** //

    return path;
}
