#include <iostream>
#include <cmath>
#include <queue>
#include <functional>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do that too.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize which cells are visited in the navigation webapp, save each
 * visited cell in the vector in the graph struct as follows:
 *      graph.visited_cells.push_back(c);
 * where c is a Cell struct corresponding to the visited cell you want to
 * visualize.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
*/

std::vector<Cell> depthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    // *** Task: Implement this function if completing the advanced extensions *** //

    // *** End student code *** //

    return path;
}





std::vector<Cell> breadthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; 
    initGraph(graph); 

    int start_index = cellToIdx(start.i, start.j, graph);
    int goal_index = cellToIdx(goal.i, goal.j, graph);

    
    graph.nodes[start_index].distance = 0; 

    
    std::queue<int> queue;
    queue.push(start_index);

    while (!queue.empty())
    {

        int current_index = queue.front();
        queue.pop();

       
        if (current_index == goal_index) {
            return tracePath(goal_index, graph);
        }

        if (graph.nodes[current_index].visited){
            continue; 
        }
        graph.nodes[current_index].visited = true;
        std::vector<int> neighbors = findNeighbors(current_index, graph);
        for (int neighbor_index : neighbors) {
            Cell current_cell = idxToCell(current_index, graph);
            Cell current_neighbor = idxToCell(neighbor_index, graph);
            
            if (!graph.nodes[neighbor_index].visited && !checkCollision(neighbor_index, graph)) {
                queue.push(neighbor_index); 
 

                
                float cost = sqrt(pow(current_cell.i - current_neighbor.i, 2) + pow(current_cell.j - current_neighbor.j, 2));
                float new_distance = graph.nodes[current_index].distance + cost;

                
                if (new_distance < graph.nodes[neighbor_index].distance) { 
                    graph.nodes[neighbor_index].parent = current_index;
                    graph.nodes[neighbor_index].distance = new_distance;
                }
            }
        }
    }

    
    return path;
}




std::vector<Cell> iterativeDeepeningSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    // *** Task: Implement this function if completing the advanced extensions *** //

    // *** End student code *** //

    return path;
}

std::vector<Cell> aStarSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    // *** Task: Implement this function if completing the advanced extensions *** //

    // *** End student code *** //

    return path;
}
