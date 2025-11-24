#include <iostream>
#include <cmath>
#include <queue>

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

namespace
{
float heuristicCost(int from_idx, int goal_idx, const GridGraph& graph)
{
    Cell from_cell = idxToCell(from_idx, graph);
    Cell goal_cell = idxToCell(goal_idx, graph);
    float di = static_cast<float>(from_cell.i - goal_cell.i);
    float dj = static_cast<float>(from_cell.j - goal_cell.j);
    return std::sqrt(di * di + dj * dj);
}
}

std::vector<Cell> depthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    std::vector<int> stack;
    stack.push_back(start_idx);
    graph.nodes[start_idx].visited = true;

    while (!stack.empty())
    {
        int current = stack.back();
        stack.pop_back();

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            return tracePath(current, graph);
        }

        for (int neighbor : findNeighbors(current, graph))
        {
            if (graph.nodes[neighbor].visited)
            {
                continue;
            }

            graph.nodes[neighbor].visited = true;
            graph.nodes[neighbor].parent_idx = current;
            stack.push_back(neighbor);
        }
    }

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    std::queue<int> frontier;
    frontier.push(start_idx);
    graph.nodes[start_idx].visited = true;

    while (!frontier.empty())
    {
        int current = frontier.front();
        frontier.pop();

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            return tracePath(current, graph);
        }

        for (int neighbor : findNeighbors(current, graph))
        {
            if (graph.nodes[neighbor].visited)
            {
                continue;
            }

            graph.nodes[neighbor].visited = true;
            graph.nodes[neighbor].parent_idx = current;
            frontier.push(neighbor);
        }
    }

    return path;
}

std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    std::vector<int> open_set;
    open_set.push_back(start_idx);

    graph.nodes[start_idx].g_cost = 0.0f;
    graph.nodes[start_idx].h_cost = heuristicCost(start_idx, goal_idx, graph);
    graph.nodes[start_idx].f_cost = graph.nodes[start_idx].h_cost;
    graph.nodes[start_idx].in_open_set = true;

    while (!open_set.empty())
    {
        int lowest_idx_in_open = findLowestScore(open_set, graph);
        int current = open_set[lowest_idx_in_open];
        open_set.erase(open_set.begin() + lowest_idx_in_open);
        graph.nodes[current].in_open_set = false;

        if (graph.nodes[current].visited)
        {
            continue;
        }

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            return tracePath(current, graph);
        }

        graph.nodes[current].visited = true;

        for (int neighbor : findNeighbors(current, graph))
        {
            if (graph.nodes[neighbor].visited)
            {
                continue;
            }

            float tentative_g = graph.nodes[current].g_cost + 1.0f;
            if (tentative_g >= graph.nodes[neighbor].g_cost)
            {
                continue;
            }

            graph.nodes[neighbor].parent_idx = current;
            graph.nodes[neighbor].g_cost = tentative_g;
            graph.nodes[neighbor].h_cost = heuristicCost(neighbor, goal_idx, graph);
            graph.nodes[neighbor].f_cost = graph.nodes[neighbor].g_cost + graph.nodes[neighbor].h_cost;

            if (!graph.nodes[neighbor].in_open_set)
            {
                open_set.push_back(neighbor);
                graph.nodes[neighbor].in_open_set = true;
            }
        }
    }

    return path;
}
