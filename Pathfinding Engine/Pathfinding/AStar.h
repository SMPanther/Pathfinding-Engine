#pragma once
#include "Graph/Graph.h"
#include "C:\VISUAL STUDIO PROJECTS\Pathfinding Engine\Pathfinding Engine\Data Structures\DataStructures.h"
#include <cmath>
#include <cfloat>
// ============================================================
//  AStar.h  —  A* Pathfinding Algorithm
//  UCP BSCS — Data Structures Project
//
//  f(n) = g(n) + h(n)
//    g(n) = actual cost from start to n
//    h(n) = Euclidean distance heuristic from n to goal
//
//  Data structures used:
//    MinHeap   → open set   (always expand lowest f(n) first)
//    HashSet   → closed set (O(1) visited lookup)
//    HashMap   → cameFrom   (nodeId → parentId, for reconstruction)
//    HashMap   → gCost      (nodeId → best g(n) so far, as float)
//    Stack     → final path (start at top, goal at bottom... reversed)
// ============================================================

class AStar {
private:
    // ── Heuristic: Euclidean distance ────────────────────────
    static float heuristic(const Graph& graph, int nodeId, int goalId) {
        const GraphNode& n = graph.getNode(nodeId);
        const GraphNode& g = graph.getNode(goalId);
        float dx = n.x - g.x;
        float dy = n.y - g.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // ── Path reconstruction using Stack reversal ─────────────
    // cameFrom holds child→parent pairs.
    // We trace goal→start backwards, push each node onto
    // reversedPath, then pop into finalPath to get start→goal order.
    static Stack<int> reconstructPath(HashMap& cameFrom, int start, int goal) {
        Stack<int> reversed;
        Stack<int> finalPath;

        int current = goal;
        while (current != start) {
            reversed.push(current);
            int parent = -1;
            if (!cameFrom.tryGet(current, parent)) break;
            current = parent;
        }
        reversed.push(start);

        // Reverse: pop reversed → push into finalPath
        // finalPath top = start, bottom = goal
        while (!reversed.isEmpty())
            finalPath.push(reversed.pop());

        return finalPath;
    }

public:
    // ── Result struct ─────────────────────────────────────────
    struct Result {
        Stack<int> path;       // node IDs from start (top) to goal
        bool       found;
        int        nodesExpanded;
        float      totalCost;

        Result() : found(false), nodesExpanded(0), totalCost(0.0f) {}
    };

    // ── Main A* search ────────────────────────────────────────
    static Result search(const Graph& graph, int startId, int goalId) {
        Result result;

        if (!graph.nodeExists(startId) || !graph.nodeExists(goalId))
            return result;

        if (startId == goalId) {
            result.path.push(startId);
            result.found    = true;
            result.totalCost = 0.0f;
            return result;
        }

        MinHeap openSet;
        HashSet closedSet;
        HashMap cameFrom;
        HashMap gCostMap;   // stores floats via bit-cast

        // Initialise start
        gCostMap.insertFloat(startId, 0.0f);
        float h = heuristic(graph, startId, goalId);
        openSet.insert(PathNode(startId, h, 0.0f));

        while (!openSet.isEmpty()) {
            PathNode current = openSet.extractMin();
            result.nodesExpanded++;

            // Goal reached
            if (current.nodeId == goalId) {
                result.path      = reconstructPath(cameFrom, startId, goalId);
                result.found     = true;
                result.totalCost = current.gCost;
                return result;
            }

            // Skip if already in closed set (stale entry)
            if (closedSet.contains(current.nodeId)) continue;
            closedSet.insert(current.nodeId);

            // Expand neighbors
            for (const Edge& edge : graph.getNeighbors(current.nodeId)) {
                int   neighborId = edge.targetId;
                if (closedSet.contains(neighborId)) continue;

                float currentG = 0.0f;
                gCostMap.tryGetFloat(current.nodeId, currentG);
                float tentativeG = currentG + edge.weight;

                float neighborG = FLT_MAX;
                bool  known     = gCostMap.tryGetFloat(neighborId, neighborG);

                if (!known || tentativeG < neighborG) {
                    // Better path found to neighbor
                    cameFrom.insert(neighborId, current.nodeId);
                    gCostMap.insertFloat(neighborId, tentativeG);

                    float f = tentativeG + heuristic(graph, neighborId, goalId);

                    if (openSet.contains(neighborId))
                        openSet.decreaseKey(neighborId, f, tentativeG);
                    else
                        openSet.insert(PathNode(neighborId, f, tentativeG));
                }
            }
        }

        return result;  // no path found
    }
};
