#pragma once
#include "Graph/Graph.h"
#include "Data Structures\DataStructures.h"
// ============================================================
//  DFS.h  —  Depth-First Search
//  UCP BSCS — Data Structures Project
//
//  Uses: custom Stack (LIFO) for deep-branch-first expansion
//        HashSet for O(1) visited tracking
//        HashMap for parent reconstruction
//
//  Two modes:
//    1. search(start, goal)   → path Stack (not necessarily shortest)
//    2. fullTraversal(start)  → visits all reachable nodes,
//                               returns visit order as Stack
//
//  Note: DFS paths are NOT guaranteed optimal.
//        Use A* for shortest paths; DFS for connectivity/maze checks.
// ============================================================

class DFS {
private:
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

        while (!reversed.isEmpty())
            finalPath.push(reversed.pop());

        return finalPath;
    }

public:
    // ── Result struct ─────────────────────────────────────────
    struct Result {
        Stack<int> path;
        bool       found;
        int        nodesVisited;

        Result() : found(false), nodesVisited(0) {}
    };

    // ── DFS point-to-point search ─────────────────────────────
    static Result search(const Graph& graph, int startId, int goalId) {
        Result result;

        if (!graph.nodeExists(startId) || !graph.nodeExists(goalId))
            return result;

        if (startId == goalId) {
            result.path.push(startId);
            result.found = true;
            return result;
        }

        Stack<int> frontier;
        HashSet    visited;
        HashMap    cameFrom;

        frontier.push(startId);

        while (!frontier.isEmpty()) {
            int current = frontier.pop();

            if (visited.contains(current)) continue;
            visited.insert(current);
            result.nodesVisited++;

            if (current == goalId) {
                result.path  = reconstructPath(cameFrom, startId, goalId);
                result.found = true;
                return result;
            }

            // Push neighbors onto stack (reverse order so leftmost explored first)
            // Collect into temp array first so we can push in reverse
            int   nbuf[32];
            int   ncount = 0;
            for (const Edge& edge : graph.getNeighbors(current)) {
                if (!visited.contains(edge.targetId)) {
                    if (ncount < 32) nbuf[ncount++] = edge.targetId;
                    // Record first encounter as parent (for reconstruction)
                    if (!cameFrom.containsKey(edge.targetId))
                        cameFrom.insert(edge.targetId, current);
                }
            }
            // Push in reverse so first neighbor is explored first
            for (int i = ncount - 1; i >= 0; i--)
                frontier.push(nbuf[i]);
        }

        return result;  // goal not reachable
    }

    // ── Full DFS traversal from a start node ─────────────────
    // Returns a Stack containing all visited node IDs
    // (top of stack = last visited, useful for topological sort)
    static Stack<int> fullTraversal(const Graph& graph, int startId) {
        Stack<int> frontier;
        Stack<int> visitOrder;
        HashSet    visited;

        if (!graph.nodeExists(startId)) return visitOrder;

        frontier.push(startId);

        while (!frontier.isEmpty()) {
            int current = frontier.pop();
            if (visited.contains(current)) continue;
            visited.insert(current);
            visitOrder.push(current);

            for (const Edge& edge : graph.getNeighbors(current))
                if (!visited.contains(edge.targetId))
                    frontier.push(edge.targetId);
        }

        return visitOrder;
    }

    // ── Connectivity check ────────────────────────────────────
    static bool isReachable(const Graph& graph, int startId, int goalId) {
        if (!graph.nodeExists(startId) || !graph.nodeExists(goalId))
            return false;
        Result r = search(graph, startId, goalId);
        return r.found;
    }
};
