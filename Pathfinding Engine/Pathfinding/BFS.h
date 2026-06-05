#pragma once
#include "Graph/Graph.h"
#include "C:\VISUAL STUDIO PROJECTS\Pathfinding Engine\Pathfinding Engine\Data Structures\DataStructures.h"
// ============================================================
//  BFS.h  —  Breadth-First Search
//  UCP BSCS — Data Structures Project
//
//  Uses: custom Queue (FIFO) for level-order expansion
//        HashSet for O(1) visited tracking
//        HashMap for parent reconstruction
//
//  Two modes:
//    1. search(start, goal)   → returns path Stack if reachable
//    2. floodFill(start)      → returns HashSet of all reachable IDs
//       (used to verify whether A* could possibly find a path)
// ============================================================

class BFS {
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

    // ── BFS point-to-point search ─────────────────────────────
    static Result search(const Graph& graph, int startId, int goalId) {
        Result result;

        if (!graph.nodeExists(startId) || !graph.nodeExists(goalId))
            return result;

        if (startId == goalId) {
            result.path.push(startId);
            result.found = true;
            return result;
        }

        Queue<int> frontier;
        HashSet    visited;
        HashMap    cameFrom;

        frontier.enqueue(startId);
        visited.insert(startId);

        while (!frontier.isEmpty()) {
            int current = frontier.dequeue();
            result.nodesVisited++;

            for (const Edge& edge : graph.getNeighbors(current)) {
                int next = edge.targetId;
                if (visited.contains(next)) continue;

                visited.insert(next);
                cameFrom.insert(next, current);

                if (next == goalId) {
                    result.path  = reconstructPath(cameFrom, startId, goalId);
                    result.found = true;
                    result.nodesVisited++;
                    return result;
                }

                frontier.enqueue(next);
            }
        }

        return result;  // goal not reachable
    }

    // ── Flood fill: find all nodes reachable from start ───────
    // Returns count of reachable nodes. Fills reachable HashSet.
    static int floodFill(const Graph& graph, int startId, HashSet& reachable) {
        if (!graph.nodeExists(startId)) return 0;

        Queue<int> frontier;
        frontier.enqueue(startId);
        reachable.insert(startId);

        int count = 0;
        while (!frontier.isEmpty()) {
            int current = frontier.dequeue();
            count++;
            for (const Edge& edge : graph.getNeighbors(current)) {
                int next = edge.targetId;
                if (!reachable.contains(next)) {
                    reachable.insert(next);
                    frontier.enqueue(next);
                }
            }
        }
        return count;
    }

    // ── Reachability check (convenience wrapper) ─────────────
    static bool isReachable(const Graph& graph, int startId, int goalId) {
        if (!graph.nodeExists(startId) || !graph.nodeExists(goalId))
            return false;
        HashSet reachable;
        floodFill(graph, startId, reachable);
        return reachable.contains(goalId);
    }
};
