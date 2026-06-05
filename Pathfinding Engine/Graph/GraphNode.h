#pragma once
#include "Graph/Edge.h"
#include "Data Structures\DataStructures.h"
// ============================================================
//  GraphNode.h  —  Single Node in the Navigation Graph
//  UCP BSCS — Data Structures Project
//
//  Each walkable grid cell becomes one GraphNode.
//  Non-walkable (obstacle) cells have no node created.
//
//  id    = unique sequential index (row * gridCols + col)
//  x, y  = world-space pixel centre of the cell
//  col, row = grid coordinates (for grid ↔ id conversion)
//  neighbors = custom LinkedList of outgoing Edge objects
// ============================================================

class GraphNode {
public:
    int   id;
    int   col, row;        // grid coordinates
    float x,  y;           // world-space centre (pixels)
    bool  walkable;

    LinkedList<Edge> neighbors;   // outgoing edges (adjacency list)

    // ── Constructors ─────────────────────────────────────────
    GraphNode()
        : id(-1), col(0), row(0), x(0.0f), y(0.0f), walkable(true) {}

    GraphNode(int id, int col, int row, float x, float y, bool walkable = true)
        : id(id), col(col), row(row), x(x), y(y), walkable(walkable) {}

    // ── Edge Management ──────────────────────────────────────
    void addEdge(int targetId, float weight) {
        // Avoid duplicates
        for (auto& e : neighbors)
            if (e.targetId == targetId) return;
        neighbors.insertBack(Edge(targetId, weight));
    }

    void removeEdge(int targetId) {
        neighbors.remove(Edge(targetId, 0.0f));
    }

    bool hasEdgeTo(int targetId) const {
        for (auto& e : neighbors)
            if (e.targetId == targetId) return true;
        return false;
    }
};
