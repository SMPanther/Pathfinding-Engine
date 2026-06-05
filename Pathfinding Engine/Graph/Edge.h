#pragma once
// ============================================================
//  Edge.h  —  Weighted Directed Graph Edge
//  UCP BSCS — Data Structures Project
//
//  Each Edge represents a one-way connection:
//    sourceNode → targetId  with a given weight
//
//  Stored inside LinkedList<Edge> on each GraphNode.
//  Weight = Euclidean distance between cells (1.0 cardinal,
//  √2 ≈ 1.414 diagonal).
// ============================================================

struct Edge {
    int   targetId;   // destination GraphNode id
    float weight;     // travel cost (distance)

    Edge() : targetId(-1), weight(0.0f) {}
    Edge(int target, float w) : targetId(target), weight(w) {}

    bool operator==(const Edge& o) const {
        return targetId == o.targetId;
    }
};
