#pragma once
#include "Graph/GraphNode.h"
#include "Data Structures\DataStructures.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
// ============================================================
//  Graph.h  —  Weighted Directed Graph (Adjacency List)
//  UCP BSCS — Data Structures Project
//
//  Represents the navigable world as a graph:
//    - Each walkable grid cell = 1 GraphNode
//    - Each edge = directional connection to a neighbor
//    - 8-directional movement (cardinal + diagonal)
//    - Obstacle cells are excluded; no node created for them
//
//  Node IDs:  row * cols + col   (row-major)
//  Max nodes: MAX_NODES (default 4096 = 64×64 grid)
//
//  No STL. All storage via raw arrays + custom LinkedList.
// ============================================================

static const int MAX_NODES = 4096;    // 64×64 grid max

class Graph {
private:
    GraphNode nodes[MAX_NODES];   // flat node array indexed by id
    bool      exists[MAX_NODES];  // true if node with that id exists
    int       nodeCount;
    int       gridRows;
    int       gridCols;
    float     cellSize;           // pixel size of one grid cell

    // ── Euclidean distance between two node centres ───────────
    float distance(int idA, int idB) const {
        const GraphNode& a = nodes[idA];
        const GraphNode& b = nodes[idB];
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // ── Convert (row, col) → flat id ─────────────────────────
    int toId(int row, int col) const {
        return row * gridCols + col;
    }

    bool inBounds(int row, int col) const {
        return row >= 0 && row < gridRows &&
               col >= 0 && col < gridCols;
    }

public:
    // ── Constructor ───────────────────────────────────────────
    Graph() : nodeCount(0), gridRows(0), gridCols(0), cellSize(32.0f) {
        for (int i = 0; i < MAX_NODES; i++) exists[i] = false;
    }

    // ── Manual node / edge management ────────────────────────
    void addNode(int id, int col, int row, float x, float y) {
        if (id < 0 || id >= MAX_NODES)
            throw std::out_of_range("Graph::addNode — id out of range");
        nodes[id]  = GraphNode(id, col, row, x, y, true);
        exists[id] = true;
        nodeCount++;
    }

    // Directed edge: srcId → dstId with Euclidean weight
    void addEdge(int srcId, int dstId) {
        if (!exists[srcId] || !exists[dstId]) return;
        float w = distance(srcId, dstId);
        nodes[srcId].addEdge(dstId, w);
    }

    // Add both directions (undirected pair)
    void addEdgeBidirectional(int idA, int idB) {
        addEdge(idA, idB);
        addEdge(idB, idA);
    }

    void removeEdge(int srcId, int dstId) {
        if (!exists[srcId]) return;
        nodes[srcId].removeEdge(dstId);
    }

    // Remove all edges to/from a node (make it an obstacle)
    void isolateNode(int id) {
        if (!exists[id]) return;
        nodes[id].neighbors.clear();
        // Remove incoming edges from all neighbors
        for (int i = 0; i < MAX_NODES; i++)
            if (exists[i]) nodes[i].removeEdge(id);
    }

    // Re-connect a previously isolated node to its walkable neighbors
    void restoreNode(int id) {
        if (!exists[id]) return;
        GraphNode& n = nodes[id];
        int r = n.row, c = n.col;

        const int dr[] = {-1,-1,-1, 0, 0, 1, 1, 1};
        const int dc[] = {-1, 0, 1,-1, 1,-1, 0, 1};

        for (int d = 0; d < 8; d++) {
            int nr = r + dr[d], nc = c + dc[d];
            if (!inBounds(nr, nc)) continue;
            int nid = toId(nr, nc);
            if (!exists[nid]) continue;
            addEdgeBidirectional(id, nid);
        }
    }

    // ── Getters ───────────────────────────────────────────────
    const GraphNode& getNode(int id) const {
        if (id < 0 || id >= MAX_NODES || !exists[id])
            throw std::out_of_range("Graph::getNode — invalid id");
        return nodes[id];
    }

    GraphNode& getNodeMut(int id) {
        if (id < 0 || id >= MAX_NODES || !exists[id])
            throw std::out_of_range("Graph::getNodeMut — invalid id");
        return nodes[id];
    }

    // Returns iterator over neighbors (const LinkedList reference)
    const LinkedList<Edge>& getNeighbors(int id) const {
        if (!exists[id])
            throw std::out_of_range("Graph::getNeighbors — invalid id");
        return nodes[id].neighbors;
    }

    bool nodeExists(int id) const {
        return (id >= 0 && id < MAX_NODES && exists[id]);
    }

    int  getNodeCount() const { return nodeCount; }
    int  getGridRows()  const { return gridRows;  }
    int  getGridCols()  const { return gridCols;  }
    float getCellSize() const { return cellSize;  }

    // (row,col) → id — returns -1 if out of bounds or no node there
    int getId(int row, int col) const {
        if (!inBounds(row, col)) return -1;
        int id = toId(row, col);
        return exists[id] ? id : -1;
    }

    // ── Grid Loader ───────────────────────────────────────────
    // File format (text):
    //   Row 0:  ROWS COLS CELL_SIZE
    //   Rows 1…ROWS+1: space-separated 0/1 per cell
    //     0 = walkable, 1 = obstacle
    //
    // Example 5×5 map file:
    //   5 5 32
    //   0 0 0 0 0
    //   0 1 1 1 0
    //   0 0 0 1 0
    //   0 1 0 0 0
    //   0 0 0 0 0
    void loadFromFile(const char* filepath) {
        std::ifstream file(filepath);
        if (!file.is_open())
            throw std::runtime_error("Graph::loadFromFile — cannot open file");

        file >> gridRows >> gridCols >> cellSize;
        if (gridRows * gridCols > MAX_NODES)
            throw std::runtime_error("Graph::loadFromFile — grid too large for MAX_NODES");

        // Reset
        for (int i = 0; i < MAX_NODES; i++) {
            exists[i] = false;
            nodes[i]  = GraphNode();
        }
        nodeCount = 0;

        // Pass 1: create nodes for walkable cells
        for (int r = 0; r < gridRows; r++) {
            for (int c = 0; c < gridCols; c++) {
                int tile;
                file >> tile;
                if (tile == 0) {  // 0 = walkable
                    int id     = toId(r, c);
                    float wx   = c * cellSize + cellSize * 0.5f;
                    float wy   = r * cellSize + cellSize * 0.5f;
                    addNode(id, c, r, wx, wy);
                }
            }
        }

        // Pass 2: connect 8-directional neighbors
        const int dr[] = {-1,-1,-1, 0, 0, 1, 1, 1};
        const int dc[] = {-1, 0, 1,-1, 1,-1, 0, 1};

        for (int r = 0; r < gridRows; r++) {
            for (int c = 0; c < gridCols; c++) {
                int id = toId(r, c);
                if (!exists[id]) continue;

                for (int d = 0; d < 8; d++) {
                    int nr = r + dr[d], nc = c + dc[d];
                    if (!inBounds(nr, nc)) continue;
                    int nid = toId(nr, nc);
                    if (!exists[nid]) continue;

                    // Block diagonal cuts through obstacle corners
                    // e.g. moving NE: both N and E must be walkable
                    if (dr[d] != 0 && dc[d] != 0) {
                        int northId = toId(r + dr[d], c);
                        int eastId  = toId(r, c + dc[d]);
                        if (!exists[northId] || !exists[eastId]) continue;
                    }

                    addEdge(id, nid);
                }
            }
        }

        file.close();
    }

    // ── Inline grid loader (from 2D int array) ────────────────
    // Useful for in-code test grids without needing a file
    // grid[r][c] = 0 (walkable) or 1 (obstacle)
    void loadFromArray(const int* grid, int rows, int cols, float cell = 32.0f) {
        gridRows = rows;
        gridCols = cols;
        cellSize = cell;

        for (int i = 0; i < MAX_NODES; i++) {
            exists[i] = false;
            nodes[i]  = GraphNode();
        }
        nodeCount = 0;

        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                if (grid[r * cols + c] == 0) {
                    int   id = toId(r, c);
                    float wx = c * cell + cell * 0.5f;
                    float wy = r * cell + cell * 0.5f;
                    addNode(id, c, r, wx, wy);
                }

        const int dr[] = {-1,-1,-1, 0, 0, 1, 1, 1};
        const int dc[] = {-1, 0, 1,-1, 1,-1, 0, 1};

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int id = toId(r, c);
                if (!exists[id]) continue;
                for (int d = 0; d < 8; d++) {
                    int nr = r + dr[d], nc = c + dc[d];
                    if (!inBounds(nr, nc)) continue;
                    int nid = toId(nr, nc);
                    if (!exists[nid]) continue;
                    if (dr[d] != 0 && dc[d] != 0) {
                        if (!exists[toId(r + dr[d], c)]) continue;
                        if (!exists[toId(r, c + dc[d])]) continue;
                    }
                    addEdge(id, nid);
                }
            }
        }
    }

    // ── Debug helpers ─────────────────────────────────────────
    void printAdjacencyList() const {
        for (int i = 0; i < MAX_NODES; i++) {
            if (!exists[i]) continue;
            printf("Node %d (r%d,c%d) -> ", i, nodes[i].row, nodes[i].col);
            for (auto& e : nodes[i].neighbors)
                printf("[%d w=%.2f] ", e.targetId, e.weight);
            printf("\n");
        }
    }

    int countEdges() const {
        int total = 0;
        for (int i = 0; i < MAX_NODES; i++)
            if (exists[i]) total += nodes[i].neighbors.getSize();
        return total;
    }
};
