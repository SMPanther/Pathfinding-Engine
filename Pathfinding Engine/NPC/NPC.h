#pragma once
#include "Graph/Graph.h"
#include "C:\VISUAL STUDIO PROJECTS\Pathfinding Engine\Pathfinding Engine\Data Structures\DataStructures.h"
#include "Pathfinding/Pathfinding.h"
#include <cmath>
// ============================================================
//  NPC.h  —  Individual NPC Agent
//  UCP BSCS — Data Structures Project
//
//  Each NPC:
//    - Has a world position (x, y) and a movement speed
//    - Holds its remaining path as a Stack<int> of node IDs
//    - Pops nodes off the Stack one-by-one each update tick
//    - Requests a new A* path when its goal changes
//    - Has an optional circular patrol route (CircularLinkedList)
//    - Applies a simple separation force when too close to others
//
//  Path Stack convention (from AStar/BFS/DFS):
//    top    = GOAL  (first pop gives the final destination)
//    bottom = START
//    NPC reverses this on receive so top = next waypoint
// ============================================================

enum class NPCState {
    IDLE,       // standing still, no goal
    MOVING,     // walking along path
    PATROLLING, // following circular waypoint route
    ARRIVED     // just reached goal, waiting for new assignment
};

class NPC {
public:
    // ── Identity ──────────────────────────────────────────────
    int   id;
    float x, y;          // world-space position (pixels)
    float speed;         // pixels per second
    int   goalNodeId;    // current navigation target (-1 = none)
    bool  hasPath;
    NPCState state;

    // ── Color (for SFML rendering later) ─────────────────────
    unsigned char colorR, colorG, colorB;

    // ── Path data ─────────────────────────────────────────────
    Stack<int> path;           // remaining waypoints — top = next node

    // ── Patrol route ─────────────────────────────────────────
    CircularLinkedList<int> patrolRoute;   // cyclic waypoint node IDs
    bool patrolMode;

    // ── Separation (set by NPCManager each tick) ─────────────
    float sepX, sepY;   // accumulated separation force this frame

    // ── Constructor ───────────────────────────────────────────
    NPC()
        : id(-1), x(0), y(0), speed(80.0f), goalNodeId(-1),
          hasPath(false), state(NPCState::IDLE),
          colorR(255), colorG(255), colorB(255),
          patrolMode(false), sepX(0), sepY(0) {}

    NPC(int id, float startX, float startY, float speed = 80.0f)
        : id(id), x(startX), y(startY), speed(speed),
          goalNodeId(-1), hasPath(false), state(NPCState::IDLE),
          colorR(255), colorG(200), colorB(100),
          patrolMode(false), sepX(0), sepY(0) {}

    // ── Assign a new goal and request a path ─────────────────
    // Returns true if a path was found
    // Original setGoal — always uses A* (kept for patrol)
    bool setGoal(int nodeId, const Graph& graph) {
        if (nodeId == goalNodeId && hasPath) return true;
        goalNodeId = nodeId;
        path.clear();
        hasPath    = false;
        patrolMode = false;

        int startNodeId = nearestNode(graph);
        if (startNodeId == -1) return false;

        AStar::Result result = AStar::search(graph, startNodeId, goalNodeId);
        if (!result.found) return false;

        reverseIntoPath(result.path);
        hasPath = true;
        state   = NPCState::MOVING;
        return true;
    }

    // NEW: setGoalWithPath — accepts a pre-computed path from any algorithm
    // Call this from main.cpp so BFS/DFS paths are actually followed
    bool setGoalWithPath(int nodeId, Stack<int>& precomputedPath) {
        goalNodeId = nodeId;
        path.clear();
        hasPath    = false;
        patrolMode = false;

        if (precomputedPath.isEmpty()) return false;

        reverseIntoPath(precomputedPath);
        hasPath = true;
        state   = NPCState::MOVING;
        return true;
    }

    // ── Set a patrol route (cyclic waypoints) ─────────────────
    void setPatrol(const Graph& graph) {
        if (patrolRoute.isEmpty()) return;
        // setGoal resets patrolMode, so set it AFTER
        setGoal(patrolRoute.front(), graph);
        patrolMode = true;
        state      = NPCState::PATROLLING;
    }

    // ── Per-frame update ──────────────────────────────────────
    // dt = delta time in seconds
    // graph used to look up node positions for next waypoint
    void update(float dt, const Graph& graph) {
        if (state == NPCState::IDLE || state == NPCState::ARRIVED) {
            applySeparation(dt);
            return;
        }

        if (!hasPath || path.isEmpty()) {
            if (patrolMode) advancePatrol(graph);
            else            state = NPCState::ARRIVED;
            return;
        }

        // Peek at next node (top of stack)
        int nextNodeId = path.peek();
        const GraphNode& target = graph.getNode(nextNodeId);

        float dx = target.x - x;
        float dy = target.y - y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Apply separation force
        dx += sepX * 0.3f;
        dy += sepY * 0.3f;

        // Arrived at waypoint threshold
        const float ARRIVAL_THRESHOLD = 4.0f;
        if (dist < ARRIVAL_THRESHOLD) {
            x = target.x;
            y = target.y;
            path.pop();

            if (path.isEmpty()) {
                if (patrolMode) advancePatrol(graph);
                else {
                    state   = NPCState::ARRIVED;
                    hasPath = false;
                }
            }
            return;
        }

        // Normalize direction and move
        float len = std::sqrt(dx * dx + dy * dy);
        if (len > 0.0001f) {
            x += (dx / len) * speed * dt;
            y += (dy / len) * speed * dt;
        }

        sepX = sepY = 0.0f;   // reset separation each frame
    }

    // ── Force immediate position (teleport) ───────────────────
    void teleport(float nx, float ny) {
        x = nx; y = ny;
        path.clear();
        hasPath = false;
        state   = NPCState::IDLE;
    }

    // ── Utility ───────────────────────────────────────────────
    bool isMoving()  const { return state == NPCState::MOVING || state == NPCState::PATROLLING; }
    bool isArrived() const { return state == NPCState::ARRIVED; }

private:
    // Reverse incoming path (top=goal → top=next waypoint)
    void reverseIntoPath(Stack<int>& incoming) {
        Stack<int> temp;
        while (!incoming.isEmpty())
            temp.push(incoming.pop());   // temp: top=start, bottom=goal
        path = temp;                     // path: top=start... we want top=next
        // Actually we want top = first step after start.
        // temp top = start, so pop start, then path top = first real waypoint.
        // But we want the FULL path including start for rendering.
        // Convention: path top = start (pop it immediately on first update).
        // Already correct — path top = start, next pop = second node, etc.
    }

    // Find the graph node closest to current (x,y) position
    int nearestNode(const Graph& graph) const {
        int   bestId   = -1;
        float bestDist = 1e9f;

        int rows = graph.getGridRows();
        int cols = graph.getGridCols();

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int id = graph.getId(r, c);
                if (id == -1) continue;
                const GraphNode& n = graph.getNode(id);
                float dx = n.x - x;
                float dy = n.y - y;
                float d  = dx*dx + dy*dy;
                if (d < bestDist) {
                    bestDist = d;
                    bestId   = id;
                }
            }
        }
        return bestId;
    }

    void advancePatrol(const Graph& graph) {
        patrolRoute.advanceHead();
        setGoal(patrolRoute.front(), graph);
    }

    void applySeparation(float dt) {
        float len = std::sqrt(sepX*sepX + sepY*sepY);
        if (len > 0.0001f) {
            x += (sepX / len) * speed * 0.5f * dt;
            y += (sepY / len) * speed * 0.5f * dt;
        }
        sepX = sepY = 0.0f;
    }
};
