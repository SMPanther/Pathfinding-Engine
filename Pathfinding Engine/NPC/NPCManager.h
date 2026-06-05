#pragma once
#include "NPC.h"
#include "C:\VISUAL STUDIO PROJECTS\Pathfinding Engine\Pathfinding Engine\Data Structures\DataStructures.h"
#include "Graph/Graph.h"
#include <cmath>
// ============================================================
//  NPCManager.h  —  Manages All NPC Agents
//  UCP BSCS — Data Structures Project
//
//  Responsibilities:
//    1. Owns a LinkedList<NPC*> of all active agents
//    2. Rebuilds the Quadtree every tick with current positions
//    3. Queries Quadtree for each NPC to find nearby agents
//    4. Applies separation forces to prevent overlap
//    5. Ticks each NPC's update()
//    6. Provides spawn / remove / goal-assignment interface
//
//  Separation model:
//    If two NPCs are within SEPARATION_RADIUS pixels,
//    a repulsion vector pushes them apart proportional
//    to how close they are. Applied as sepX/sepY on NPC.
// ============================================================

static const float SEPARATION_RADIUS = 28.0f;   // pixels
static const float NPC_RADIUS        = 10.0f;   // visual/collision radius
static const int   MAX_NPCS          = 64;

class NPCManager {
private:
    NPC*             npcs[MAX_NPCS];   // raw pointer array (no STL)
    int              npcCount;
    int              nextId;

    Quadtree*        quadtree;
    float            worldW, worldH;

    // ── Rebuild quadtree with all current NPC positions ──────
    void rebuildQuadtree() {
        quadtree->clear();
        for (int i = 0; i < npcCount; i++)
            quadtree->insert(npcs[i]->x, npcs[i]->y, npcs[i]->id);
    }

    // ── Compute and apply separation forces ──────────────────
    void applySeparationForces() {
        for (int i = 0; i < npcCount; i++) {
            NPC* a = npcs[i];

            // Query quadtree for neighbors within separation radius
            LinkedList<AgentPoint> nearby = quadtree->query(
                a->x, a->y,
                SEPARATION_RADIUS, SEPARATION_RADIUS
            );

            for (auto& pt : nearby) {
                if (pt.agentId == a->id) continue;   // skip self

                float dx   = a->x - pt.px;
                float dy   = a->y - pt.py;
                float dist = std::sqrt(dx * dx + dy * dy);

                if (dist < SEPARATION_RADIUS && dist > 0.001f) {
                    // Force magnitude: stronger when closer
                    float force = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
                    a->sepX += (dx / dist) * force;
                    a->sepY += (dy / dist) * force;
                }
            }
        }
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    NPCManager(float worldWidth, float worldHeight)
        : npcCount(0), nextId(0), worldW(worldWidth), worldH(worldHeight) {
        for (int i = 0; i < MAX_NPCS; i++) npcs[i] = nullptr;
        quadtree = new Quadtree(worldW, worldH);
    }

    ~NPCManager() {
        for (int i = 0; i < npcCount; i++) {
            delete npcs[i];
            npcs[i] = nullptr;
        }
        delete quadtree;
    }

    // ── Spawn a new NPC at world position ─────────────────────
    // Returns pointer to the new NPC (caller does NOT own it)
    NPC* spawnNPC(float x, float y, float speed = 80.0f) {
        if (npcCount >= MAX_NPCS) return nullptr;
        NPC* npc     = new NPC(nextId++, x, y, speed);
        npcs[npcCount++] = npc;

        // Give each NPC a unique color (simple palette cycle)
        static const unsigned char PALETTE[][3] = {
            {255, 100, 100}, {100, 200, 255}, {100, 255, 150},
            {255, 220,  80}, {220, 100, 255}, {255, 160,  60},
            { 80, 220, 220}, {255, 255, 100}, {180, 255, 100},
            {255, 100, 200}
        };
        int ci = npc->id % 10;
        npc->colorR = PALETTE[ci][0];
        npc->colorG = PALETTE[ci][1];
        npc->colorB = PALETTE[ci][2];

        return npc;
    }

    // ── Remove NPC by id ──────────────────────────────────────
    bool removeNPC(int id) {
        for (int i = 0; i < npcCount; i++) {
            if (npcs[i]->id == id) {
                delete npcs[i];
                // Shift array left
                for (int j = i; j < npcCount - 1; j++)
                    npcs[j] = npcs[j + 1];
                npcs[--npcCount] = nullptr;
                return true;
            }
        }
        return false;
    }

    // ── Assign goal to a specific NPC ─────────────────────────
    bool setGoal(int npcId, int nodeId, const Graph& graph) {
        NPC* npc = getNPC(npcId);
        if (!npc) return false;
        return npc->setGoal(nodeId, graph);
    }

    // ── Assign same goal to ALL NPCs ─────────────────────────
    void setGoalAll(int nodeId, const Graph& graph) {
        for (int i = 0; i < npcCount; i++)
            npcs[i]->setGoal(nodeId, graph);
    }

    // ── Assign random goals from walkable node list ───────────
    // Picks a random walkable node for each NPC
    void assignRandomGoals(const Graph& graph, unsigned int seed = 42) {
        // Collect all valid node IDs into a temp array
        int nodeIds[MAX_NODES];
        int nodeCount = 0;
        for (int r = 0; r < graph.getGridRows(); r++)
            for (int c = 0; c < graph.getGridCols(); c++) {
                int id = graph.getId(r, c);
                if (id != -1) nodeIds[nodeCount++] = id;
            }
        if (nodeCount == 0) return;

        // Simple LCG random
        unsigned int rng = seed;
        for (int i = 0; i < npcCount; i++) {
            rng = rng * 1664525u + 1013904223u;
            int idx = (int)((rng >> 16) % nodeCount);
            npcs[i]->setGoal(nodeIds[idx], graph);
        }
    }

    // ── Main update tick ──────────────────────────────────────
    // Call once per simulation frame with delta time
    void update(float dt, const Graph& graph) {
        // 1. Rebuild spatial index
        rebuildQuadtree();

        // 2. Compute separation forces using quadtree
        applySeparationForces();

        // 3. Update each NPC's position along its path
        for (int i = 0; i < npcCount; i++)
            npcs[i]->update(dt, graph);
    }

    // ── Getters ───────────────────────────────────────────────
    NPC* getNPC(int id) const {
        for (int i = 0; i < npcCount; i++)
            if (npcs[i]->id == id) return npcs[i];
        return nullptr;
    }

    NPC* getNPCByIndex(int index) const {
        if (index < 0 || index >= npcCount) return nullptr;
        return npcs[index];
    }

    int  getNPCCount()   const { return npcCount; }
    bool isEmpty()       const { return npcCount == 0; }

    const Quadtree* getQuadtree() const { return quadtree; }

    // ── Stats ─────────────────────────────────────────────────
    int countMoving() const {
        int n = 0;
        for (int i = 0; i < npcCount; i++)
            if (npcs[i]->isMoving()) n++;
        return n;
    }

    int countArrived() const {
        int n = 0;
        for (int i = 0; i < npcCount; i++)
            if (npcs[i]->isArrived()) n++;
        return n;
    }
};
