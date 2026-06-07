#pragma once
#include "NPC.h"
#include "Data Structures/DataStructures.h"
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

    // ── Compute and apply separation — obstacle aware ────────
    void applySeparationForces(const Graph& graph) {
        float cell = graph.getCellSize();

        for (int i = 0; i < npcCount; i++) {
            NPC* a = npcs[i];

            // Accumulate repulsion from nearby NPCs
            float repX = 0.0f, repY = 0.0f;

            LinkedList<AgentPoint> nearby = quadtree->query(
                a->x, a->y,
                SEPARATION_RADIUS, SEPARATION_RADIUS
            );

            for (auto& pt : nearby) {
                if (pt.agentId == a->id) continue;
                float dx   = a->x - pt.px;
                float dy   = a->y - pt.py;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist < SEPARATION_RADIUS && dist > 0.001f) {
                    float force = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
                    repX += (dx / dist) * force * 1.2f;
                    repY += (dy / dist) * force * 1.2f;
                }
            }

            // No repulsion needed
            if (std::fabs(repX) < 0.001f && std::fabs(repY) < 0.001f)
                continue;

            // Normalise and scale repulsion
            float rLen = std::sqrt(repX*repX + repY*repY);
            float moveAmt = 1.8f;   // pixels to push per frame
            float mvX = (repX / rLen) * moveAmt;
            float mvY = (repY / rLen) * moveAmt;

            float nx = a->x + mvX;
            float ny = a->y + mvY;

            // Hard world boundary clamp
            float maxX = graph.getGridCols() * cell - cell * 0.5f;
            float maxY = graph.getGridRows() * cell - cell * 0.5f;
            nx = nx < cell*0.5f ? cell*0.5f : (nx > maxX ? maxX : nx);
            ny = ny < cell*0.5f ? cell*0.5f : (ny > maxY ? maxY : ny);

            // Check destination cell walkable
            int nc = (int)(nx / cell);
            int nr = (int)(ny / cell);

            if (graph.nodeExists(graph.getId(nr, nc))) {
                // Fully walkable — apply
                a->x = nx;
                a->y = ny;
            } else {
                // Try X only
                float tx = a->x + mvX;
                tx = tx < cell*0.5f ? cell*0.5f : (tx > maxX ? maxX : tx);
                int txc = (int)(tx / cell);
                int txr = (int)(a->y / cell);
                if (graph.nodeExists(graph.getId(txr, txc))) {
                    a->x = tx;
                } else {
                    // Try Y only
                    float ty = a->y + mvY;
                    ty = ty < cell*0.5f ? cell*0.5f : (ty > maxY ? maxY : ty);
                    int tyc = (int)(a->x / cell);
                    int tyr = (int)(ty / cell);
                    if (graph.nodeExists(graph.getId(tyr, tyc))) {
                        a->y = ty;
                    }
                    // Both blocked — NPC stays, wall wins
                }
            }
        }
    }

    // ── Safety snap — fix any NPC that ended up inside obstacle ──
    void safetySnap(const Graph& graph) {
        float cell = graph.getCellSize();
        for (int i = 0; i < npcCount; i++) {
            NPC* a = npcs[i];
            int cc = (int)(a->x / cell);
            int cr = (int)(a->y / cell);
            if (graph.nodeExists(graph.getId(cr, cc))) continue;

            // Find nearest walkable cell within 3-cell radius
            float bestDist = 1e9f;
            float bestX = a->x, bestY = a->y;
            for (int dr = -3; dr <= 3; dr++) {
                for (int dc = -3; dc <= 3; dc++) {
                    int tr = cr+dr, tc = cc+dc;
                    if (tr<0||tr>=graph.getGridRows()) continue;
                    if (tc<0||tc>=graph.getGridCols()) continue;
                    if (!graph.nodeExists(graph.getId(tr,tc))) continue;
                    float wx = (tc+0.5f)*cell;
                    float wy = (tr+0.5f)*cell;
                    float d  = (wx-a->x)*(wx-a->x)+(wy-a->y)*(wy-a->y);
                    if (d < bestDist) { bestDist=d; bestX=wx; bestY=wy; }
                }
            }
            a->x = bestX;
            a->y = bestY;
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

        // 2. Move each NPC along its path
        for (int i = 0; i < npcCount; i++)
            npcs[i]->update(dt, graph);

        // 3. Apply separation AFTER path movement, obstacle-aware
        applySeparationForces(graph);

        // 4. Safety snap — fix any NPC stuck in obstacle
        safetySnap(graph);
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
