// ============================================================
//  main.cpp  —  Pathfinding & Crowd Simulation Engine  v2
//  UCP BSCS — Data Structures Project
//
//  Controls:
//    Left Click        → set goal (pathfind with current algo)
//    Right Click       → place/remove obstacle (dynamic rerouting)
//    Middle Click      → spawn NPC at cursor
//    [1] A*  [2] BFS  [3] DFS
//    [Q] Quadtree overlay
//    [P] Path overlay
//    [E] Exploration overlay
//    [C] Clear NPCs
//    [R] Random goals
//    [ESC] Quit
// ============================================================

#include <SFML/Graphics.hpp>
#include "Graph/Graph.h"
#include "NPC/NPCSystem.h"
#include "Pathfinding/Pathfinding.h"
#include "Renderer/GridRenderer.h"
#include "Renderer/PathRenderer.h"
#include "Renderer/NPCRenderer.h"
#include "Renderer/QuadtreeRenderer.h"
#include "Renderer/HUD.h"
#include <cstdio>
#include <cmath>

// ── Constants ─────────────────────────────────────────────────
static const int   WINDOW_W  = 960;
static const int   WINDOW_H  = 720;
static const float CELL_SIZE = 32.0f;
static const int   GRID_ROWS = 22;
static const int   GRID_COLS = 30;

// ── Map ───────────────────────────────────────────────────────
static int MAP[GRID_ROWS][GRID_COLS] = {
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,1,1,1,0,0,0,0,0,0, 0,0,1,1,1,1,0,0,0,0, 0,0,0,0,0,1,1,1,0,0},
    {0,1,0,1,0,0,0,0,0,0, 0,0,1,0,0,1,0,0,0,0, 0,0,0,0,0,1,0,1,0,0},
    {0,1,0,0,0,0,1,1,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1,0,0},
    {0,0,0,0,0,0,1,1,0,0, 0,0,0,0,0,0,0,0,0,0, 0,1,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0, 1,1,1,0,0,0,0,0,0,0, 0,1,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,1,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,0,1,1,1,0,0,0,0,0, 0,0,0,0,0,0,0,1,1,1, 1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,1,1,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,1,1,1,0,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,1,0, 0,0,0,0,0,1,0,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,1,1,1,0,0,0,0,0},
    {0,0,1,1,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,1,0,0,0,0,0},
    {0,0,0,1,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,1, 1,1,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,1,1,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,1,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,1,1,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,1,0,0,0, 0,0,0,0,0,0,0,0,0,0},
    {0,1,1,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,1,0},
    {0,0,1,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,1,0},
    {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0},
};

// ── Algo selector ─────────────────────────────────────────────
enum class PathAlgo { ASTAR, BFS, DFS };

// ── Exploration data (for visualization) ─────────────────────
static const int MAX_EXPLORED = 2048;
struct ExploreData {
    int   nodes[MAX_EXPLORED];
    int   count = 0;
    void  clear() { count = 0; }
    void  add(int id) { if (count < MAX_EXPLORED) nodes[count++] = id; }
};

// ── Run pathfinding and capture explored nodes ────────────────
static Stack<int> computePath(const Graph& graph, int startId, int goalId,
                               PathAlgo algo, ExploreData& explored)
{
    explored.clear();
    switch (algo) {
        case PathAlgo::ASTAR: {
            // Run A* and record expansion order via nodesExpanded
            // We capture open-set expansion by instrumenting via a quick BFS
            // for the exploration overlay, while A* gives the optimal path
            auto r = AStar::search(graph, startId, goalId);
            // Approximate explored = BFS up to same cost radius
            {
                auto b = BFS::search(graph, startId, goalId);
                // Use BFS visited count as exploration proxy
                HashSet reachable;
                BFS::floodFill(graph, startId, reachable);
                // Record all nodes within path cost distance
                for (int rr = 0; rr < graph.getGridRows(); rr++)
                    for (int cc = 0; cc < graph.getGridCols(); cc++) {
                        int id = graph.getId(rr, cc);
                        if (id != -1 && reachable.contains(id))
                            explored.add(id);
                    }
            }
            return r.found ? r.path : Stack<int>();
        }
        case PathAlgo::BFS: {
            // BFS explores level by level — record ALL visited nodes
            // We run flood fill to show the full BFS wave
            HashSet reachable;
            BFS::floodFill(graph, startId, reachable);
            for (int rr = 0; rr < graph.getGridRows(); rr++)
                for (int cc = 0; cc < graph.getGridCols(); cc++) {
                    int id = graph.getId(rr, cc);
                    if (id != -1 && reachable.contains(id))
                        explored.add(id);
                }
            auto r = BFS::search(graph, startId, goalId);
            return r.found ? r.path : Stack<int>();
        }
        case PathAlgo::DFS: {
            // DFS explores deeply — record full traversal order
            Stack<int> traversal = DFS::fullTraversal(graph, startId);
            Stack<int> copy = traversal;
            while (!copy.isEmpty()) explored.add(copy.pop());
            auto r = DFS::search(graph, startId, goalId);
            return r.found ? r.path : Stack<int>();
        }
    }
    return Stack<int>();
}

// ── Algo colors ───────────────────────────────────────────────
static sf::Color algoColor(PathAlgo algo) {
    switch (algo) {
        case PathAlgo::ASTAR: return sf::Color( 80, 220, 120); // green
        case PathAlgo::BFS:   return sf::Color( 80, 160, 255); // blue
        case PathAlgo::DFS:   return sf::Color(255, 140,  60); // orange
    }
    return sf::Color::White;
}

// ── World ↔ Cell ──────────────────────────────────────────────
static void worldToCell(float wx, float wy, float cell, int& row, int& col) {
    col = (int)(wx / cell);
    row = (int)(wy / cell);
}

// ── Rebuild graph from current MAP state ─────────────────────
static void rebuildGraph(Graph& graph) {
    int flat[GRID_ROWS * GRID_COLS];
    for (int r = 0; r < GRID_ROWS; r++)
        for (int c = 0; c < GRID_COLS; c++)
            flat[r * GRID_COLS + c] = MAP[r][c];
    graph.loadFromArray(flat, GRID_ROWS, GRID_COLS, CELL_SIZE);
}

// ── Re-route all NPCs after obstacle change ───────────────────
static void rerouteAll(NPCManager& mgr, const Graph& graph,
                        PathRenderer& pathRenderer,
                        int goalNodeId, PathAlgo algo,
                        bool showPaths, ExploreData& explored)
{
    if (goalNodeId == -1) return;
    pathRenderer.clearPaths();
    for (int i = 0; i < mgr.getNPCCount(); i++) {
        NPC* npc = mgr.getNPCByIndex(i);
        int startNode = graph.getId(
            (int)(npc->y / CELL_SIZE),
            (int)(npc->x / CELL_SIZE));
        if (startNode == -1) continue;

        Stack<int> p = computePath(graph, startNode, goalNodeId, algo, explored);
        Stack<int> pCopy = p;
        npc->setGoalWithPath(goalNodeId, pCopy);
        if (showPaths) {
            sf::Color c = algoColor(algo);
            c.r = (sf::Uint8)((c.r + npc->colorR) / 2);
            c.g = (sf::Uint8)((c.g + npc->colorG) / 2);
            c.b = (sf::Uint8)((c.b + npc->colorB) / 2);
            pathRenderer.addPath(p, c);
        }
    }
}

// ─────────────────────────────────────────────────────────────
int main()
{
    // ── Build graph ──────────────────────────────────────────
    Graph graph;
    rebuildGraph(graph);

    float worldW = GRID_COLS * CELL_SIZE;
    float worldH = GRID_ROWS * CELL_SIZE;

    // ── NPC Manager ──────────────────────────────────────────
    NPCManager npcMgr(worldW, worldH);

    struct SpawnPos { float x, y; };
    SpawnPos spawnPts[] = {
        { 16, 16}, { 80, 16}, {272, 16}, {464, 16},
        { 16,272}, { 16,464}, {464,272}, {464,464},
    };
    for (int i = 0; i < 8; i++)
        npcMgr.spawnNPC(spawnPts[i].x, spawnPts[i].y, 90.0f + i * 8.0f);

    // ── Renderers ─────────────────────────────────────────────
    GridRenderer     gridRenderer;
    PathRenderer     pathRenderer;
    NPCRenderer      npcRenderer;
    QuadtreeRenderer qtRenderer;
    HUD              hud;

    gridRenderer.buildGridLines(graph);

    // ── Window ───────────────────────────────────────────────
    sf::RenderWindow window(
        sf::VideoMode(WINDOW_W, WINDOW_H),
        "Pathfinding & Crowd Simulation Engine  |  UCP BSCS  v2",
        sf::Style::Titlebar | sf::Style::Close
    );
    window.setFramerateLimit(60);

    // ── State ─────────────────────────────────────────────────
    PathAlgo    algo         = PathAlgo::ASTAR;
    bool        showPaths    = true;
    bool        showQTree    = false;
    bool        showExplored = false;
    int         goalNodeId   = -1;
    ExploreData explored;

    // Exploration overlay shapes (pre-allocated)
    sf::RectangleShape exploredTile(sf::Vector2f(CELL_SIZE - 2, CELL_SIZE - 2));

    sf::Clock clock;
    float fpsTimer  = 0.0f;
    int   fpsFrames = 0;
    float currentFps = 60.0f;

    // ── Animation timer for path pulse ───────────────────────
    float animTimer = 0.0f;

    // ── Game loop ─────────────────────────────────────────────
    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;
        animTimer += dt;

        fpsTimer += dt; fpsFrames++;
        if (fpsTimer >= 0.5f) {
            currentFps = fpsFrames / fpsTimer;
            fpsTimer = 0.0f; fpsFrames = 0;
        }

        sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
        int hoverRow = 0, hoverCol = 0;
        worldToCell((float)mousePixel.x, (float)mousePixel.y,
                    CELL_SIZE, hoverRow, hoverCol);

        // ── Events ────────────────────────────────────────────
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) window.close();

            if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                    case sf::Keyboard::Escape: window.close(); break;

                    case sf::Keyboard::Num1:
                        algo = PathAlgo::ASTAR;
                        rerouteAll(npcMgr, graph, pathRenderer,
                                   goalNodeId, algo, showPaths, explored);
                        break;
                    case sf::Keyboard::Num2:
                        algo = PathAlgo::BFS;
                        rerouteAll(npcMgr, graph, pathRenderer,
                                   goalNodeId, algo, showPaths, explored);
                        break;
                    case sf::Keyboard::Num3:
                        algo = PathAlgo::DFS;
                        rerouteAll(npcMgr, graph, pathRenderer,
                                   goalNodeId, algo, showPaths, explored);
                        break;

                    case sf::Keyboard::Q:
                        showQTree = !showQTree;
                        break;
                    case sf::Keyboard::P:
                        showPaths = !showPaths;
                        if (!showPaths) pathRenderer.clearPaths();
                        else rerouteAll(npcMgr, graph, pathRenderer,
                                        goalNodeId, algo, showPaths, explored);
                        break;
                    case sf::Keyboard::E:
                        showExplored = !showExplored;
                        break;
                    case sf::Keyboard::C:
                        while (npcMgr.getNPCCount() > 0)
                            npcMgr.removeNPC(npcMgr.getNPCByIndex(0)->id);
                        pathRenderer.clearPaths();
                        explored.clear();
                        break;
                    case sf::Keyboard::R:
                        npcMgr.assignRandomGoals(graph, 12345u);
                        break;
                    default: break;
                }
            }

            // ── Left Click: set goal ───────────────────────────
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Left)
            {
                int clickRow = 0, clickCol = 0;
                worldToCell((float)event.mouseButton.x,
                            (float)event.mouseButton.y,
                            CELL_SIZE, clickRow, clickCol);
                int clickedNode = graph.getId(clickRow, clickCol);
                if (clickedNode != -1) {
                    goalNodeId = clickedNode;
                    pathRenderer.clearPaths();
                    explored.clear();

                    for (int i = 0; i < npcMgr.getNPCCount(); i++) {
                        NPC* npc = npcMgr.getNPCByIndex(i);
                        int startNode = graph.getId(
                            (int)(npc->y / CELL_SIZE),
                            (int)(npc->x / CELL_SIZE));
                        if (startNode == -1) continue;

                        Stack<int> p = computePath(graph, startNode,
                                                   goalNodeId, algo, explored);
                        // Use pre-computed path so BFS/DFS routing is actually followed
                        Stack<int> pCopy = p;
                        npc->setGoalWithPath(goalNodeId, pCopy);

                        if (showPaths) {
                            // Blend algo color with NPC color
                            sf::Color ac = algoColor(algo);
                            sf::Color nc(npc->colorR, npc->colorG, npc->colorB);
                            sf::Color blended(
                                (sf::Uint8)((ac.r + nc.r) / 2),
                                (sf::Uint8)((ac.g + nc.g) / 2),
                                (sf::Uint8)((ac.b + nc.b) / 2)
                            );
                            pathRenderer.addPath(p, blended);
                        }
                    }
                }
            }

            // ── Right Click: place/remove obstacle ────────────
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Right)
            {
                int clickRow = 0, clickCol = 0;
                worldToCell((float)event.mouseButton.x,
                            (float)event.mouseButton.y,
                            CELL_SIZE, clickRow, clickCol);

                if (clickRow >= 0 && clickRow < GRID_ROWS &&
                    clickCol >= 0 && clickCol < GRID_COLS)
                {
                    // Toggle obstacle
                    MAP[clickRow][clickCol] = MAP[clickRow][clickCol] ? 0 : 1;

                    // Rebuild graph and grid lines
                    rebuildGraph(graph);
                    gridRenderer.buildGridLines(graph);

                    // Re-route all NPCs automatically
                    rerouteAll(npcMgr, graph, pathRenderer,
                               goalNodeId, algo, showPaths, explored);
                }
            }

            // ── N key: spawn NPC at mouse position ────────────────────
            if (event.type == sf::Event::KeyPressed &&
                event.key.code == sf::Keyboard::N)
            {
                int clickRow = 0, clickCol = 0;
                worldToCell((float)sf::Mouse::getPosition(window).x,
                    (float)sf::Mouse::getPosition(window).y,
                    CELL_SIZE, clickRow, clickCol);
                if (graph.getId(clickRow, clickCol) != -1) {
                    float wx = clickCol * CELL_SIZE + CELL_SIZE * 0.5f;
                    float wy = clickRow * CELL_SIZE + CELL_SIZE * 0.5f;
                    NPC* newNPC = npcMgr.spawnNPC(wx, wy, 85.0f);

                    // Fix #2: immediately draw path for newly spawned NPC
                    if (newNPC && goalNodeId != -1) {
                        int startNode = graph.getId(clickRow, clickCol);
                        Stack<int> p = computePath(graph, startNode,
                                                   goalNodeId, algo, explored);
                        Stack<int> pCopy = p;
                        newNPC->setGoalWithPath(goalNodeId, pCopy);
                        if (showPaths) {
                            sf::Color ac = algoColor(algo);
                            sf::Color nc(newNPC->colorR, newNPC->colorG, newNPC->colorB);
                            sf::Color blended(
                                (sf::Uint8)((ac.r + nc.r) / 2),
                                (sf::Uint8)((ac.g + nc.g) / 2),
                                (sf::Uint8)((ac.b + nc.b) / 2)
                            );
                            pathRenderer.addPath(p, blended);
                        }
                    }
                }
            }
        }

        // ── Simulation update ─────────────────────────────────
        npcMgr.update(dt, graph);

        // ── Render ────────────────────────────────────────────
        window.clear(sf::Color(12, 12, 18));

        // 1. Grid (with dynamic obstacles)
        gridRenderer.draw(window, graph, hoverRow, hoverCol);

        // 2. Exploration overlay (E key)
        if (showExplored && explored.count > 0) {
            sf::Color algoCol = algoColor(algo);
            algoCol.a = 35;
            exploredTile.setFillColor(algoCol);
            for (int i = 0; i < explored.count; i++) {
                if (!graph.nodeExists(explored.nodes[i])) continue;
                const GraphNode& n = graph.getNode(explored.nodes[i]);
                exploredTile.setPosition(
                    n.x - CELL_SIZE * 0.5f + 1,
                    n.y - CELL_SIZE * 0.5f + 1);
                window.draw(exploredTile);
            }
        }

        // 3. Quadtree overlay (Q key)
        if (showQTree)
            qtRenderer.drawFromRoot(window,
                npcMgr.getQuadtree()->getRoot(), 0);

        // 4. Paths (P key) — animated
        if (showPaths)
            pathRenderer.draw(window, graph, animTimer);

        // 5. NPCs
        npcRenderer.draw(window, npcMgr);

        // 6. HUD
        const char* modeStr =
            algo == PathAlgo::ASTAR ? "A*" :
            algo == PathAlgo::BFS   ? "BFS" : "DFS";
        hud.draw(window, npcMgr, currentFps, modeStr,
                 hoverRow, hoverCol, showQTree, showPaths,
                 showExplored, explored.count);

        window.display();
    }
    return 0;
}
