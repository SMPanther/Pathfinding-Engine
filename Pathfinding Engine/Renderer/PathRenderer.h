#pragma once
#include <SFML/Graphics.hpp>
#include "Graph/Graph.h"
#include "C:\VISUAL STUDIO PROJECTS\Pathfinding Engine\Pathfinding Engine\Data Structures\DataStructures.h"
#include <cmath>
// ============================================================
//  PathRenderer.h  —  Animated path visualization
//  Colors per algorithm:
//    A* = green   BFS = blue   DFS = orange
//  Features: pulsing glow, moving dash animation
// ============================================================

class PathRenderer {
private:
    static const int MAX_PATH_NODES = 512;
    static const int MAX_PATHS      = 64;

    struct PathEntry {
        int       nodeIds[MAX_PATH_NODES];
        int       count;
        sf::Color color;
    };

    PathEntry paths[MAX_PATHS];
    int       pathCount;

public:
    PathRenderer() : pathCount(0) {}
    void clearPaths() { pathCount = 0; }

    void addPath(Stack<int> pathStack, sf::Color color) {
        if (pathCount >= MAX_PATHS) return;
        PathEntry& entry = paths[pathCount];
        entry.count = 0;
        entry.color = color;

        int tmp[MAX_PATH_NODES];
        int n = 0;
        while (!pathStack.isEmpty() && n < MAX_PATH_NODES)
            tmp[n++] = pathStack.pop();
        for (int i = n - 1; i >= 0; i--)
            entry.nodeIds[entry.count++] = tmp[i];
        pathCount++;
    }

    void draw(sf::RenderWindow& window, const Graph& graph,
              float animTimer = 0.0f)
    {
        float cell = graph.getCellSize();

        // Pulse: oscillate alpha between 40 and 90
        float pulse = 0.5f + 0.5f * std::sin(animTimer * 3.0f);

        for (int p = 0; p < pathCount; p++) {
            PathEntry& entry = paths[p];
            if (entry.count < 2) continue;

            // ── Tile highlight (pulsing) ──────────────────────
            sf::RectangleShape tile(sf::Vector2f(cell - 4, cell - 4));
            sf::Color tileCol = entry.color;
            tileCol.a = (sf::Uint8)(35 + pulse * 40);
            tile.setFillColor(tileCol);

            for (int i = 1; i < entry.count - 1; i++) {
                if (!graph.nodeExists(entry.nodeIds[i])) continue;
                const GraphNode& n = graph.getNode(entry.nodeIds[i]);
                tile.setPosition(n.x - cell*0.5f + 2, n.y - cell*0.5f + 2);
                window.draw(tile);
            }

            // ── Connecting line (solid) ───────────────────────
            sf::VertexArray line(sf::LinesStrip, entry.count);
            sf::Color lineCol = entry.color;
            lineCol.a = 200;
            for (int i = 0; i < entry.count; i++) {
                if (!graph.nodeExists(entry.nodeIds[i])) continue;
                const GraphNode& n = graph.getNode(entry.nodeIds[i]);
                line[i].position = sf::Vector2f(n.x, n.y);
                line[i].color    = lineCol;
            }
            window.draw(line);

            // ── Animated moving dot along path ────────────────
            float totalLen = 0.0f;
            for (int i = 0; i < entry.count - 1; i++) {
                if (!graph.nodeExists(entry.nodeIds[i])) continue;
                if (!graph.nodeExists(entry.nodeIds[i+1])) continue;
                const GraphNode& a = graph.getNode(entry.nodeIds[i]);
                const GraphNode& b = graph.getNode(entry.nodeIds[i+1]);
                float dx = b.x - a.x, dy = b.y - a.y;
                totalLen += std::sqrt(dx*dx + dy*dy);
            }
            if (totalLen > 0.0f) {
                float t = std::fmod(animTimer * 80.0f, totalLen);
                float walked = 0.0f;
                sf::Vector2f dotPos;
                for (int i = 0; i < entry.count - 1; i++) {
                    if (!graph.nodeExists(entry.nodeIds[i])) continue;
                    if (!graph.nodeExists(entry.nodeIds[i+1])) continue;
                    const GraphNode& a = graph.getNode(entry.nodeIds[i]);
                    const GraphNode& b = graph.getNode(entry.nodeIds[i+1]);
                    float dx = b.x - a.x, dy = b.y - a.y;
                    float segLen = std::sqrt(dx*dx + dy*dy);
                    if (walked + segLen >= t) {
                        float frac = (t - walked) / segLen;
                        dotPos = sf::Vector2f(a.x + dx*frac, a.y + dy*frac);
                        break;
                    }
                    walked += segLen;
                }
                sf::CircleShape dot(4.0f);
                dot.setOrigin(4.0f, 4.0f);
                dot.setFillColor(sf::Color(255,255,255,200));
                dot.setPosition(dotPos);
                window.draw(dot);
            }

            // ── Start marker (green) ──────────────────────────
            if (graph.nodeExists(entry.nodeIds[0])) {
                const GraphNode& s = graph.getNode(entry.nodeIds[0]);
                sf::CircleShape mk(5.0f);
                mk.setOrigin(5.0f, 5.0f);
                mk.setFillColor(sf::Color(80,255,120,220));
                mk.setPosition(s.x, s.y);
                window.draw(mk);
            }

            // ── Goal marker (red, pulsing size) ───────────────
            int last = entry.nodeIds[entry.count - 1];
            if (graph.nodeExists(last)) {
                const GraphNode& g = graph.getNode(last);
                float r = 5.0f + pulse * 3.0f;
                sf::CircleShape mk(r);
                mk.setOrigin(r, r);
                mk.setFillColor(sf::Color(255,80,80,220));
                mk.setPosition(g.x, g.y);
                window.draw(mk);
            }
        }
    }
};
