#pragma once
#include <SFML/Graphics.hpp>
#include "NPC/NPCSystem.h"
#include <cstdio>
// ============================================================
//  HUD.h  v2 — Stats panel with exploration count
// ============================================================

class HUD {
private:
    sf::Font  font;
    bool      fontLoaded;
    sf::Text  statsText;
    sf::Text  legendText;
    sf::RectangleShape panel;
    char statsBuf[600];
    char legendBuf[600];

public:
    HUD() : fontLoaded(false) {
        const char* fontPaths[] = {
            "C:\\Windows\\Fonts\\consola.ttf",
            "C:\\Windows\\Fonts\\cour.ttf",
            "C:\\Windows\\Fonts\\arial.ttf",
            "C:\\Windows\\Fonts\\calibri.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
            nullptr
        };
        for (int i = 0; fontPaths[i]; i++) {
            if (font.loadFromFile(fontPaths[i])) { fontLoaded = true; break; }
        }
        statsText.setFont(font);
        statsText.setCharacterSize(13);
        statsText.setFillColor(sf::Color(220,220,240));

        legendText.setFont(font);
        legendText.setCharacterSize(12);
        legendText.setFillColor(sf::Color(160,160,180));

        panel.setFillColor(sf::Color(15,15,25,210));
        panel.setOutlineThickness(1.0f);
        panel.setOutlineColor(sf::Color(60,60,90));
    }

    void draw(sf::RenderWindow& window,
              const NPCManager& mgr,
              float fps,
              const char* mode,
              int hoverRow, int hoverCol,
              bool showQTree, bool showPaths,
              bool showExplored, int exploredCount)
    {
        if (!fontLoaded) return;

        // ── Algo color indicator ──────────────────────────────
        sf::Color modeColor =
            (mode[0]=='A') ? sf::Color(80,220,120) :
            (mode[0]=='B') ? sf::Color(80,160,255) :
                             sf::Color(255,140,60);

        snprintf(statsBuf, sizeof(statsBuf),
            "FPS:      %.0f\n"
            "NPCs:     %d\n"
            "Moving:   %d\n"
            "Arrived:  %d\n"
            "Mode:     %s\n"
            "Cell:     (%d,%d)\n"
            "Explored: %d\n"
            "Paths:    %s\n"
            "Quadtree: %s\n"
            "Explore:  %s",
            fps,
            mgr.getNPCCount(),
            mgr.countMoving(),
            mgr.countArrived(),
            mode,
            hoverCol, hoverRow,
            exploredCount,
            showPaths    ? "ON" : "OFF",
            showQTree    ? "ON" : "OFF",
            showExplored ? "ON" : "OFF"
        );

        statsText.setString(statsBuf);
        statsText.setPosition(10.0f, 10.0f);
        sf::FloatRect sb = statsText.getLocalBounds();
        panel.setSize(sf::Vector2f(sb.width + 20, sb.height + 20));
        panel.setPosition(5.0f, 5.0f);
        window.draw(panel);

        // Mode color bar on left edge
        sf::RectangleShape bar(sf::Vector2f(4.0f, sb.height + 20));
        bar.setPosition(5.0f, 5.0f);
        bar.setFillColor(modeColor);
        window.draw(bar);

        window.draw(statsText);

        // ── Legend (bottom) ───────────────────────────────────
        snprintf(legendBuf, sizeof(legendBuf),
            "[LClick] Goal   [RClick] Place/Remove Obstacle   [N] Spawn NPC\n"
            "[1] A*(green)   [2] BFS(blue)   [3] DFS(orange)\n"
            "[Q] Quadtree    [P] Paths    [E] Exploration    [R] Random    [C] Clear"
        );
        legendText.setString(legendBuf);
        float winH = (float)window.getSize().y;
        sf::FloatRect lb = legendText.getLocalBounds();
        legendText.setPosition(10.0f, winH - lb.height - 30.0f);

        sf::RectangleShape lp;
        lp.setFillColor(sf::Color(15,15,25,190));
        lp.setOutlineThickness(1.0f);
        lp.setOutlineColor(sf::Color(60,60,90));
        lp.setSize(sf::Vector2f(lb.width+20, lb.height+20));
        lp.setPosition(5.0f, winH - lb.height - 35.0f);
        window.draw(lp);
        window.draw(legendText);
    }
};
