#pragma once
#include <SFML/Graphics.hpp>
#include "C:\VISUAL STUDIO PROJECTS\Pathfinding Engine\Pathfinding Engine\Data Structures\DataStructures.h"
#include <cmath>
// ============================================================
//  QuadtreeRenderer.h  v2
//  Draws quadtree subdivisions recursively with depth coloring
//  Depth 0 = faint outer boundary
//  Deeper = brighter + different hue so subdivisions are obvious
// ============================================================

class QuadtreeRenderer {
private:
    bool enabled = true;

    // Recursively draw AABB subdivisions
    // Since children are private in QuadtreeNode, we simulate
    // the subdivision pattern from parent bounds + isDivided flag
    void drawNode(sf::RenderWindow& window,
                  const QuadtreeNode* node,
                  int depth)
    {
        if (!node) return;

        const AABB& b = node->getBounds();
        float x = b.x - b.halfW;
        float y = b.y - b.halfH;
        float w = b.halfW * 2.0f;
        float h = b.halfH * 2.0f;

        // Color shifts from cyan (shallow) to magenta (deep)
        float t = std::min(depth / 5.0f, 1.0f);
        sf::Uint8 r = (sf::Uint8)(80  + t * 175);
        sf::Uint8 g = (sf::Uint8)(200 - t * 170);
        sf::Uint8 bv= (sf::Uint8)(255 - t * 80);
        sf::Uint8 a = (sf::Uint8)(40  + depth * 30);
        if (a > 200) a = 200;

        sf::RectangleShape rect(sf::Vector2f(w, h));
        rect.setPosition(x, y);
        rect.setFillColor(sf::Color::Transparent);
        rect.setOutlineThickness(1.5f);
        rect.setOutlineColor(sf::Color(r, g, bv, a));
        window.draw(rect);

        // If this node is subdivided, draw its 4 children regions
        if (node->isDivided()) {
            float hw = b.halfW / 2.0f;
            float hh = b.halfH / 2.0f;

            // NW, NE, SW, SE
            float cxs[4] = { b.x-hw, b.x+hw, b.x-hw, b.x+hw };
            float cys[4] = { b.y-hh, b.y-hh, b.y+hh, b.y+hh };

            for (int i = 0; i < 4; i++) {
                float cx = cxs[i] - hw;
                float cy = cys[i] - hh;
                float cw = hw * 2.0f;
                float ch = hh * 2.0f;

                float t2 = std::min((depth+1) / 5.0f, 1.0f);
                sf::Uint8 r2  = (sf::Uint8)(80  + t2 * 175);
                sf::Uint8 g2  = (sf::Uint8)(200 - t2 * 170);
                sf::Uint8 b2  = (sf::Uint8)(255 - t2 * 80);
                sf::Uint8 a2  = (sf::Uint8)(40  + (depth+1) * 30);
                if (a2 > 200) a2 = 200;

                sf::RectangleShape child(sf::Vector2f(cw, ch));
                child.setPosition(cx, cy);
                child.setFillColor(sf::Color::Transparent);
                child.setOutlineThickness(1.0f);
                child.setOutlineColor(sf::Color(r2, g2, b2, a2));
                window.draw(child);
            }
        }
    }

public:
    void setEnabled(bool on) { enabled = on; }
    bool isEnabled()   const { return enabled; }
    void toggle()            { enabled = !enabled; }

    void draw(sf::RenderWindow& /*window*/, const Quadtree* /*qt*/) {
        if (!enabled) return;
    }

    void drawFromRoot(sf::RenderWindow& window,
                      const QuadtreeNode* root,
                      int depth = 0)
    {
        if (!enabled || !root) return;
        drawRecursive(window, root, depth);
    }

private:
    void drawRecursive(sf::RenderWindow& window,
                       const QuadtreeNode* node,
                       int depth)
    {
        if (!node) return;
        drawNode(window, node, depth);

        if (node->isDivided()) {
            // Simulate children at subdivided bounds
            const AABB& b  = node->getBounds();
            float hw = b.halfW / 2.0f;
            float hh = b.halfH / 2.0f;

            float cxs[4] = { b.x-hw, b.x+hw, b.x-hw, b.x+hw };
            float cys[4] = { b.y-hh, b.y-hh, b.y+hh, b.y+hh };

            for (int i = 0; i < 4; i++) {
                // Create temporary node-like AABB for next level
                // We can't recurse into private children, so we draw
                // up to depth 3 by simulating further subdivision
                if (depth < 3) {
                    AABB childBounds(cxs[i], cys[i], hw, hh);
                    // Draw this sub-region manually
                    float t2 = std::min((depth+2) / 5.0f, 1.0f);
                    sf::Uint8 r2  = (sf::Uint8)(80  + t2*175);
                    sf::Uint8 g2  = (sf::Uint8)(200 - t2*170);
                    sf::Uint8 bv2 = (sf::Uint8)(255 - t2*80);
                    sf::Uint8 a2  = (sf::Uint8)(40  + (depth+2)*30);
                    if (a2>200) a2=200;
                    sf::RectangleShape sub(sf::Vector2f(hw*2, hh*2));
                    sub.setPosition(cxs[i]-hw, cys[i]-hh);
                    sub.setFillColor(sf::Color::Transparent);
                    sub.setOutlineThickness(0.8f);
                    sub.setOutlineColor(sf::Color(r2,g2,bv2,a2));
                    window.draw(sub);
                }
            }
        }
    }
};
