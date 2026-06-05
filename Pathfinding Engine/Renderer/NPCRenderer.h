#pragma once
#include <SFML/Graphics.hpp>
#include "NPC/NPCSystem.h"
#include <cmath>
// ============================================================
//  NPCRenderer.h  v2
//  - Smooth body with glow ring
//  - Direction indicator (small dot toward next waypoint)
//  - Status ring: green=moving, yellow=patrol, blue=arrived
//  - Shadow beneath each NPC
// ============================================================

class NPCRenderer {
public:
    void draw(sf::RenderWindow& window, const NPCManager& mgr)
    {
        for (int i = 0; i < mgr.getNPCCount(); i++) {
            const NPC* npc = mgr.getNPCByIndex(i);
            if (!npc) continue;

            sf::Vector2f pos(npc->x, npc->y);

            // ── Shadow ────────────────────────────────────────
            sf::CircleShape shadow(NPC_RADIUS + 2.0f);
            shadow.setOrigin(NPC_RADIUS + 2.0f, NPC_RADIUS + 2.0f);
            shadow.setFillColor(sf::Color(0, 0, 0, 60));
            shadow.setPosition(pos.x + 3.0f, pos.y + 3.0f);
            window.draw(shadow);

            // ── Status ring color ─────────────────────────────
            sf::Color ringColor;
            switch (npc->state) {
                case NPCState::MOVING:
                    ringColor = sf::Color(80, 230, 100, 200); break;
                case NPCState::PATROLLING:
                    ringColor = sf::Color(255, 210, 60,  200); break;
                case NPCState::ARRIVED:
                    ringColor = sf::Color(100, 160, 255, 200); break;
                default:
                    ringColor = sf::Color(120, 120, 120, 140); break;
            }

            // ── Outer glow ring ───────────────────────────────
            sf::CircleShape glow(NPC_RADIUS + 5.0f);
            glow.setOrigin(NPC_RADIUS + 5.0f, NPC_RADIUS + 5.0f);
            glow.setFillColor(sf::Color::Transparent);
            glow.setOutlineThickness(2.5f);
            glow.setOutlineColor(ringColor);
            glow.setPosition(pos);
            window.draw(glow);

            // ── Body ──────────────────────────────────────────
            sf::CircleShape body(NPC_RADIUS);
            body.setOrigin(NPC_RADIUS, NPC_RADIUS);
            body.setFillColor(sf::Color(npc->colorR, npc->colorG,
                                        npc->colorB, 235));
            body.setPosition(pos);
            window.draw(body);

            // ── White outline ─────────────────────────────────
            sf::CircleShape outline(NPC_RADIUS);
            outline.setOrigin(NPC_RADIUS, NPC_RADIUS);
            outline.setFillColor(sf::Color::Transparent);
            outline.setOutlineThickness(1.5f);
            outline.setOutlineColor(sf::Color(255, 255, 255, 100));
            outline.setPosition(pos);
            window.draw(outline);

            // ── Direction dot toward next waypoint ───────────
            if (npc->isMoving() && !npc->path.isEmpty()) {
                // Direction: from NPC toward next waypoint on stack
                // path.peek() = next node but we can't get world pos
                // without graph — approximate: show a static top dot
                sf::CircleShape dot(2.5f);
                dot.setOrigin(2.5f, 2.5f);
                dot.setFillColor(sf::Color(255, 255, 255, 210));
                // Place dot slightly above centre
                dot.setPosition(pos.x, pos.y - (NPC_RADIUS - 3.5f));
                window.draw(dot);
            }

            // ── ID label (small number inside circle) ─────────
            // Skipped if no font — handled by HUD
        }
    }
};
