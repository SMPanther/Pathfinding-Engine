#pragma once
#include "LinkedList.h"
// ============================================================
//  Quadtree.h  —  Quadtree Spatial Partition
//  UCP BSCS — Data Structures Project
//  Used for: Fast NPC–NPC / NPC–obstacle proximity queries
//            Insert NPC by (x,y), query a rectangular region
//
//  Each leaf holds up to CAPACITY agents before subdividing.
//  Coordinate system matches SFML (origin top-left, y down).
// ============================================================

struct AABB {
    float x, y;        // centre
    float halfW, halfH;

    AABB() : x(0), y(0), halfW(0), halfH(0) {}
    AABB(float cx, float cy, float hw, float hh)
        : x(cx), y(cy), halfW(hw), halfH(hh) {}

    bool contains(float px, float py) const {
        return (px >= x - halfW && px <= x + halfW &&
                py >= y - halfH && py <= y + halfH);
    }

    bool intersects(const AABB& other) const {
        return !(other.x - other.halfW > x + halfW ||
                 other.x + other.halfW < x - halfW ||
                 other.y - other.halfH > y + halfH ||
                 other.y + other.halfH < y - halfH);
    }
};

struct AgentPoint {
    float px, py;
    int   agentId;
    AgentPoint() : px(0), py(0), agentId(-1) {}
    AgentPoint(float x, float y, int id) : px(x), py(y), agentId(id) {}
};

class QuadtreeNode {
private:
    static const int CAPACITY = 4;   // max agents before subdivide

    AABB               bounds;
    LinkedList<AgentPoint> points;   // agents stored here (leaf)
    QuadtreeNode*      children[4];  // NW, NE, SW, SE
    bool               divided;

    // ── Subdivide this node into 4 children ──────────────────
    void subdivide() {
        float hw = bounds.halfW / 2.0f;
        float hh = bounds.halfH / 2.0f;
        float cx = bounds.x;
        float cy = bounds.y;

        children[0] = new QuadtreeNode(AABB(cx - hw, cy - hh, hw, hh)); // NW
        children[1] = new QuadtreeNode(AABB(cx + hw, cy - hh, hw, hh)); // NE
        children[2] = new QuadtreeNode(AABB(cx - hw, cy + hh, hw, hh)); // SW
        children[3] = new QuadtreeNode(AABB(cx + hw, cy + hh, hw, hh)); // SE
        divided = true;

        // Re-insert existing points into children
        for (auto& pt : points) {
            for (int i = 0; i < 4; i++)
                if (children[i]->bounds.contains(pt.px, pt.py)) {
                    children[i]->insert(pt);
                    break;
                }
        }
        points.clear();
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    QuadtreeNode(const AABB& boundary)
        : bounds(boundary), divided(false) {
        for (int i = 0; i < 4; i++) children[i] = nullptr;
    }

    ~QuadtreeNode() {
        if (divided)
            for (int i = 0; i < 4; i++) {
                delete children[i];
                children[i] = nullptr;
            }
    }

    // ── Insert ────────────────────────────────────────────────
    // Returns false if point is out of this node's bounds
    bool insert(const AgentPoint& pt) {
        if (!bounds.contains(pt.px, pt.py)) return false;

        if (!divided && points.getSize() < CAPACITY) {
            points.insertBack(pt);
            return true;
        }

        if (!divided) subdivide();

        for (int i = 0; i < 4; i++)
            if (children[i]->insert(pt)) return true;

        return false;  // shouldn't happen
    }

    // ── Query ─────────────────────────────────────────────────
    // Fills 'result' with all agents whose position lies within 'range'
    void query(const AABB& range, LinkedList<AgentPoint>& result) const {
        if (!bounds.intersects(range)) return;

        if (!divided) {
            for (auto& pt : points)
                if (range.contains(pt.px, pt.py))
                    result.insertBack(pt);
            return;
        }

        for (int i = 0; i < 4; i++)
            children[i]->query(range, result);
    }

    // ── Clear (reset to empty leaf) ───────────────────────────
    void clear() {
        points.clear();
        if (divided) {
            for (int i = 0; i < 4; i++) {
                delete children[i];
                children[i] = nullptr;
            }
            divided = false;
        }
    }

    const AABB& getBounds() const { return bounds; }
    bool        isDivided() const { return divided; }
};

// ── Convenience wrapper ───────────────────────────────────────
class Quadtree {
    QuadtreeNode* root;

public:
    Quadtree(float worldW, float worldH) {
        root = new QuadtreeNode(AABB(worldW / 2.0f, worldH / 2.0f,
                                     worldW / 2.0f, worldH / 2.0f));
    }

    ~Quadtree() { delete root; }

    void insert(float x, float y, int agentId) {
        root->insert(AgentPoint(x, y, agentId));
    }

    // Query a rectangular region centred at (cx,cy) with half-dims hw x hh
    LinkedList<AgentPoint> query(float cx, float cy, float hw, float hh) const {
        LinkedList<AgentPoint> result;
        root->query(AABB(cx, cy, hw, hh), result);
        return result;
    }

    // Rebuild the tree each frame
    void clear() { root->clear(); }

    // Expose root for debug rendering
    const QuadtreeNode* getRoot() const { return root; }
};
