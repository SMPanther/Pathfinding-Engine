#pragma once
#include <SFML/Graphics.hpp>
#include "Graph/Graph.h"
// ============================================================
//  GridRenderer.h  v2
//  - Dynamic obstacle cells shown in red tint when placed
//  - Hover highlight
//  - Thin grid lines
// ============================================================

class GridRenderer {
private:
    sf::VertexArray gridLines;

    sf::Color walkableColor  = sf::Color( 42,  42,  52);
    sf::Color obstacleColor  = sf::Color( 18,  18,  26);
    sf::Color lineColor      = sf::Color( 65,  65,  80, 160);
    sf::Color highlightColor = sf::Color(100, 180, 255,  55);
    sf::Color obstacleHover  = sf::Color(255,  80,  80,  80);

public:
    void buildGridLines(const Graph& graph) {
        int   rows = graph.getGridRows();
        int   cols = graph.getGridCols();
        float cell = graph.getCellSize();

        gridLines.setPrimitiveType(sf::Lines);
        gridLines.resize(((rows + 1) + (cols + 1)) * 2);

        int idx = 0;
        for (int c = 0; c <= cols; c++) {
            gridLines[idx  ].position = sf::Vector2f(c * cell, 0);
            gridLines[idx  ].color    = lineColor;
            gridLines[idx+1].position = sf::Vector2f(c * cell, rows * cell);
            gridLines[idx+1].color    = lineColor;
            idx += 2;
        }
        for (int r = 0; r <= rows; r++) {
            gridLines[idx  ].position = sf::Vector2f(0,          r * cell);
            gridLines[idx  ].color    = lineColor;
            gridLines[idx+1].position = sf::Vector2f(cols * cell, r * cell);
            gridLines[idx+1].color    = lineColor;
            idx += 2;
        }
    }

    void draw(sf::RenderWindow& window, const Graph& graph,
              int highlightRow = -1, int highlightCol = -1)
    {
        float cell = graph.getCellSize();
        int   rows = graph.getGridRows();
        int   cols = graph.getGridCols();

        sf::RectangleShape shape(sf::Vector2f(cell, cell));

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                float px = c * cell;
                float py = r * cell;
                shape.setPosition(px, py);

                bool walkable = graph.nodeExists(graph.getId(r, c));
                shape.setFillColor(walkable ? walkableColor : obstacleColor);
                window.draw(shape);

                // Hover over walkable cell → blue tint
                if (r == highlightRow && c == highlightCol) {
                    sf::RectangleShape hl(sf::Vector2f(cell, cell));
                    hl.setPosition(px, py);
                    hl.setFillColor(walkable ? highlightColor : obstacleHover);
                    window.draw(hl);
                }
            }
        }

        window.draw(gridLines);
    }
};
