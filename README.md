# Pathfinding & Crowd Simulation Engine

A real-time simulation engine built from scratch in C++ as part of the
Data Structures & Algorithms course at University of Central Punjab (UCP), BSCS.

## Features
- A*, BFS, and DFS pathfinding with visual algorithm comparison
- 8-directional grid navigation with diagonal movement
- Dynamic obstacle placement and real-time NPC rerouting
- Crowd simulation with quadtree-based spatial partitioning
- NPC separation and collision avoidance
- Animated path visualization with per-algorithm color coding
- Exploration overlay showing visited nodes per algorithm
- SFML-based real-time rendering at 60 FPS

## Data Structures Used (No STL)
- Custom LinkedList, Stack, Queue, CircularQueue
- Min-Heap (A* open set)
- Hash Set & Hash Map (visited tracking, path reconstruction)
- Quadtree (spatial partitioning for NPC proximity queries)
- Circular Linked List (NPC patrol routes)

## Controls
| Input | Action |
|-------|--------|
| Left Click | Set goal for all NPCs |
| Right Click | Place / Remove obstacle |
| N | Spawn NPC at cursor |
| 1 / 2 / 3 | Switch A* / BFS / DFS |
| Q | Toggle Quadtree overlay |
| P | Toggle path visualization |
| E | Toggle exploration overlay |
| R | Assign random goals |
| C | Clear all NPCs |
| ESC | Quit |

## Built With
- C++17
- SFML 2.6

## Author
Umer — University of Central Punjab (UCP), BSCS
