# ============================================================
#  Makefile  —  Pathfinding & Crowd Simulation Engine
#  UCP BSCS — Data Structures Project
#
#  Usage:
#    make          → build the simulation (./engine)
#    make run      → build and launch
#    make clean    → remove binaries
#    make test_ds  → run Phase 1 data structure tests
#    make test_g   → run Phase 2 graph tests
#    make test_pf  → run Phase 3 pathfinding tests
#    make test_npc → run Phase 4 NPC tests
#    make test_all → run all test suites
# ============================================================

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
SFML     = -lsfml-graphics -lsfml-window -lsfml-system

TARGET   = engine

# ── Main simulation ──────────────────────────────────────────
$(TARGET): main.cpp
	$(CXX) $(CXXFLAGS) -o $(TARGET) main.cpp $(SFML)

run: $(TARGET)
	./$(TARGET)

# ── Unit tests ───────────────────────────────────────────────
test_ds: test_ds.cpp
	$(CXX) $(CXXFLAGS) -o test_ds test_ds.cpp
	./test_ds

test_g: test_graph.cpp
	$(CXX) $(CXXFLAGS) -o test_graph test_graph.cpp
	./test_graph

test_pf: test_pathfinding.cpp
	$(CXX) $(CXXFLAGS) -o test_pathfinding test_pathfinding.cpp
	./test_pathfinding

test_npc: test_npc.cpp
	$(CXX) $(CXXFLAGS) -o test_npc test_npc.cpp
	./test_npc

test_all: test_ds test_g test_pf test_npc
	@echo ""
	@echo "All test suites passed."

# ── Clean ─────────────────────────────────────────────────────
clean:
	rm -f $(TARGET) test_ds test_graph test_pathfinding test_npc

.PHONY: run clean test_ds test_g test_pf test_npc test_all
