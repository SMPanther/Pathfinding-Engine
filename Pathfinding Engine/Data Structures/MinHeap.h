#pragma once
// ============================================================
//  MinHeap.h  —  Min-Heap Priority Queue
//  UCP BSCS — Data Structures Project
//  Used for: A* open set — always expand lowest f(n) first
//
//  PathNode stores: nodeId, fCost = g(n) + h(n), gCost = g(n)
//  Parent key is always <= children (min-heap property).
// ============================================================

struct PathNode {
    int   nodeId;
    float fCost;   // f(n) = g(n) + h(n)
    float gCost;   // g(n) = cost from start

    PathNode() : nodeId(-1), fCost(0.0f), gCost(0.0f) {}
    PathNode(int id, float f, float g) : nodeId(id), fCost(f), gCost(g) {}
};

class MinHeap {
private:
    PathNode* data;
    int       size;
    int       capacity;

    // ── Internal helpers ──────────────────────────────────────
    int parent(int i) const { return (i - 1) / 2; }
    int left  (int i) const { return 2 * i + 1; }
    int right (int i) const { return 2 * i + 2; }

    void swap(int a, int b) {
        PathNode tmp = data[a];
        data[a]      = data[b];
        data[b]      = tmp;
    }

    // Bubble newly inserted node UP until heap property holds
    void heapifyUp(int i) {
        while (i > 0 && data[parent(i)].fCost > data[i].fCost) {
            swap(i, parent(i));
            i = parent(i);
        }
    }

    // Push root DOWN after extractMin until heap property holds
    void heapifyDown(int i) {
        int smallest = i;
        int l = left(i);
        int r = right(i);

        if (l < size && data[l].fCost < data[smallest].fCost)
            smallest = l;
        if (r < size && data[r].fCost < data[smallest].fCost)
            smallest = r;

        if (smallest != i) {
            swap(i, smallest);
            heapifyDown(smallest);
        }
    }

    void resize() {
        capacity *= 2;
        PathNode* newData = new PathNode[capacity];
        for (int i = 0; i < size; i++)
            newData[i] = data[i];
        delete[] data;
        data = newData;
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    MinHeap(int initialCapacity = 256)
        : size(0), capacity(initialCapacity) {
        data = new PathNode[capacity];
    }

    ~MinHeap() { delete[] data; }

    // Deep copy
    MinHeap(const MinHeap& other)
        : size(other.size), capacity(other.capacity) {
        data = new PathNode[capacity];
        for (int i = 0; i < size; i++)
            data[i] = other.data[i];
    }

    MinHeap& operator=(const MinHeap& other) {
        if (this == &other) return *this;
        delete[] data;
        size     = other.size;
        capacity = other.capacity;
        data     = new PathNode[capacity];
        for (int i = 0; i < size; i++)
            data[i] = other.data[i];
        return *this;
    }

    // ── Core Operations ───────────────────────────────────────
    // O(log n)
    void insert(const PathNode& node) {
        if (size == capacity) resize();
        data[size++] = node;
        heapifyUp(size - 1);
    }

    // Returns and removes the node with smallest fCost — O(log n)
    PathNode extractMin() {
        if (isEmpty()) throw "MinHeap::extractMin — heap is empty";
        PathNode minNode = data[0];
        data[0] = data[--size];
        heapifyDown(0);
        return minNode;
    }

    // Peek without removing — O(1)
    const PathNode& peekMin() const {
        if (isEmpty()) throw "MinHeap::peekMin — heap is empty";
        return data[0];
    }

    // Check if a nodeId already exists in the heap — O(n)
    // Used to decide whether to insert or update
    bool contains(int nodeId) const {
        for (int i = 0; i < size; i++)
            if (data[i].nodeId == nodeId) return true;
        return false;
    }

    // Update fCost for an existing node and re-heapify — O(n)
    // Called when a better path to an already-open node is found
    void decreaseKey(int nodeId, float newFCost, float newGCost) {
        for (int i = 0; i < size; i++) {
            if (data[i].nodeId == nodeId) {
                data[i].fCost = newFCost;
                data[i].gCost = newGCost;
                heapifyUp(i);
                return;
            }
        }
    }

    // ── Query ─────────────────────────────────────────────────
    bool isEmpty() const { return size == 0; }
    int  getSize() const { return size; }

    void clear() { size = 0; }
};
