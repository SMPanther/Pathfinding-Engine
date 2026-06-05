#pragma once
// ============================================================
//  Queue.h  —  FIFO Queue (Templated, Circular Array)
//  UCP BSCS — Data Structures Project
//  Used for: BFS traversal, NPC update scheduling,
//            general event queuing
// ============================================================

template <typename T>
class Queue {
private:
    T*  data;
    int frontIdx;
    int backIdx;
    int count;
    int capacity;

    void resize() {
        int newCap  = capacity * 2;
        T*  newData = new T[newCap];
        // Copy in logical order
        for (int i = 0; i < count; i++)
            newData[i] = data[(frontIdx + i) % capacity];
        delete[] data;
        data     = newData;
        frontIdx = 0;
        backIdx  = count;
        capacity = newCap;
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    Queue(int initialCapacity = 64)
        : frontIdx(0), backIdx(0), count(0), capacity(initialCapacity) {
        data = new T[capacity];
    }

    ~Queue() { delete[] data; }

    // Deep copy constructor
    Queue(const Queue<T>& other)
        : frontIdx(0), backIdx(0), count(other.count), capacity(other.capacity) {
        data = new T[capacity];
        for (int i = 0; i < count; i++)
            data[i] = other.data[(other.frontIdx + i) % other.capacity];
        backIdx = count;
    }

    // Assignment operator
    Queue<T>& operator=(const Queue<T>& other) {
        if (this == &other) return *this;
        delete[] data;
        capacity = other.capacity;
        count    = other.count;
        frontIdx = 0;
        backIdx  = count;
        data     = new T[capacity];
        for (int i = 0; i < count; i++)
            data[i] = other.data[(other.frontIdx + i) % other.capacity];
        return *this;
    }

    // ── Core Operations ───────────────────────────────────────
    // O(1) amortised
    void enqueue(const T& val) {
        if (count == capacity) resize();
        data[backIdx] = val;
        backIdx = (backIdx + 1) % capacity;
        count++;
    }

    // O(1)
    T dequeue() {
        if (isEmpty()) throw "Queue::dequeue — queue is empty";
        T val    = data[frontIdx];
        frontIdx = (frontIdx + 1) % capacity;
        count--;
        return val;
    }

    // O(1)
    T& peekFront() {
        if (isEmpty()) throw "Queue::peekFront — queue is empty";
        return data[frontIdx];
    }

    // ── Query ─────────────────────────────────────────────────
    bool isEmpty() const { return count == 0; }
    int  getSize() const { return count; }

    // ── Utility ───────────────────────────────────────────────
    void clear() { frontIdx = backIdx = count = 0; }
};
