#pragma once
// ============================================================
//  CircularQueue.h  —  Fixed-Capacity Ring Buffer (Templated)
//  UCP BSCS — Data Structures Project
//  Used for: NPC frame-by-frame action buffer
//            (overwrite old entries when full — ring behavior)
// ============================================================

template <typename T>
class CircularQueue {
private:
    T*  data;
    int frontIdx;
    int backIdx;
    int count;
    int capacity;

public:
    // ── Constructor / Destructor ──────────────────────────────
    explicit CircularQueue(int cap = 16)
        : frontIdx(0), backIdx(0), count(0), capacity(cap) {
        data = new T[capacity];
    }

    ~CircularQueue() { delete[] data; }

    CircularQueue(const CircularQueue<T>& other)
        : frontIdx(0), backIdx(0), count(other.count), capacity(other.capacity) {
        data = new T[capacity];
        for (int i = 0; i < count; i++)
            data[i] = other.data[(other.frontIdx + i) % other.capacity];
        backIdx = count % capacity;
    }

    CircularQueue<T>& operator=(const CircularQueue<T>& other) {
        if (this == &other) return *this;
        delete[] data;
        capacity = other.capacity;
        count    = other.count;
        data     = new T[capacity];
        for (int i = 0; i < count; i++)
            data[i] = other.data[(other.frontIdx + i) % other.capacity];
        frontIdx = 0;
        backIdx  = count % capacity;
        return *this;
    }

    // ── Core Operations ───────────────────────────────────────
    // Enqueue: overwrites oldest entry if full (ring behavior)
    void enqueue(const T& val) {
        if (count == capacity) {
            // Overwrite: advance front (discard oldest)
            data[backIdx] = val;
            backIdx  = (backIdx  + 1) % capacity;
            frontIdx = (frontIdx + 1) % capacity;
        } else {
            data[backIdx] = val;
            backIdx = (backIdx + 1) % capacity;
            count++;
        }
    }

    T dequeue() {
        if (isEmpty()) throw "CircularQueue::dequeue — queue is empty";
        T val    = data[frontIdx];
        frontIdx = (frontIdx + 1) % capacity;
        count--;
        return val;
    }

    T& peekFront() {
        if (isEmpty()) throw "CircularQueue::peekFront — queue is empty";
        return data[frontIdx];
    }

    // ── Query ─────────────────────────────────────────────────
    bool isEmpty()   const { return count == 0; }
    bool isFull()    const { return count == capacity; }
    int  getSize()   const { return count; }
    int  getCapacity() const { return capacity; }

    void clear() { frontIdx = backIdx = count = 0; }
};
