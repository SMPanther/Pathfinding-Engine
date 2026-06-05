#pragma once
// ============================================================
//  Stack.h  —  LIFO Stack (Templated, Dynamic Array)
//  UCP BSCS — Data Structures Project
//  Used for: DFS traversal, path reconstruction reversal,
//            NPC path storage (pop node each frame)
// ============================================================

template <typename T>
class Stack {
private:
    T*  data;
    int top;
    int capacity;

    void resize() {
        capacity *= 2;
        T* newData = new T[capacity];
        for (int i = 0; i <= top; i++)
            newData[i] = data[i];
        delete[] data;
        data = newData;
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    Stack(int initialCapacity = 64)
        : top(-1), capacity(initialCapacity) {
        data = new T[capacity];
    }

    ~Stack() { delete[] data; }

    // Deep copy constructor
    Stack(const Stack<T>& other)
        : top(other.top), capacity(other.capacity) {
        data = new T[capacity];
        for (int i = 0; i <= top; i++)
            data[i] = other.data[i];
    }

    // Assignment operator
    Stack<T>& operator=(const Stack<T>& other) {
        if (this == &other) return *this;
        delete[] data;
        top      = other.top;
        capacity = other.capacity;
        data     = new T[capacity];
        for (int i = 0; i <= top; i++)
            data[i] = other.data[i];
        return *this;
    }

    // ── Core Operations ───────────────────────────────────────
    // O(1) amortised
    void push(const T& val) {
        if (top + 1 == capacity) resize();
        data[++top] = val;
    }

    // O(1)
    T pop() {
        if (isEmpty()) throw "Stack::pop — stack is empty";
        return data[top--];
    }

    // O(1)
    T& peek() {
        if (isEmpty()) throw "Stack::peek — stack is empty";
        return data[top];
    }

    const T& peek() const {
        if (isEmpty()) throw "Stack::peek — stack is empty";
        return data[top];
    }

    // ── Query ─────────────────────────────────────────────────
    bool isEmpty() const { return top == -1; }
    int  getSize() const { return top + 1; }

    // ── Utility ───────────────────────────────────────────────
    void clear() { top = -1; }
};
