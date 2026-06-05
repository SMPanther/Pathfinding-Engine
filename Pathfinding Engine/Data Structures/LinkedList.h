#pragma once
// ============================================================
//  LinkedList.h  —  Singly Linked List (Templated)
//  UCP BSCS — Data Structures Project
//  Used for: Graph adjacency lists, NPC neighbor lists,
//            QuadtreeNode agent storage
// ============================================================

template <typename T>
class LinkedList {
private:
    struct Node {
        T     data;
        Node* next;
        Node(const T& val) : data(val), next(nullptr) {}
    };

    Node* head;
    Node* tail;
    int   count;

public:
    // ── Constructor / Destructor ──────────────────────────────
    LinkedList() : head(nullptr), tail(nullptr), count(0) {}

    ~LinkedList() { clear(); }

    // Deep copy constructor
    LinkedList(const LinkedList<T>& other) : head(nullptr), tail(nullptr), count(0) {
        Node* cur = other.head;
        while (cur) {
            insertBack(cur->data);
            cur = cur->next;
        }
    }

    // Assignment operator
    LinkedList<T>& operator=(const LinkedList<T>& other) {
        if (this == &other) return *this;
        clear();
        Node* cur = other.head;
        while (cur) {
            insertBack(cur->data);
            cur = cur->next;
        }
        return *this;
    }

    // ── Insert ────────────────────────────────────────────────
    // O(1) — prepend
    void insertFront(const T& val) {
        Node* n = new Node(val);
        if (!head) { head = tail = n; }
        else       { n->next = head; head = n; }
        count++;
    }

    // O(1) — append (tail pointer maintained)
    void insertBack(const T& val) {
        Node* n = new Node(val);
        if (!tail) { head = tail = n; }
        else       { tail->next = n; tail = n; }
        count++;
    }

    // ── Remove ────────────────────────────────────────────────
    // Remove first occurrence of val — O(n)
    bool remove(const T& val) {
        Node* prev = nullptr;
        Node* cur  = head;
        while (cur) {
            if (cur->data == val) {
                if (prev) prev->next = cur->next;
                else      head      = cur->next;
                if (cur == tail) tail = prev;
                delete cur;
                count--;
                return true;
            }
            prev = cur;
            cur  = cur->next;
        }
        return false;
    }

    // Remove and return front element — O(1)
    T removeFront() {
        if (!head) throw "LinkedList::removeFront — list is empty";
        T val = head->data;
        Node* old = head;
        head = head->next;
        if (!head) tail = nullptr;
        delete old;
        count--;
        return val;
    }

    // ── Query ─────────────────────────────────────────────────
    bool contains(const T& val) const {
        Node* cur = head;
        while (cur) {
            if (cur->data == val) return true;
            cur = cur->next;
        }
        return false;
    }

    bool isEmpty()  const { return count == 0; }
    int  getSize()  const { return count; }
    T    getFront() const {
        if (!head) throw "LinkedList::getFront — list is empty";
        return head->data;
    }

    // ── Clear ─────────────────────────────────────────────────
    void clear() {
        while (head) {
            Node* tmp = head;
            head = head->next;
            delete tmp;
        }
        tail  = nullptr;
        count = 0;
    }

    // ── Iterator (range-for compatible) ──────────────────────
    class Iterator {
        Node* cur;
    public:
        Iterator(Node* start) : cur(start) {}
        bool      hasNext()  const { return cur != nullptr; }
        T&        next()           { T& v = cur->data; cur = cur->next; return v; }
        // Standard iterator interface
        Iterator& operator++()     { cur = cur->next; return *this; }
        T&        operator*()      { return cur->data; }
        bool      operator!=(const Iterator& o) const { return cur != o.cur; }
    };

    Iterator begin() const { return Iterator(head); }
    Iterator end()   const { return Iterator(nullptr); }
};
