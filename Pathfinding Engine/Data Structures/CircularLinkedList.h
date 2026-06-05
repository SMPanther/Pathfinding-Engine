#pragma once
// ============================================================
//  CircularLinkedList.h  —  Circular Singly Linked List (Templated)
//  UCP BSCS — Data Structures Project
//  Used for: NPC cyclic waypoint patrol paths
//            (last node points back to first — loops forever)
// ============================================================

template <typename T>
class CircularLinkedList {
private:
    struct Node {
        T     data;
        Node* next;
        Node(const T& val) : data(val), next(nullptr) {}
    };

    Node* tail;   // tail->next == head (the circular link)
    int   count;

public:
    // ── Constructor / Destructor ──────────────────────────────
    CircularLinkedList() : tail(nullptr), count(0) {}

    ~CircularLinkedList() { clear(); }

    CircularLinkedList(const CircularLinkedList<T>& other)
        : tail(nullptr), count(0) {
        if (!other.tail) return;
        Node* start = other.tail->next; // head
        Node* cur   = start;
        do {
            insertBack(cur->data);
            cur = cur->next;
        } while (cur != start);
    }

    CircularLinkedList<T>& operator=(const CircularLinkedList<T>& other) {
        if (this == &other) return *this;
        clear();
        if (!other.tail) return *this;
        Node* start = other.tail->next;
        Node* cur   = start;
        do {
            insertBack(cur->data);
            cur = cur->next;
        } while (cur != start);
        return *this;
    }

    // ── Insert ────────────────────────────────────────────────
    // Append to back, maintaining circular link — O(1)
    void insertBack(const T& val) {
        Node* n = new Node(val);
        if (!tail) {
            tail   = n;
            n->next = n;  // points to itself
        } else {
            n->next    = tail->next;  // new node -> head
            tail->next = n;           // old tail  -> new node
            tail       = n;           // advance tail
        }
        count++;
    }

    // ── Access ────────────────────────────────────────────────
    // Returns the head node's data (first inserted)
    T& front() {
        if (!tail) throw "CircularLinkedList::front — list is empty";
        return tail->next->data;
    }

    // ── Advance head (for patrol: move to next waypoint) ─────
    // Rotates: old head becomes last; new head = old head->next
    void advanceHead() {
        if (!tail) throw "CircularLinkedList::advanceHead — list is empty";
        tail = tail->next;  // tail advances = head advances
    }

    // ── Remove ────────────────────────────────────────────────
    bool remove(const T& val) {
        if (!tail) return false;
        Node* prev = tail;
        Node* cur  = tail->next;  // start at head
        do {
            if (cur->data == val) {
                if (count == 1) {
                    delete cur;
                    tail = nullptr;
                } else {
                    prev->next = cur->next;
                    if (cur == tail) tail = prev;
                    delete cur;
                }
                count--;
                return true;
            }
            prev = cur;
            cur  = cur->next;
        } while (cur != tail->next);
        return false;
    }

    // ── Query ─────────────────────────────────────────────────
    bool isEmpty() const { return count == 0; }
    int  getSize() const { return count; }

    // ── Clear ─────────────────────────────────────────────────
    void clear() {
        if (!tail) return;
        Node* head = tail->next;
        tail->next = nullptr;   // break the circle
        Node* cur  = head;
        while (cur) {
            Node* tmp = cur;
            cur = cur->next;
            delete tmp;
        }
        tail  = nullptr;
        count = 0;
    }

    // ── Circular Iterator ─────────────────────────────────────
    // Iterates once through all nodes starting from head
    class Iterator {
        Node* start;
        Node* cur;
        bool  done;
    public:
        Iterator(Node* head) : start(head), cur(head), done(head == nullptr) {}
        bool hasNext() const { return !done; }
        T&   next() {
            T& v = cur->data;
            cur  = cur->next;
            if (cur == start) done = true;
            return v;
        }
    };

    Iterator begin() const {
        return Iterator(tail ? tail->next : nullptr);
    }
};
