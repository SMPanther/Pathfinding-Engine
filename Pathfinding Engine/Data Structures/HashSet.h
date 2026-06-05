#pragma once
// ============================================================
//  HashSet.h  —  Hash Set (Open Addressing, Linear Probing)
//  UCP BSCS — Data Structures Project
//  Used for: A* closed set — O(1) visited node lookup
//            BFS/DFS visited tracking
// ============================================================

class HashSet {
private:
    enum SlotState { EMPTY, OCCUPIED, DELETED };

    struct Slot {
        int       key;
        SlotState state;
        Slot() : key(-1), state(EMPTY) {}
    };

    Slot* table;
    int   capacity;
    int   count;

    static const float LOAD_FACTOR;  // 0.7

    int hash(int key) const {
        // Knuth multiplicative hash
        unsigned int k = (unsigned int)key;
        return (int)((k * 2654435769u) % (unsigned int)capacity);
    }

    void rehash() {
        int   oldCap   = capacity;
        Slot* oldTable = table;
        capacity = oldCap * 2;
        table    = new Slot[capacity];
        count    = 0;

        for (int i = 0; i < oldCap; i++) {
            if (oldTable[i].state == OCCUPIED)
                insert(oldTable[i].key);
        }
        delete[] oldTable;
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    HashSet(int initialCapacity = 128)
        : capacity(initialCapacity), count(0) {
        table = new Slot[capacity];
    }

    ~HashSet() { delete[] table; }

    HashSet(const HashSet& other)
        : capacity(other.capacity), count(0) {
        table = new Slot[capacity];
        for (int i = 0; i < capacity; i++)
            table[i] = other.table[i];
        count = other.count;
    }

    HashSet& operator=(const HashSet& other) {
        if (this == &other) return *this;
        delete[] table;
        capacity = other.capacity;
        count    = other.count;
        table    = new Slot[capacity];
        for (int i = 0; i < capacity; i++)
            table[i] = other.table[i];
        return *this;
    }

    // ── Core Operations ───────────────────────────────────────
    // O(1) amortised
    void insert(int key) {
        if ((float)(count + 1) / capacity > LOAD_FACTOR)
            rehash();

        int idx = hash(key);
        while (table[idx].state == OCCUPIED) {
            if (table[idx].key == key) return;  // already exists
            idx = (idx + 1) % capacity;
        }
        table[idx].key   = key;
        table[idx].state = OCCUPIED;
        count++;
    }

    // O(1) amortised
    bool contains(int key) const {
        int idx   = hash(key);
        int start = idx;
        while (table[idx].state != EMPTY) {
            if (table[idx].state == OCCUPIED && table[idx].key == key)
                return true;
            idx = (idx + 1) % capacity;
            if (idx == start) break;
        }
        return false;
    }

    // O(1) amortised — marks slot DELETED (tombstone)
    bool remove(int key) {
        int idx   = hash(key);
        int start = idx;
        while (table[idx].state != EMPTY) {
            if (table[idx].state == OCCUPIED && table[idx].key == key) {
                table[idx].state = DELETED;
                count--;
                return true;
            }
            idx = (idx + 1) % capacity;
            if (idx == start) break;
        }
        return false;
    }

    // ── Query ─────────────────────────────────────────────────
    bool isEmpty() const { return count == 0; }
    int  getSize() const { return count; }

    void clear() {
        for (int i = 0; i < capacity; i++)
            table[i] = Slot();
        count = 0;
    }
};

const float HashSet::LOAD_FACTOR = 0.7f;
