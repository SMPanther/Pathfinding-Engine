#pragma once
// ============================================================
//  HashMap.h  —  Hash Map  int → int  (Open Addressing)
//  UCP BSCS — Data Structures Project
//  Used for: A* cameFrom map (nodeId → parentId)
//            gCost map     (nodeId → best g(n) so far)
// ============================================================

class HashMap {
private:
    enum SlotState { EMPTY, OCCUPIED, DELETED };

    struct Slot {
        int       key;
        int       value;
        SlotState state;
        Slot() : key(-1), value(-1), state(EMPTY) {}
    };

    Slot* table;
    int   capacity;
    int   count;

    static const float LOAD_FACTOR;   // 0.7

    int hash(int key) const {
        unsigned int k = (unsigned int)key;
        return (int)((k * 2654435769u) % (unsigned int)capacity);
    }

    void rehash() {
        int   oldCap   = capacity;
        Slot* oldTable = table;
        capacity = oldCap * 2;
        table    = new Slot[capacity];
        count    = 0;
        for (int i = 0; i < oldCap; i++)
            if (oldTable[i].state == OCCUPIED)
                insert(oldTable[i].key, oldTable[i].value);
        delete[] oldTable;
    }

public:
    // ── Constructor / Destructor ──────────────────────────────
    HashMap(int initialCapacity = 256)
        : capacity(initialCapacity), count(0) {
        table = new Slot[capacity];
    }

    ~HashMap() { delete[] table; }

    HashMap(const HashMap& other)
        : capacity(other.capacity), count(other.count) {
        table = new Slot[capacity];
        for (int i = 0; i < capacity; i++)
            table[i] = other.table[i];
    }

    HashMap& operator=(const HashMap& other) {
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
    // Insert or update — O(1) amortised
    void insert(int key, int value) {
        if ((float)(count + 1) / capacity > LOAD_FACTOR)
            rehash();

        int idx = hash(key);
        while (table[idx].state == OCCUPIED) {
            if (table[idx].key == key) {
                table[idx].value = value;  // update existing
                return;
            }
            idx = (idx + 1) % capacity;
        }
        table[idx].key   = key;
        table[idx].value = value;
        table[idx].state = OCCUPIED;
        count++;
    }

    // Returns value, throws if not found — O(1) amortised
    int get(int key) const {
        int idx   = hash(key);
        int start = idx;
        while (table[idx].state != EMPTY) {
            if (table[idx].state == OCCUPIED && table[idx].key == key)
                return table[idx].value;
            idx = (idx + 1) % capacity;
            if (idx == start) break;
        }
        throw "HashMap::get — key not found";
    }

    // Safe get: writes to out, returns false if missing — O(1)
    bool tryGet(int key, int& out) const {
        int idx   = hash(key);
        int start = idx;
        while (table[idx].state != EMPTY) {
            if (table[idx].state == OCCUPIED && table[idx].key == key) {
                out = table[idx].value;
                return true;
            }
            idx = (idx + 1) % capacity;
            if (idx == start) break;
        }
        return false;
    }

    bool containsKey(int key) const {
        int dummy;
        return tryGet(key, dummy);
    }

    // Tombstone delete — O(1) amortised
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

    // ── Float value variants ──────────────────────────────────
    // The gCost map needs float values — store as bit-cast int
    void insertFloat(int key, float value) {
        // bit-cast float → int safely via union
        union { float f; int i; } u;
        u.f = value;
        insert(key, u.i);
    }

    float getFloat(int key) const {
        union { float f; int i; } u;
        u.i = get(key);
        return u.f;
    }

    bool tryGetFloat(int key, float& out) const {
        int raw;
        if (!tryGet(key, raw)) return false;
        union { float f; int i; } u;
        u.i = raw;
        out = u.f;
        return true;
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

const float HashMap::LOAD_FACTOR = 0.7f;
