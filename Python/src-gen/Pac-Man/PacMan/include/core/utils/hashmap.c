/**
 * @author Peter Donovan (peterdonovan@berkeley.edu)
 * @brief Defines a generic, non-resizing hashmap data type.
 *
 * Hashmaps are defined by redefining K, V, HASH_OF, and HASHMAP, and including this file. A default
 * hashmap type is defined in hashmap.h. See hashmap.h for an example of a hashmap declaration.
 * - K and V must be the types of keys and values of the hashmap, respectively.
 * - HASH_OF must be the hash of a key.
 * - HASHMAP must be a function-like macro that prefixes tokens with the name of the hashmap. For
 *   example, the name of the hashmap data type is given by evaluation of the macro HASHMAP(t) so
 *   that it is "t" prefixed with the name of the hashmap. The function names associated with the
 *   data type are similar.
 */

#ifndef K
#define K void*
#endif
#ifndef V
#define V void*
#endif
#ifndef HASH_OF
#define HASH_OF(key) (size_t) key
#endif
#ifndef HASHMAP
#define HASHMAP(token) hashmap ## _ ## token
#endif

#include <stddef.h>
#include <assert.h>
#include <stdbool.h>

////////////////////////// Type definitions ///////////////////////////

typedef struct HASHMAP(entry_t) {
    K key;
    V value;
} HASHMAP(entry_t);

typedef struct HASHMAP(t) {
    HASHMAP(entry_t)* entries;
    size_t capacity;
    size_t num_entries;
    K nothing;
} HASHMAP(t);

//////////////////////// Function declarations ////////////////////////

/**
 * @brief Construct a new hashmap object.
 * @param capacity A number that is much larger than the maximum number of items that this
 * hashmap will contain. Insufficient surplus capacity will cause poor performance.
 * @param nothing A key that is guaranteed never to be used.
 */
HASHMAP(t)* HASHMAP(new)(size_t capacity, K nothing);

/** @brief Free all memory used by the given hashmap. */
void HASHMAP(free)(HASHMAP(t)* hashmap);

/** @brief Associate a value with the given key. */
void HASHMAP(put)(HASHMAP(t)* hashmap, K key, V value);

/**
 * @brief Get the value associated with the given key.
 * Precondition: The key must be present in the map.
 */
V HASHMAP(get)(HASHMAP(t)* hashmap, K key);

/////////////////////////// Private helpers ///////////////////////////

static HASHMAP(entry_t)* HASHMAP(get_ideal_address)(HASHMAP(t)* hashmap, K key) {
    HASHMAP(entry_t)* address = hashmap->entries + (HASH_OF(key) % hashmap->capacity);
    assert(address >= hashmap->entries);
    assert(address < hashmap->entries + hashmap->capacity);
    return address;
}

/**
 * @brief Return the actual address of the hashmap entry corresponding to `key`, or the address of
 * the closest empty entry if no such entry exists.
 * @param key The key from which to begin a search.
 * @param desired The key that the desired returnable entry should have.
 */
static HASHMAP(entry_t)* HASHMAP(get_actual_address)(HASHMAP(t)* hashmap, K key) {
    HASHMAP(entry_t)* address = HASHMAP(get_ideal_address)(hashmap, key);
    HASHMAP(entry_t)* upper_limit = hashmap->entries + hashmap->capacity;
    while ((address->key != hashmap->nothing) & (address->key != key)) address++;
    if (address == upper_limit) {
        address = hashmap->entries;
        while ((address->key != hashmap->nothing) & (address->key != key)) address++;
        if (address == upper_limit) return NULL;
    }
    assert(address->key == key || address->key == hashmap->nothing);
    return address;
}

//////////////////////// Function definitions /////////////////////////

HASHMAP(t)* HASHMAP(new)(size_t capacity, K nothing) {
    HASHMAP(entry_t)* entries = (HASHMAP(entry_t)*) malloc(
        (capacity + 1) * sizeof(HASHMAP(entry_t))
    );
    if (!entries) exit(1);
    HASHMAP(t)* ret = (HASHMAP(t)*) malloc(sizeof(HASHMAP(t)));
    if (!ret) exit(1);
    // The entry at the end is used as a boundary. It will never again be written to.
    for (size_t i = 0; i < capacity + 1; i++) {
        entries[i].key = nothing;
    }
    ret->entries = entries;
    ret->capacity = capacity;
    ret->num_entries = 0;
    // A second nothing may be required if removal is to be supported and we want to make removal
    // a constant-time operation.
    ret->nothing = nothing;
    return ret;
}

void HASHMAP(free)(HASHMAP(t)* hashmap) {
    free(hashmap->entries);
    free(hashmap);
}

void HASHMAP(put)(HASHMAP(t)* hashmap, K key, V value) {
    assert(key != hashmap->nothing);
    assert(key >= 0);
    HASHMAP(entry_t)* write_to = HASHMAP(get_actual_address)(hashmap, key);
    write_to->key = key;
    write_to->value = value;
}

V HASHMAP(get)(HASHMAP(t)* hashmap, K key) {
    assert(key != hashmap->nothing);
    HASHMAP(entry_t)* read_from = HASHMAP(get_actual_address)(hashmap, key);
    return read_from->value; // Crash the program if the key cannot be found
}
