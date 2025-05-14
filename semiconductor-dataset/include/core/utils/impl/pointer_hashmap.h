/**
 * @author Peter Donovan (peterdonovan@berkeley.edu)
 * @brief Defines a hashmap type that maps void pointers to integers.
 * To use this:
 * ```
 * #include "core/utils/impl/pointer_hashmap.h"
 * ```
 * To create a new hashmap:
 * ```
 * hashmap_object2int_t my_map = hashmap_object2int_new(CAPACITY, NOTHING);
 * ```
 * where CAPACITY is the maximum capacity of the hashmap and NOTHING is
 * a pointer key that is guaranteed to be never used (e.g., NULL).
 * To put an entry into the hashmap:
 * ```
 * hashmap_object2int_put(POINTER, VALUE);
 * ```
 * where POINTER is a pointer not equal to NOTHING and VALUE is an integer.
 * This will segfault if the hashmap is already at capacity.
 *
 * To retrieve a value from the hashmap:
 * ```
 * int value = hashmap_object2int_get(POINTER);
 * ```
 * This will segfault if the entry is not in the hashmap.
 *
 * See hashmap.h for documentation on how to declare other hashmap types.
 */

#define HASHMAP(token) hashmap_object2int##_##token
#define K void*
#define V int
#define HASH_OF(key) (size_t)key
#include "hashmap.h"
#undef HASHMAP
#undef K
#undef V
#undef HASH_OF
