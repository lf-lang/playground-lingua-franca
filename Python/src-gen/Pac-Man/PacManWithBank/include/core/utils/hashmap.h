/**
 * @author Peter Donovan (peterdonovan@berkeley.edu)
 * @brief Defines a hashmap type that maps void pointers to integers.
 *
 * See hashmap.c for documentation on how to declare other hashmap types.
 */

#define HASHMAP(token) hashmap_object2int ## _ ## token
#define K void*
#define V int
#define HASH_OF(key) (size_t) key
#include "hashmap.c"
#undef HASHMAP
#undef K
#undef V
#undef HASH_OF
