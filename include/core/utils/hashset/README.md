# hashset.c

A hash set implementation in C from:
https://github.com/avsej/hashset.c

The corresponding .c files are in
reactor-c/core/utils/hashset

## Example


    #include "hashset.h"

    char *foo = "foo";
    char *missing = "missing";
    hashset_t set = hashset_create(3);

    if (set == NULL) {
    	fprintf(stderr, "failed to create hashset instance\n");
    	abort();
    }

    hashset_add(set, foo);
    assert(hashset_is_member(set, foo) == 1);
    assert(hashset_is_member(set, missing) == 0);
