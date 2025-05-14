/* -*- Mode: C; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil -*- */
/*
 *     Copyright 2012 Couchbase, Inc.
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 *  Modified in 2022 by Edward A. Lee to conform to documentation standards.
 *  Also, changed so that hashset_create() takes an initial capacity argument.
 */

#ifndef HASHSET_H
#define HASHSET_H 1

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

struct hashset_st {
  size_t nbits;
  size_t mask;

  size_t capacity;
  void** items;
  size_t nitems;
  size_t n_deleted_items;
};

typedef struct hashset_st* hashset_t;

/**
 * @brief Create a hashset instance.
 * The returned value is a pointer.
 * The caller must call hashset_destroy() to free allocated memory.
 * @param nbits The log base 2 of the initial capacity of the hashset.
 */
hashset_t hashset_create(unsigned short nbits);

/**
 * @brief Destroy the hashset instance, freeing allocated memory.
 */
void hashset_destroy(hashset_t set);

/**
 * @brief Return the number of items in the hashset.
 */
size_t hashset_num_items(hashset_t set);

/**
 * @brief Add a pointer to the hashset.
 * Note that 0 and 1 are special values, meaning nil and deleted items.
 * This function will return -1 indicating error if you try to add 0 or 1.
 * This function may resize the hashset if it is approaching capacity.
 * Returns zero if the item is already in the set and non-zero otherwise.
 */
int hashset_add(hashset_t set, void* item);

/**
 * @brief Remove an item from the hashset.
 * Return non-zero if the item was removed and zero if the item
 * is not on the hashset.
 */
int hashset_remove(hashset_t set, void* item);

/**
 * @brief Returns non-zero if the item is in the hashset and zero otherwise.
 */
int hashset_is_member(hashset_t set, void* item);

#ifdef __cplusplus
}
#endif

#endif
