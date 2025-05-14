/*
 * hashset_itr.h
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
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
 *  Also, changed the logic to iterate using just hashset_iterator_next().
 */

#ifndef HASHSET_ITR_H_
#define HASHSET_ITR_H_

#include "hashset.h"

#ifdef __cplusplus
extern "C" {
#endif

struct hashset_itr_st {
  hashset_t set;
  int index;
};

typedef struct hashset_itr_st* hashset_itr_t;

/**
 * @brief Create a hashset iterator.
 * The caller should then iterate over the hashset as follows:
 * ```
 *   hashset_itr_t iterator = hashset_iterator(my_hashset);
 *   while (hashset_iterator_next(iterator) >= 0) {
 *     void* my_value = hashset_iterator_value(iterator);
 *     ...
 *   }
 *   free(iterator);
 * ```
 * The caller must call free() on this iterator after using it.
 */
hashset_itr_t hashset_iterator(hashset_t set);

/**
 * @brief Returns the value at the current index.
 * The called should check hashset_iterator_has_next before calling this.
 */
void* hashset_iterator_value(hashset_itr_t itr);

/**
 * @brief Return 1 if there is a next value in the hashset and 0 otherwise.
 */
int hashset_iterator_has_next(hashset_itr_t itr);

/**
 * @brief Advance to the next value in the hashset.
 * This returns a non-negative number (the current index) if there is a next item
 * and -1 otherwise.
 */
int hashset_iterator_next(hashset_itr_t itr);

#endif /* HASHSET_ITR_H_ */

#ifdef __cplusplus
}
#endif
