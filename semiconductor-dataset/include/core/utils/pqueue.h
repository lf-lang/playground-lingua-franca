/**
 * @file pqueue.h
 * @author Marten Lohstroh
 * @author Edward A. Lee
 * @author Byeonggil Jun
 * @copyright (c) 2020-2023, The University of California at Berkeley.
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 *
 * @brief Priority queue definitions for queues where the priority is a number that can be compared with ordinary
 * numerical comparisons.
 *
 * This is used for the reaction queue. The event queue uses a `tag_t` struct for its priority, so it cannot use this.
 */

#ifndef PQUEUE_H
#define PQUEUE_H

#include "pqueue_base.h"

/**
 * Return 1 if the first argument is greater than the second and zero otherwise.
 * @param thiz First argument.
 * @param that Second argument.
 */
int in_reverse_order(pqueue_pri_t thiz, pqueue_pri_t that);

/**
 * Return 0 regardless of argument order.
 * @param thiz First argument.
 * @param that Second argument.
 */
int in_no_particular_order(pqueue_pri_t thiz, pqueue_pri_t that);

/**
 * Return 1 if the two arguments are identical pointers.
 * @param a First argument.
 * @param b Second argument.
 */
int reaction_matches(void* a, void* b);

/**
 * Report a priority equal to the index of the given reaction.
 * Used for sorting pointers to reaction_t structs in the
 * blocked and executing queues.
 * @param reaction A pointer to a reaction_t.
 */
pqueue_pri_t get_reaction_index(void* reaction_t);

/**
 * Return the given reaction's position in the queue.
 * @param reaction A pointer to a reaction_t.
 */
size_t get_reaction_position(void* reaction);

/**
 * Set the given reaction's position in the queue.
 * @param event A pointer to a reaction_t.
 * @param pos The position.
 */
void set_reaction_position(void* reaction, size_t pos);

/**
 * Print some information about the given reaction.
 * This only prints something if logging is set to DEBUG.
 * @param reaction A pointer to a reaction_t.
 */
void print_reaction(void* reaction);

#endif /* PQUEUE_H */
