/*
 * Copyright (c) 2014, Volkan Yazıcı <volkan.yazici@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified by Marten Lohstroh (May, 2019).
 * Changes:
 * - Require implementation of a pqueue_eq_elem_f function to determine
 *   whether two elements are equal or not; and
 * - The provided pqueue_eq_elem_f implementation is used to test and
 *   search for equal elements present in the queue; and
 * - Removed capability to reassign priorities.
 *
 * Modified by Byeonggil Jun (Apr, 2024).
 * Changes:
 * - Made the pqueue_cmp_pri_f function return do the three-way comparison
 *   rather than the two-way comparison.
 * - The changed pqueue_cmp_pri_f function is used to check the equality of
 *   two elements in the pqueue_find_equal_same_priority function.
 * - Remove the pqueue_find_equal function.
 *
 * @brief Priority Queue function declarations used as a base for Lingua Franca priority queues.
 *
 * @{
 */

#ifndef PQUEUE_BASE_H
#define PQUEUE_BASE_H

#include <stddef.h>

/** Priority data type. */
typedef unsigned long long pqueue_pri_t;

/** Callback to get the priority of an element. */
typedef pqueue_pri_t (*pqueue_get_pri_f)(void* a);

/** Callback to compare two priorities. */
typedef int (*pqueue_cmp_pri_f)(pqueue_pri_t next, pqueue_pri_t curr);

/** Callback to determine whether two elements are equivalent. */
typedef int (*pqueue_eq_elem_f)(void* next, void* curr);

/** Callback functions to get the position of an element. */
typedef size_t (*pqueue_get_pos_f)(void* a);

/** Callback functions to set the position of an element. */
typedef void (*pqueue_set_pos_f)(void* a, size_t pos);

/** Debug callback function to print a entry. */
typedef void (*pqueue_print_entry_f)(void* a);

/** The priority queue handle. */
typedef struct pqueue_t {
  size_t size;              /**< number of elements in this queue plus 1 */
  size_t avail;             /**< slots available in this queue */
  size_t step;              /**< growth stepping setting */
  pqueue_cmp_pri_f cmppri;  /**< callback to compare priorities */
  pqueue_get_pri_f getpri;  /**< callback to get priority of a node */
  pqueue_get_pos_f getpos;  /**< callback to get position of a node */
  pqueue_set_pos_f setpos;  /**< callback to set position of a node */
  pqueue_eq_elem_f eqelem;  /**< callback to compare elements */
  pqueue_print_entry_f prt; /**< callback to print elements */
  void** d;                 /**< The actual queue in binary heap form */
} pqueue_t;

/**
 * @brief Allocate and initialize a priority queue.
 *
 * @param n the initial estimate of the number of queue items for which memory
 *     should be preallocated
 * @param cmppri The callback function to run to compare two elements
 *     This callback should return -1 for 'lower', 0 for 'same', and 1
 *     for 'higher', or vice versa if reverse priority is desired
 * @param getpri the callback function to run to set a score to an element
 * @param getpos the callback function to get the current element's position
 * @param setpos the callback function to set the current element's position
 * @param eqelem the callback function to check equivalence of entries
 * @param prt the callback function to print an element
 *
 * @return The handle or NULL for insufficent memory.
 */
pqueue_t* pqueue_init(size_t n, pqueue_cmp_pri_f cmppri, pqueue_get_pri_f getpri, pqueue_get_pos_f getpos,
                      pqueue_set_pos_f setpos, pqueue_eq_elem_f eqelem, pqueue_print_entry_f prt);

/**
 * free all memory used by the queue
 * @param q the queue
 */
void pqueue_free(pqueue_t* q);

/**
 * return the size of the queue.
 * @param q the queue
 */
size_t pqueue_size(pqueue_t* q);

/**
 * Insert an element into the queue.
 * @param q the queue
 * @param e the element
 * @return 0 on success
 */
int pqueue_insert(pqueue_t* q, void* d);

/**
 * Pop the highest-ranking item from the queue.
 * @param q the queue
 * @return NULL on error, otherwise the entry
 */
void* pqueue_pop(pqueue_t* q);

/**
 * @brief Empty 'src' into 'dest'.
 *
 * As an optimization, this function might swap 'src' and 'dest'.
 *
 * @param dest The queue to fill up
 * @param src  The queue to empty
 */
void pqueue_empty_into(pqueue_t** dest, pqueue_t** src);

/**
 * Return an entry with the same priority as the specified entry or NULL if there is no such entry.
 * @param q the queue
 * @param e the entry to compare against
 * @return NULL if no matching event has been found, otherwise the entry
 */
void* pqueue_find_same_priority(pqueue_t* q, void* e);

/**
 * Return an entry with the same priority (determined by `cmppri`) that matches the supplied entry (determined
 * by `eqelem`) or `NULL` if there is no such entry.
 * @param q the queue
 * @param e the entry to compare against
 * @return NULL if no matching event has been found, otherwise the entry
 */
void* pqueue_find_equal_same_priority(pqueue_t* q, void* e);

/**
 * Remove an item from the queue.
 * @param q the queue
 * @param e the entry
 * @return 0 on success
 */
int pqueue_remove(pqueue_t* q, void* e);

/**
 * Access highest-ranking item without removing it.
 * @param q the queue
 * @return NULL on error, otherwise the entry
 */
void* pqueue_peek(pqueue_t* q);

/**
 * Print the contents of the queue.
 * @param q The queue.
 * @param print The callback function to print the entry or NULL to use the default.
 */
void pqueue_print(pqueue_t* q, pqueue_print_entry_f print);

/**
 * Dump the queue and it's internal structure.
 * @internal
 * debug function only
 * @param q the queue
 * @param the callback function to print the entry
 */
void pqueue_dump(pqueue_t* q, pqueue_print_entry_f print);

/**
 * Check that the all entries are in the right order, etc.
 * @internal
 * debug function only
 * @param q the queue
 */
int pqueue_is_valid(pqueue_t* q);

#endif /* PQUEUE_BASE_H */
/** @} */
