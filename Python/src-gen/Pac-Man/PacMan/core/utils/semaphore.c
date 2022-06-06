/* Semaphore utility for reactor C. */

/*************
Copyright (c) 2021, The University of Texas at Dallas.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/** 
 * Semaphore utility for reactor C.
 *  
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#include "semaphore.h"
#include "assert.h"

/**
 * @brief Create a new semaphore.
 * 
 * @param count The count to start with.
 * @return semaphore_t* Can be NULL on error.
 */
semaphore_t* lf_semaphore_new(int count) {
    semaphore_t* semaphore = (semaphore_t*)malloc(sizeof(semaphore_t));
    lf_cond_init(&semaphore->cond);
    lf_mutex_init(&semaphore->mutex);
    semaphore->count = count;
    return semaphore;
}

/**
 * @brief Release the 'semaphore' and add 'i' to its count.
 * 
 * @param semaphore Instance of a semaphore
 * @param i The count to add.
 */
void lf_semaphore_release(semaphore_t* semaphore, int i) {
    assert(semaphore != NULL);
    lf_mutex_lock(&semaphore->mutex);
    semaphore->count += i;
    lf_cond_broadcast(&semaphore->cond);
    lf_mutex_unlock(&semaphore->mutex);
}

/**
 * @brief Acquire the 'semaphore'. Will block if count is 0.
 * 
 * @param semaphore Instance of a semaphore.
 */
void lf_semaphore_acquire(semaphore_t* semaphore) {
    assert(semaphore != NULL);
    lf_mutex_lock(&semaphore->mutex);
    while (semaphore->count == 0) {
        lf_cond_wait(&semaphore->cond, &semaphore->mutex);
    }
    semaphore->count--;
    lf_mutex_unlock(&semaphore->mutex);
}

/**
 * @brief Wait on the 'semaphore' if count is 0.
 * 
 * @param semaphore Instance of a semaphore.
 */
void lf_semaphore_wait(semaphore_t* semaphore) {
    assert(semaphore != NULL);
    lf_mutex_lock(&semaphore->mutex);
    while (semaphore->count == 0) {
        lf_cond_wait(&semaphore->cond, &semaphore->mutex);
    }
    lf_mutex_unlock(&semaphore->mutex);
}

/**
 * @brief Destroy the 'semaphore'.
 * 
 * @param semaphore Instance of a semaphore.
 */
void lf_semaphore_destroy(semaphore_t* semaphore) {
    assert(semaphore != NULL);
    free(semaphore);
}