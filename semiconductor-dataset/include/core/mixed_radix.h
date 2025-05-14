/**
 * @file
 * @author Edward A. Lee
 *
 * @section LICENSE
Copyright (c) 2022, The University of California at Berkeley.

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

 @section DESCRIPTION
 @brief Header file for permuted mixed-radix numbers used in Lingua Franca programs.

 A mixed radix number is a number representation where each digit can have
 a distinct radix. The radixes are given by a list of numbers, r0, r1, ... , rn,
 where r0 is the radix of the lowest-order digit and rn is the radix of the
 highest order digit that has a specified radix.

 A permuted mixed radix number is a mixed radix number that, when incremented,
 increments the digits in the order given by the permutation matrix.
 For an ordinary mixed radix number, the permutation matrix is
 [0, 1, ..., n-1]. The permutation matrix may be any permutation of
 these digits, [d0, d1, ..., dn-1], in which case, when incremented,
 the d0 digit will be incremented first. If it overflows, it will be
 set to 0 and the d1 digit will be incremented. If it overflows, the
 next digit is incremented.  If the last digit overflows, then the
 number wraps around so that all digits become zero.

 The functions defined here are pretty limited and assume that the
 caller is well behaved. These functions are used in code generated
 by the Lingua-Franca compiler and are not intended to be used by
 end users. For example, there is very limited error checking and
 misuse of the functions is likely to result in assertion errors
 and/or segmentation faults that will cause the program to exit.

 To use these functions, you can create the arrays on the stack
 as follows:
 ```
        int range_start[] =  { 0, 0, 0 };
        int range_radixes[] = { 2, 3, 4 };
        int permutation[] = { 1, 0, 2 };
        mixed_radix_int_t x = {
            3,
            range_start,
            range_radixes,
            permutation
        };
 ```
 This will create a mixed-radix number where the low-order digit
 has radix 2 (value is either 0 or 1), the second digit has radix 3,
 and the third digit has radix 4. When you increment this number:
 ```
    mixed_radix_incr(&x);
 ```
 the second digit will be incremented first (because of the
 permutation matrix).  The return value for
 ```
    mixed_radix_to_int(&x);
 ```
 will be 2 (the second digit will be 1, and because the radix of
 the low-order digit is 2, the value of this digit is 2).
 The return value of
 ```
    mixed_radix_parent(&x, 1);
 ```
 will be 1 because, after dropping one digit, we will have a
 mixed radix number with value 1%3; 0%4 (radixes are 3 and 4
 and digits are 1 and 0).

 The above mixed-radix number has a total of 24 possible values.
 If you increment it 24 times, it will cover all possible value
 (albeit in a strange order because of the permutation) and return
 to the original value with digits 0, 0, 0.
 */

#ifndef MIXED_RADIX_H
#define MIXED_RADIX_H

/**
 * Representation of a permuted mixed radix integer.
 * The three arrays (digits, radixes, and permutation) are all
 * assumed to have the same size as given by the size field.
 */
typedef struct mixed_radix_int_t {
  int size;
  int* digits;
  int* radixes;
  int* permutation;
} mixed_radix_int_t;

/**
 * Increment the mixed radix number by one according to the permutation matrix.
 * @param mixed A pointer to the mixed-radix number.
 */
void mixed_radix_incr(mixed_radix_int_t* mixed);

/**
 * Return the int value of a mixed-radix number after dropping
 * the first n digits. If n is larger than or equal to the size
 * of the mixed-radix number, then return 0.
 * @param mixed A pointer to the mixed-radix number.
 * @param n The number of digits to drop, which is assumed to
 *  be greater than or equal to 0.
 */
int mixed_radix_parent(mixed_radix_int_t* mixed, int n);

/**
 * Return the int value of a mixed-radix number.
 * @param mixed A pointer to the mixed-radix number.
 */
int mixed_radix_to_int(mixed_radix_int_t* mixed);

#endif /* MIXED_RADIX_H */
