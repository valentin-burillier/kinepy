#ifndef UTIL_MACROS_H
#define UTIL_MACROS_H

#include "string.h"

#define I_BASE_CONCAT(x,y) x ## y
#define II_BASE_CONCAT(x,y) I_BASE_CONCAT(x,y)

#define kinepy_type(NAME) II_BASE_CONCAT(Kp ## NAME, type_suffix)
#define kinepy_internal(NAME) II_BASE_CONCAT(NAME, type_suffix)
#define kinepy_interface(NAME) II_BASE_CONCAT(kp_ ## NAME, type_suffix)
#define avx(NAME) II_BASE_CONCAT(_mm256_ ## NAME, avx_suffix)
#define math(NAME)  II_BASE_CONCAT(NAME, math_suffix)

#define allocate_array(NAME, count) NAME = malloc((count) * sizeof(*(NAME))); if (!NAME)
#define allocate_array_jump(NAME, count) NAME = malloc((count) * sizeof(*(NAME))); if (!NAME) {goto malloc_err;} ++allocated; memset((NAME), 0x00, (count) * sizeof(*(NAME)))

#ifdef USE_AVX
#include "immintrin.h"
#define allocate_result_array_jump(NAME, count) NAME = _mm_malloc((count) * sizeof(*(NAME)), 0x20); if (!NAME) {goto malloc_err;} ++allocated; memset((NAME), 0x00, (count) * sizeof(*(NAME)))
#define free_result_array _mm_free
#else
#define allocate_result_array_jump allocate_array_jump
#define free_result_array free
#endif


#endif //UTIL_MACROS_H
