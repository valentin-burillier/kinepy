#ifndef KINEMATICS_KINEMATICS_H

// TODO: remove that
#define USE_AVX

#ifdef USE_AVX
#include "immintrin.h"
#endif

#include "internal/define_names.h"

#define float_type float
#define type_suffix _s
#define avx_type __m256
#include "template/kinematics_template.h"
#undef avx_type
#undef float_type
#undef type_suffix

#define float_type double
#define type_suffix _d
#define avx_type __m256d
#include "template/kinematics_template.h"
#undef avx_type
#undef float_type
#undef type_suffix

#include "internal/undef_names.h"

#define KINEMATICS_KINEMATICS_H
#endif //KINEMATICS_KINEMATICS_H
