#include "kinematics.h"
#include "internal/define_names.h"


#define float_type float
#define type_suffix _s
#define avx_type __m256
#define avx_suffix s
#define trig(NAME) NAME ## f
#define avx_count 8
#include "template/kinematics_template.c"
#undef avx_count
#undef trig
#undef avx_suffix
#undef avx_type
#undef float_type
#undef type_suffix

#define float_type double
#define type_suffix _d
#define avx_type __m256d
#define avx_suffix d
#define trig(NAME) NAME
#define avx_count 4
#include "template/kinematics_template.c"
#undef avx_count
#undef trig
#undef avx_suffix
#undef avx_type
#undef float_type
#undef type_suffix


#include "internal/undef_names.h"