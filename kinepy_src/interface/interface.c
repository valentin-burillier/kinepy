#include "interface.h"

#include "internal/define_names.h"

#define float_type float
#define type_suffix _s
#define trig(NAME) NAME ## f
#include "interface/templates/interface_template.c"
#undef trig
#undef float_type
#undef type_suffix

#define float_type double
#define type_suffix _d
#define trig(NAME) NAME
#include "interface/templates/interface_template.c"
#undef trig
#undef float_type
#undef type_suffix

#include "internal/undef_names.h"