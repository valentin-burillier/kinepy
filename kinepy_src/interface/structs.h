#ifndef INTERFACE_STRUCTS_H

#include "internal/define_names.h"

#define float_type float
#define type_suffix _s
#include "templates/structs_template.h"
#undef float_type
#undef type_suffix

#define float_type double
#define type_suffix _d
#include "templates/structs_template.h"
#undef float_type
#undef type_suffix

#include "internal/undef_names.h"

#define INTERFACE_STRUCTS_H
#endif //INTERFACE_STRUCTS_H
