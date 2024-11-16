#ifndef INTERFACE_INTERFACE_H

#include "structs.h"

#include "internal/define_names.h"

#define float_type float
#define type_suffix _s
#include "templates/interface_template.h"
#undef float_type
#undef type_suffix

#define float_type double
#define type_suffix _d
#include "templates/interface_template.h"
#undef float_type
#undef type_suffix

#include "internal/undef_names.h"

#define INTERFACE_INTERFACE_H
#endif //INTERFACE_INTERFACE_H
