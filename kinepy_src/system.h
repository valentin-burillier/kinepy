#ifndef SYSTEM_H
#define SYSTEM_H

#include "pure_macro_madness/interface.h"

typedef struct System {
    SolidDescriptionArrayView solids;
    JointDescriptionArrayView joints;
    SolidResultArrayView solid_results;
    JointResultArrayView joint_results;
} System;


#endif //SYSTEM_H