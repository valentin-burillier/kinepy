#include "internal/util_macros.h"

#pragma region Types

#define System kinepy_type(System)
#define UnitSystem kinepy_type(UnitSystem)
#define Result kinepy_type(Result)

#pragma endregion

#pragma region Public Functions

#define allocate_system kinepy_interface(allocate_system)
#define free_system kinepy_interface(free_system)
#define update_solid_physics kinepy_interface(update_solid_physics)
#define update_revolute_constraints kinepy_interface(update_revolute_constraints)
#define update_prismatic_constraints kinepy_interface(update_prismatic_constraints)
#define update_relation_parameters kinepy_interface(update_relation_parameters)
#define allocate_result kinepy_interface(allocate_result)
#define free_result kinepy_interface(free_result)

#pragma endregion

#pragma region Private Functions

#define get_ratio_unit kinepy_internal(get_ratio_unit)
#define get_v0_unit kinepy_internal(get_v0_unit)

#pragma endregion