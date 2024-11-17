#ifndef HELPER_TEMPLATE_MACROS
#define HELPER_TEMPLATE_MACROS

#define I_BASE_CONCAT(x,y) x ## y
#define II_BASE_CONCAT(x,y) I_BASE_CONCAT(x,y)

#define kinepy_type(NAME) II_BASE_CONCAT(Kp ## NAME, type_suffix)
#define kinepy_internal(NAME) II_BASE_CONCAT(NAME, type_suffix)
#define kinepy_interface(NAME) II_BASE_CONCAT(kp_ ## NAME, type_suffix)

#endif // HELPER_TEMPLATE_MACROS

#define UnitSystem kinepy_type(UnitSystem)
#define SolidDescriptionArray kinepy_type(SolidDescriptionArray)
#define JointDescriptionArray kinepy_type(JointDescriptionArray)
#define RelationDescriptionArray kinepy_type(RelationDescriptionArray)
#define System kinepy_type(System)

#define allocate_system kinepy_interface(allocate_system)
#define free_system kinepy_interface(free_system)
#define update_solid_physics kinepy_interface(update_solid_physics)

#define update_revolute_constraints kinepy_interface(update_revolute_constraints)
#define update_prismatic_constraints kinepy_interface(update_prismatic_constraints)

#define update_relation_parameters kinepy_interface(update_relation_parameters)

#define determine_computation_order kinepy_interface(determine_computation_order)

#define get_v0_unit kinepy_internal(get_v0_unit)
#define get_ratio_unit kinepy_internal(get_ratio_unit)
