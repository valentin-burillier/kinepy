#ifndef HELPER_TEMPLATE_MACROS
#define HELPER_TEMPLATE_MACROS

#define I_BASE_CONCAT(x,y) x ## y
#define II_BASE_CONCAT(x,y) I_BASE_CONCAT(x,y)

#define kinepy_type(NAME) II_BASE_CONCAT(Kp ## NAME, type_suffix)
#define internal_type(NAME) II_BASE_CONCAT(iii_ ## NAME, type_suffix)
#define kinepy_internal(NAME) II_BASE_CONCAT(NAME, type_suffix)
#define kinepy_interface(NAME) II_BASE_CONCAT(kp_ ## NAME, type_suffix)

#endif // HELPER_TEMPLATE_MACROS

#define UnitSystem kinepy_type(UnitSystem)
#define SolidDescription kinepy_type(SolidDescription)
#define SolidDescriptionArray kinepy_type(SolidDescriptionArray)
#define JointDescription kinepy_type(JointDescription)
#define JointDescriptionArray kinepy_type(JointDescriptionArray)
#define RelationDescription kinepy_type(RelationDescription)
#define RelationDescriptionArray kinepy_type(RelationDescriptionArray)
#define System kinepy_type(System)

#define allocate_solid_descriptions kinepy_interface(allocate_solid_descriptions)
#define free_solid_descriptions kinepy_interface(free_solid_descriptions)
#define get_solid_description kinepy_interface(get_solid_description)
#define set_solid_description kinepy_interface(set_solid_description)

#define allocate_joint_descriptions kinepy_interface(allocate_joint_descriptions)
#define free_joint_descriptions kinepy_interface(free_joint_descriptions)
#define get_joint_description kinepy_interface(get_joint_description)
#define set_joint_description kinepy_interface(set_joint_description)
#define update_revolute_constraints kinepy_interface(update_revolute_constraints)
#define update_prismatic_constraints kinepy_interface(update_prismatic_constraints)

#define allocate_relation_descriptions kinepy_interface(allocate_relation_descriptions)
#define free_relation_descriptions kinepy_interface(free_relation_descriptions)
#define get_relation_description kinepy_interface(get_relation_description)
#define set_relation_description kinepy_interface(set_relation_description)
#define update_relation_parameters kinepy_interface(update_relation_parameters)

#define determine_computation_order kinepy_interface(determine_computation_order)

#define relation_parameters internal_type(relation_parameters)
#define joint_constraints internal_type(joint_constraints)

#define is_existing_solid kinepy_internal(is_existing_solid)
#define is_existing_joint_type kinepy_internal(is_existing_joint_type)
#define get_v0_unit kinepy_internal(get_v0_unit)
#define get_ratio_unit kinepy_internal(get_ratio_unit)
#define is_existing_joint kinepy_internal(is_existing_joint)
#define is_existing_relation_type kinepy_internal(is_existing_relation_type)
#define is_valid_relation kinepy_internal(is_valid_relation)
