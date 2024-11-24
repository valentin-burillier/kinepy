#ifndef HELPER_TEMPLATE_MACROS
#define HELPER_TEMPLATE_MACROS

#define I_BASE_CONCAT(x,y) x ## y
#define II_BASE_CONCAT(x,y) I_BASE_CONCAT(x,y)

#define kinepy_type(NAME) II_BASE_CONCAT(Kp ## NAME, type_suffix)
#define kinepy_internal(NAME) II_BASE_CONCAT(NAME, type_suffix)
#define kinepy_interface(NAME) II_BASE_CONCAT(kp_ ## NAME, type_suffix)
#define avx(NAME) II_BASE_CONCAT(_mm256_ ## NAME, avx_suffix)

#endif // HELPER_TEMPLATE_MACROS

#define UnitSystem kinepy_type(UnitSystem)
#define SolidDescriptionArray kinepy_type(SolidDescriptionArray)
#define JointDescriptionArray kinepy_type(JointDescriptionArray)
#define RelationDescriptionArray kinepy_type(RelationDescriptionArray)
#define System kinepy_type(System)
#define Result kinepy_type(Result)
#define PointDescription kinepy_internal(PointDescription)
#define allocate_system kinepy_interface(allocate_system)
#define free_system kinepy_interface(free_system)
#define update_solid_physics kinepy_interface(update_solid_physics)
#define update_revolute_constraints kinepy_interface(update_revolute_constraints)
#define update_prismatic_constraints kinepy_interface(update_prismatic_constraints)
#define update_relation_parameters kinepy_interface(update_relation_parameters)
#define allocate_result kinepy_interface(allocate_result)
#define free_result kinepy_interface(free_result)
#define determine_computation_order kinepy_interface(determine_computation_order)
#define get_v0_unit kinepy_internal(get_v0_unit)
#define get_ratio_unit kinepy_internal(get_ratio_unit)


#define solve_graph_rrr kinepy_internal(solve_graph_rrr)
#define solve_graph_rrp kinepy_internal(solve_graph_rrp)
#define solve_graph_ppr kinepy_internal(solve_graph_ppr)

#define complex_product_avx kinepy_internal(complex_product_avx)
#define dot_det_avx kinepy_internal(dot_det_avx)

#define compute_solid_point_avx kinepy_internal(compute_solid_point_avx)
#define compute_solid_point kinepy_internal(compute_solid_point)
#define compute_solid_point_diff_avx kinepy_internal(compute_solid_point_diff_avx)
#define compute_solid_point_diff kinepy_internal(compute_solid_point_diff)
#define load_point kinepy_internal(load_point)
#define move_eq_to_point kinepy_internal(move_eq_to_point)
#define place_and_rotate_eq kinepy_internal(place_and_rotate_eq)
#define paste_rr kinepy_internal(paste_rr)
