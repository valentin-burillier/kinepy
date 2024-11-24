
#pragma region Types
#undef UnitSystem
#undef System
#pragma endregion Types

#pragma region Interface functions
#undef allocate_system
#undef free_system

#undef update_solid_physics
#undef update_revolute_constraints
#undef update_prismatic_constraints
#undef update_relation_parameters

#undef allocate_result
#undef free_result

#pragma region Internal interface functions
#undef get_v0_unit
#undef get_ratio_unit
#pragma endregion Internal interface functions

#pragma endregion Interface functions

#undef determine_computation_order

#undef solve_graph_rrr
#undef solve_graph_rrp
#undef solve_graph_ppr
#undef compute_solid_point_avx
#undef compute_solid_point_diff_avx
#undef PointDescription

#undef complex_product_avx
#undef dot_det_avx
#undef compute_solid_point_avx
#undef compute_solid_point
#undef compute_solid_point_diff_avx
#undef compute_solid_point_diff
#undef load_point
#undef move_eq_to_point
#undef place_and_rotate_eq
#undef paste_rr