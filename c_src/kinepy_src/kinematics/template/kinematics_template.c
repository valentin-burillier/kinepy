#include "kinematics_template.h"
#include "math.h"


inline void complex_product_avx(avx_type const ax, avx_type const ay, avx_type const bx, avx_type const by, avx_type * const out_x, avx_type * const out_y) {
    *out_x = avx(mul_p)(ax, bx);
    *out_x = avx(fnmadd_p)(ay, by, *out_x);

    *out_y = avx(mul_p)(ay, bx);
    *out_y = avx(fmadd_p)(ax, by, *out_y);
}


/**
 * A * B_conjugate * inv_mag; inv_mag = 1 / |A * B|
 */
inline void dot_det_avx(avx_type const ax, avx_type const ay, avx_type const bx, avx_type const by, avx_type const inv_mag, avx_type * const out_x, avx_type * const out_y) {
    *out_x = avx(mul_p)(ax, bx);
    *out_x = avx(fmadd_p)(ay, by, *out_x);
    *out_x = avx(mul_p)(*out_x, inv_mag);

    *out_y = avx(mul_p)(ay, bx);
    *out_y = avx(fnmadd_p)(ax, by, *out_y);
    *out_y = avx(mul_p)(*out_y, inv_mag);
}

void compute_solid_point_avx(Result const * const result, uint32_t const frame_index, PointDescription const * const point, avx_type * const out_x, avx_type * const out_y) {
    avx_type const solid_orientation_x = avx(load_p)(result->solid_orientation_x + frame_index + point->solid_index * result->frame_count);
    avx_type const solid_orientation_y = avx(load_p)(result->solid_orientation_y + frame_index + point->solid_index * result->frame_count);

    *out_x = avx(load_p)(result->solid_x + frame_index + point->solid_index * result->frame_count);
    *out_x = avx(fmadd_p)(solid_orientation_x, avx(broadcast_s)(&point->px), *out_x);
    *out_x = avx(fmsub_p)(solid_orientation_y, avx(broadcast_s)(&point->py), *out_x);

    *out_y = avx(load_p)(result->solid_y + frame_index + point->solid_index * result->frame_count);
    *out_y = avx(fmadd_p)(solid_orientation_x, avx(broadcast_s)(&point->py), *out_y);
    *out_y = avx(fmadd_p)(solid_orientation_y, avx(broadcast_s)(&point->px), *out_y);
}

void compute_solid_point(Result const * const result, uint32_t const frame_index, PointDescription const * const point, float_type * const out_x, float_type * const out_y) {
    float_type const solid_orientation_x = *(result->solid_orientation_x + frame_index + point->solid_index * result->frame_count);
    float_type const solid_orientation_y = *(result->solid_orientation_y + frame_index + point->solid_index * result->frame_count);

    *out_x = *(result->solid_x + frame_index + point->solid_index * result->frame_count);
    *out_x += solid_orientation_x * point->px;
    *out_x -= solid_orientation_y * point->py;

    *out_y = *(result->solid_y + frame_index + point->solid_index * result->frame_count);
    *out_y += solid_orientation_x * point->py;
    *out_y += solid_orientation_y * point->px;
}


void compute_solid_point_diff_avx(Result const * const result, uint32_t const frame_index, PointDescription const * const point0, PointDescription const * const point1, avx_type * const out_x, avx_type * const out_y) {
    avx_type p_x0;
    avx_type p_y0;
    compute_solid_point_avx(result, frame_index, point0, &p_x0, &p_y0);
    avx_type p_x1;
    avx_type p_y1;
    compute_solid_point_avx(result, frame_index, point1, &p_x1, &p_y1);
    *out_x = avx(sub_p)(p_x0, p_x1);
    *out_y = avx(sub_p)(p_y0, p_y1);
}

void compute_solid_point_diff(Result const * const result, uint32_t const frame_index, PointDescription const * const point0, PointDescription const * const point1, float_type * const out_x, float_type * const out_y) {
    float_type p_x0;
    float_type p_y0;
    compute_solid_point(result, frame_index, point0, &p_x0, &p_y0);
    float_type p_x1;
    float_type p_y1;
    compute_solid_point(result, frame_index, point1, &p_x1, &p_y1);
    *out_x = p_x0 - p_x1;
    *out_y = p_y0 - p_y1;
}


void load_point(Result const * const result, uint32_t const start_index, uint32_t const end_index, PointDescription const * const point, float_type * const dest_x, float_type * const dest_y) {
    uint32_t frame_index = start_index;

#ifdef USE_AVX
    while (frame_index + avx_count <= end_index) {
        avx_type x; avx_type y;

        compute_solid_point_avx(result, frame_index, point, &x, &y);
        avx(store_p)(dest_x + frame_index, x);
        avx(store_p)(dest_y + frame_index, y);
        frame_index += avx_count;
    }
#endif
    while (frame_index < end_index) {
        float_type ox = result->solid_orientation_x[frame_index + result->frame_count * point->solid_index];
        float_type oy = result->solid_orientation_x[frame_index + result->frame_count * point->solid_index];
        dest_x[frame_index] = result->solid_x[frame_index + result->frame_count * point->solid_index] + ox * point->px - oy * point->py;
        dest_y[frame_index] = result->solid_y[frame_index + result->frame_count * point->solid_index] + ox * point->py + oy * point->px;
        ++frame_index;
    }

}


void move_eq_to_point(Result const * const result, uint32_t const start_index, uint32_t const end_index, float_type const * const point_x, float_type const * const point_y, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    for (uint32_t const * solid_index_ptr = eq_ptr; solid_index_ptr < eq_ptr + eq_size; ++solid_index_ptr) {
        uint32_t frame_index = start_index;
        uint32_t const result_index = *solid_index_ptr * result->frame_count;

#ifdef USE_AVX
        while (frame_index + avx_count <= end_index) {
            avx(store_p)(result->solid_x + result_index + frame_index, avx(sub_p)(avx(load_p)(result->solid_x + result_index + frame_index), avx(load_p)(point_x + frame_index)));
            avx(store_p)(result->solid_y + result_index + frame_index, avx(sub_p)(avx(load_p)(result->solid_y + result_index + frame_index), avx(load_p)(point_y + frame_index)));

            frame_index += avx_count;
        }
#endif
        while (frame_index < end_index) {
            *(result->solid_x + result_index + frame_index) -= *(point_x + frame_index);
            *(result->solid_y + result_index + frame_index) -= *(point_y + frame_index);
            ++frame_index;
        }
    }
}

void place_and_rotate_eq(Result const * const result, uint32_t const start_index, uint32_t const end_index, float_type const * const point_x, float_type const * const point_y, float_type const * const rx, float_type const * const ry, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    for (uint32_t const * solid_index_ptr = eq_ptr; solid_index_ptr < eq_ptr + eq_size; ++solid_index_ptr) {
        uint32_t frame_index = start_index;
        uint32_t const result_index = *solid_index_ptr * result->frame_count;

#ifdef USE_AVX
        while (frame_index + avx_count <= end_index) {
            avx(store_p)(result->solid_x + result_index + frame_index, avx(add_p)(avx(load_p)(result->solid_x + result_index + frame_index), avx(load_p)(point_x + frame_index)));
            avx(store_p)(result->solid_y + result_index + frame_index, avx(add_p)(avx(load_p)(result->solid_y + result_index + frame_index), avx(load_p)(point_y + frame_index)));

            avx_type new_rx; avx_type new_ry;
            complex_product_avx(avx(load_p)(result->solid_orientation_x + result_index + frame_index), avx(load_p)(result->solid_orientation_y + result_index + frame_index), avx(load_p)(rx + frame_index), avx(load_p)(ry + frame_index), &new_rx, &new_ry);
            avx(store_p)(result->solid_orientation_x + result_index + frame_index, new_rx);
            avx(store_p)(result->solid_orientation_y + result_index + frame_index, new_ry);

            frame_index += avx_count;
        }
#endif
        while (frame_index < end_index) {
            *(result->solid_x + result_index + frame_index) += *(point_x + frame_index);
            *(result->solid_y + result_index + frame_index) += *(point_y + frame_index);

            float_type const new_rx = *(result->solid_orientation_x + result_index + frame_index) * rx[frame_index] - *(result->solid_orientation_y + result_index + frame_index) * ry[frame_index];
            float_type const new_ry = *(result->solid_orientation_y + result_index + frame_index) * rx[frame_index] + *(result->solid_orientation_x + result_index + frame_index) * ry[frame_index];

            *(result->solid_orientation_x + result_index + frame_index) = new_rx;
            *(result->solid_orientation_y + result_index + frame_index) = new_ry;

            ++frame_index;
        }
    }
}



void paste_rr(Result const * const result, uint32_t const start_index, uint32_t const end_index, PointDescription const * const target_point, PointDescription const * const second_point, PointDescription const * const eq_point, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    uint32_t frame_index = start_index;
#ifdef USE_AVX
    while (frame_index + avx_count <= end_index) {
        avx_type target_x; avx_type target_y;
        compute_solid_point_avx(result, frame_index, target_point, &target_x, &target_y);
        avx(store_p)(result->_temp_arrays[0] + frame_index, target_x);
        avx(store_p)(result->_temp_arrays[1] + frame_index, target_y);

        avx_type second_x; avx_type second_y;
        compute_solid_point_avx(result, frame_index, second_point, &second_x, &second_y);

        target_x = avx(sub_p)(second_x, target_x);
        target_y = avx(sub_p)(second_y, target_y);
        float_type const one = 1.0;
        avx_type inv_sq_mag = avx(div_p)(avx(broadcast_s)(&one), avx(add_p)(avx(mul_p)(target_x, target_x), avx(mul_p)(target_y, target_y)));

        avx_type current_x; avx_type current_y;
        compute_solid_point_avx(result, frame_index, eq_point, &current_x, &current_y);

        avx_type rx; avx_type ry;
        dot_det_avx(target_x, target_y, current_x, current_y, inv_sq_mag, &rx, &ry);
        avx(store_p)(result->_temp_arrays[2] + frame_index, rx);
        avx(store_p)(result->_temp_arrays[3] + frame_index, ry);

        frame_index += avx_count;
    }
#endif
    while (frame_index < end_index) {
        float_type target_x; float_type target_y;
        compute_solid_point(result, frame_index, target_point, &target_x, &target_y);
        *(result->_temp_arrays[0] + frame_index) = target_x;
        *(result->_temp_arrays[1] + frame_index) = target_y;

        float_type second_x; float_type second_y;
        compute_solid_point(result, frame_index, second_point, &second_x, &second_y);

        target_x = second_x - target_x;
        target_y = second_y - target_y;

        float_type inv_sq_mag = 1.0 / (target_x * target_x + target_y * target_y);

        float_type current_x; float_type current_y;
        compute_solid_point(result, frame_index, eq_point, &current_x, &current_y);

        *(result->_temp_arrays[2] + frame_index) = (target_x * current_x + target_y * current_y) * inv_sq_mag;
        *(result->_temp_arrays[3] + frame_index) = (target_y * current_x - target_x * current_y) * inv_sq_mag;

        ++frame_index;
    }

    place_and_rotate_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], result->_temp_arrays[2], result->_temp_arrays[3], eq_size, eq_ptr);
}

/*
         0
        / \
       R0  R1
      /     \
     1 - R2- 2
*/

void solve_graph_rrr(System const * const system, ResolutionStep const * const step, Result * const result, uint32_t const start_index, uint32_t const end_index) {

    GraphStep const * const graph_step = &step->graph_step;

    typeof(graph_step->edges) const r0 = graph_step->edges + 0;
    typeof(graph_step->edges) const r1 = graph_step->edges + 1;
    typeof(graph_step->edges) const r2 = graph_step->edges + 2;

    typeof(system->config.joints) solid_ptr_s[] = {
        (typeof(system->config.joints)) &system->config.joints->solid1,
        (typeof(system->config.joints)) &system->config.joints->solid2,
    };
    typeof(system->joint_parameters_ptr) param_ptr_s[] = {
        (typeof(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x1,
        (typeof(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x2
    };

    /* p{eq}{joint} on graph */
    PointDescription const p00 = {
        .solid_index = *(uint32_t*)(solid_ptr_s[!r0->orientation] + r0->joint_index),
        .px = *((float_type*)(param_ptr_s[!r0->orientation] + r0->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[!r0->orientation] + r0->joint_index) + 1)
    };

    PointDescription const p10 = {
        .solid_index = *(uint32_t*)(solid_ptr_s[r0->orientation] + r0->joint_index),
        .px = *((float_type*)(param_ptr_s[r0->orientation] + r0->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[r0->orientation] + r0->joint_index) + 1)
    };

    PointDescription const p01 =  {
       .solid_index = *(uint32_t*)(solid_ptr_s[!r1->orientation] + r1->joint_index),
       .px = *((float_type*)(param_ptr_s[!r1->orientation] + r1->joint_index) + 0),
       .py = *((float_type*)(param_ptr_s[!r1->orientation] + r1->joint_index) + 1)
    };

    PointDescription const p21 =  {
        .solid_index = *(uint32_t*)(solid_ptr_s[r1->orientation] + r1->joint_index),
        .px = *((float_type*)(param_ptr_s[r1->orientation] + r1->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[r1->orientation] + r1->joint_index) + 1)
    };

    PointDescription const p12 =  {
        .solid_index = *(uint32_t*)(solid_ptr_s[!r2->orientation] + r2->joint_index),
        .px = *((float_type*)(param_ptr_s[!r2->orientation] + r2->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[!r2->orientation] + r2->joint_index) + 1)
    };

    PointDescription const p22 =  {
        .solid_index = *(uint32_t*)(solid_ptr_s[r2->orientation] + r2->joint_index),
        .px = *((float_type*)(param_ptr_s[r2->orientation] + r2->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[r2->orientation] + r2->joint_index) + 1)
    };


    load_point(result, start_index, end_index, &p10, result->_temp_arrays[0], result->_temp_arrays[1]);
    move_eq_to_point(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);

    load_point(result, start_index, end_index, &p21, result->_temp_arrays[0], result->_temp_arrays[1]);
    move_eq_to_point(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);


    float_type const signs[] = {
        1.,
        -1.
    };
    float_type const sign = signs[graph_step->solution_index];
#ifdef USE_AVX
    uint32_t frame_index = start_index;
    while (frame_index + avx_count <= end_index) {
        avx_type vec0_x; avx_type vec0_y;
        compute_solid_point_diff_avx(result, frame_index, &p01, &p00, &vec0_x, &vec0_y);
        avx_type sq_vec0 = avx(mul_p)(vec0_x, vec0_x);
        sq_vec0 = avx(fmadd_p)(vec0_y, vec0_y, sq_vec0);

        avx_type vec1_x; avx_type vec1_y;
        compute_solid_point_avx(result, frame_index, &p12, &vec1_x, &vec1_y);
        avx_type sq_vec1 = avx(mul_p)(vec1_x, vec1_x);
        sq_vec1 = avx(fmadd_p)(vec1_y, vec1_y, sq_vec1);

        avx_type vec2_x; avx_type vec2_y;
        compute_solid_point_avx(result, frame_index, &p22, &vec2_x, &vec2_y);
        avx_type sq_vec2 = avx(mul_p)(vec2_x, vec2_x);
        sq_vec2 = avx(fmadd_p)(vec2_y, vec2_y, sq_vec2);

        float_type const _one_half = 0.5;
        avx_type const one_half = avx(broadcast_s)(&_one_half);
        avx_type cos_angle = avx(add_p)(sq_vec0, sq_vec1);
        cos_angle = avx(sub_p)(cos_angle, sq_vec2);
        cos_angle = avx(mul_p)(cos_angle, one_half);

        float_type const one = 1.;
        avx_type inv_01 = avx(mul_p)(sq_vec0, sq_vec1);
        inv_01 = avx(sqrt_p)(inv_01);
        inv_01 = avx(div_p)(avx(broadcast_s)(&one), inv_01);
        cos_angle = avx(mul_p)(cos_angle, inv_01);

        avx_type sin_angle = avx(mul_p)(cos_angle, cos_angle);
        sin_angle = avx(sub_p)(avx(broadcast_s)(&one), sin_angle);
        sin_angle = avx(sqrt_p)(sin_angle);
        sin_angle = avx(mul_p)(sin_angle, avx(broadcast_s)(&sign));

        avx_type align_x; avx_type align_y;
        dot_det_avx(vec0_x, vec0_y, vec1_x, vec1_y, inv_01, &align_x, &align_y);

        avx_type rotation_x; avx_type rotation_y;
        complex_product_avx(cos_angle, sin_angle, align_x, align_y, &rotation_x, &rotation_y);

        avx(store_p)(result->_temp_arrays[2] + frame_index, rotation_x);
        avx(store_p)(result->_temp_arrays[3] + frame_index, rotation_y);

        frame_index += avx_count;
    }
#endif
    while (frame_index < end_index) {
        float_type vec0_x; float_type vec0_y;
        compute_solid_point_diff(result, frame_index, &p01, &p00, &vec0_x, &vec0_y);
        float_type sq_vec0 = vec0_x * vec0_x + vec0_y * vec0_y;

        float_type vec1_x; float_type vec1_y;
        compute_solid_point(result, frame_index, &p12, &vec1_x, &vec1_y);
        float_type sq_vec1 = vec1_x * vec1_x + vec1_y * vec1_y;

        float_type vec2_x; float_type vec2_y;
        compute_solid_point(result, frame_index, &p22, &vec2_x, &vec2_y);
        float_type sq_vec2 = vec2_x * vec2_x + vec2_y * vec2_y;

        float_type const inv_01 = 1.0 / trig(sqrt)(sq_vec0 * sq_vec1);
        float_type const cos_angle = 0.5 * (sq_vec0 + sq_vec1 - sq_vec2) * inv_01;
        float_type const sin_angle = trig(sqrt)(1. - cos_angle * cos_angle);

        float_type const align_x = (vec1_x * vec0_x + vec1_y * vec0_y) * inv_01;
        float_type const align_y = (vec1_y * vec0_x - vec1_x * vec0_y) * inv_01;

        *(result->_temp_arrays[2] + frame_index) = align_x * cos_angle - align_y * sin_angle;
        *(result->_temp_arrays[3] + frame_index) = align_x * sin_angle + align_y * cos_angle;

        ++frame_index;
    }

    load_point(result, start_index, end_index, &p00, result->_temp_arrays[0], result->_temp_arrays[1]);
    place_and_rotate_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], result->_temp_arrays[2], result->_temp_arrays[3], graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);

    paste_rr(result, start_index, end_index, &p01, &p12, &p22, graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);
}