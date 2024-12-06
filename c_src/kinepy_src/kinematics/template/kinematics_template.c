#include "kinematics_template.h"
#include "immintrin.h"
#include "math.h"

#ifdef USE_AVX
void complex_product_avx(avx_type const ax, avx_type const ay, avx_type const bx, avx_type const by, avx_type * const out_x, avx_type * const out_y) {
   *out_x = avx(mul_p)(ax, bx);
   *out_x = avx(fnmadd_p)(ay, by, *out_x);

   *out_y = avx(mul_p)(ay, bx);
   *out_y = avx(fmadd_p)(ax, by, *out_y);
}

/**
 * A * B_conjugate * inv_mag; inv_mag = 1 / |A * B|
 */
void dot_det_avx(avx_type const ax, avx_type const ay, avx_type const bx, avx_type const by, avx_type const inv_mag, avx_type * const out_x, avx_type * const out_y) {
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
    *out_x = avx(fnmadd_p)(solid_orientation_y, avx(broadcast_s)(&point->py), *out_x);

    *out_y = avx(load_p)(result->solid_y + frame_index + point->solid_index * result->frame_count);
    *out_y = avx(fmadd_p)(solid_orientation_x, avx(broadcast_s)(&point->py), *out_y);
    *out_y = avx(fmadd_p)(solid_orientation_y, avx(broadcast_s)(&point->px), *out_y);
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
#endif

static inline void set_value(float_type * const ptr, uint32_t const start_index, uint32_t const end_index, float_type const value) {
    uint32_t frame_index = start_index;

    while (frame_index < end_index) {
        *(ptr + frame_index) = value;
        ++frame_index;
    }
}

void reset_results(KpConfiguration const * const configuration, Result * const result, uint32_t const start_index, uint32_t const end_index) {
    for (uint32_t solid_index = 0; solid_index < configuration->solid_count; ++solid_index) {
        uint32_t offset = solid_index * result->frame_count;
        set_value(result->solid_orientation_x, start_index + offset, end_index + offset, 1.0);
        set_value(result->solid_orientation_y, start_index + offset, end_index + offset, 0.0);

        set_value(result->solid_x, start_index + offset, end_index + offset, 0.0);
        set_value(result->solid_y, start_index + offset, end_index + offset, 0.0);
    }
}

static inline void compute_solid_point(Result const * const result, uint32_t const frame_index, PointDescription const * const point, float_type * const out_x, float_type * const out_y) {
    float_type const solid_orientation_x = *(result->solid_orientation_x + frame_index + point->solid_index * result->frame_count);
    float_type const solid_orientation_y = *(result->solid_orientation_y + frame_index + point->solid_index * result->frame_count);

    *out_x = *(result->solid_x + frame_index + point->solid_index * result->frame_count);
    *out_x += solid_orientation_x * point->px;
    *out_x -= solid_orientation_y * point->py;

    *out_y = *(result->solid_y + frame_index + point->solid_index * result->frame_count);
    *out_y += solid_orientation_x * point->py;
    *out_y += solid_orientation_y * point->px;
}

static inline void compute_prismatic_orientation(Result const * const result, uint32_t const frame_index, PointDescription const * const point, float_type * const out_x, float_type * const out_y) {
    float_type const solid_orientation_x = *(result->solid_orientation_x + frame_index + point->solid_index * result->frame_count);
    float_type const solid_orientation_y = *(result->solid_orientation_y + frame_index + point->solid_index * result->frame_count);

    *out_x = solid_orientation_x * point->px;
    *out_x -= solid_orientation_y * point->py;

    *out_y = solid_orientation_x * point->py;
    *out_y += solid_orientation_y * point->px;
}

static inline void move_to_prismatic_point(Result const * const result, uint32_t const frame_index, PointDescription const * const point, float_type * const out_x, float_type * const out_y) {
    *out_x += *(result->solid_x + frame_index + point->solid_index * result->frame_count);
    *out_y += *(result->solid_y + frame_index + point->solid_index * result->frame_count);
}



static inline void compute_solid_point_diff(Result const * const result, uint32_t const frame_index, PointDescription const * const point0, PointDescription const * const point1, float_type * const out_x, float_type * const out_y) {
    float_type p_x0;
    float_type p_y0;
    compute_solid_point(result, frame_index, point0, &p_x0, &p_y0);
    float_type p_x1;
    float_type p_y1;
    compute_solid_point(result, frame_index, point1, &p_x1, &p_y1);
    *out_x = p_x0 - p_x1;
    *out_y = p_y0 - p_y1;
}


static inline void load_point(Result const * const result, uint32_t const start_index, uint32_t const end_index, PointDescription const * const point, float_type * const dest_x, float_type * const dest_y) {
    uint32_t frame_index = start_index;

    while (frame_index < end_index) {
        float_type ox = result->solid_orientation_x[frame_index + result->frame_count * point->solid_index];
        float_type oy = result->solid_orientation_x[frame_index + result->frame_count * point->solid_index];
        dest_x[frame_index] = result->solid_x[frame_index + result->frame_count * point->solid_index] + ox * point->px - oy * point->py;
        dest_y[frame_index] = result->solid_y[frame_index + result->frame_count * point->solid_index] + ox * point->py + oy * point->px;
        ++frame_index;
    }

}


static inline void move_eq_to_point(Result const * const result, uint32_t const start_index, uint32_t const end_index, float_type const * const point_x, float_type const * const point_y, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    for (uint32_t const * solid_index_ptr = eq_ptr; solid_index_ptr < eq_ptr + eq_size; ++solid_index_ptr) {
        uint32_t frame_index = start_index;
        uint32_t const result_index = *solid_index_ptr * result->frame_count;

        while (frame_index < end_index) {
            *(result->solid_x + result_index + frame_index) -= *(point_x + frame_index);
            *(result->solid_y + result_index + frame_index) -= *(point_y + frame_index);
            ++frame_index;
        }
    }
}

static inline void rotate_eq(Result const * const result, uint32_t const start_index, uint32_t const end_index, float_type const * const rx, float_type const * const ry, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    for (uint32_t const * solid_index_ptr = eq_ptr; solid_index_ptr < eq_ptr + eq_size; ++solid_index_ptr) {
        uint32_t frame_index = start_index;
        uint32_t const result_index = *solid_index_ptr * result->frame_count;

        while (frame_index < end_index) {
            float_type const new_rx = *(result->solid_orientation_x + result_index + frame_index) * rx[frame_index] - *(result->solid_orientation_y + result_index + frame_index) * ry[frame_index];
            float_type const new_ry = *(result->solid_orientation_y + result_index + frame_index) * rx[frame_index] + *(result->solid_orientation_x + result_index + frame_index) * ry[frame_index];

            *(result->solid_orientation_x + result_index + frame_index) = new_rx;
            *(result->solid_orientation_y + result_index + frame_index) = new_ry;

            ++frame_index;
        }
    }
}
static inline void prismatic_alignment(Result const * const result, uint32_t const start_index, uint32_t const end_index, PointDescription const * const p1, PointDescription const * const p2, float_type * const out_x, float_type * const out_y) {
    uint32_t frame_index = start_index;
    while (frame_index < end_index) {
        float_type const solid_orientation_x1 = *(result->solid_orientation_x + frame_index + p1->solid_index * result->frame_count);
        float_type const solid_orientation_y1 = *(result->solid_orientation_y + frame_index + p1->solid_index * result->frame_count);
        float_type const total_x1 = solid_orientation_x1 * p1->px - solid_orientation_y1 * p1->py;
        float_type const total_y1 = solid_orientation_x1 * p1->py + solid_orientation_y1 * p1->px;

        float_type const solid_orientation_x2 = *(result->solid_orientation_x + frame_index + p2->solid_index * result->frame_count);
        float_type const solid_orientation_y2 = *(result->solid_orientation_y + frame_index + p2->solid_index * result->frame_count);
        float_type const total_x2 = solid_orientation_x2 * p2->px - solid_orientation_y2 * p2->py;
        float_type const total_y2 = solid_orientation_x2 * p2->py + solid_orientation_y2 * p2->px;

        float_type const mag = math(sqrt)((total_x1 * total_x1 + total_y1 * total_y1) * (total_x2 * total_x2 + total_y2 * total_y2));

        *(out_x + frame_index) = (total_x1 * total_x2 + total_y1 * total_y2) / mag;
        *(out_y + frame_index) = (total_y1 * total_x2 - total_y2 * total_x1) / mag;
        ++frame_index;
    }
}

static inline void align_prismatic_eq(Result const * const result, uint32_t const start_index, uint32_t const end_index, PointDescription const * const p1, PointDescription const * const p2, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    prismatic_alignment(result, start_index, end_index, p1, p2, result->_temp_arrays[0], result->_temp_arrays[1]);
    rotate_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], eq_size, eq_ptr);
}

static inline void place_eq(Result const * const result, uint32_t const start_index, uint32_t const end_index, float_type const * const point_x, float_type const * const point_y, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    for (uint32_t const * solid_index_ptr = eq_ptr; solid_index_ptr < eq_ptr + eq_size; ++solid_index_ptr) {
        uint32_t frame_index = start_index;
        uint32_t const result_index = *solid_index_ptr * result->frame_count;

        while (frame_index < end_index) {
            *(result->solid_x + result_index + frame_index) += *(point_x + frame_index);
            *(result->solid_y + result_index + frame_index) += *(point_y + frame_index);
            ++frame_index;
        }
    }
}

static inline void place_and_rotate_eq(Result const * const result, uint32_t const start_index, uint32_t const end_index, float_type const * const point_x, float_type const * const point_y, float_type const * const rx, float_type const * const ry, uint32_t const eq_size, uint32_t const * const eq_ptr) {
    for (uint32_t const * solid_index_ptr = eq_ptr; solid_index_ptr < eq_ptr + eq_size; ++solid_index_ptr) {
        uint32_t frame_index = start_index;
        uint32_t const result_index = *solid_index_ptr * result->frame_count;

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

    __typeof__(graph_step->edges) const r0 = graph_step->edges + 0;
    __typeof__(graph_step->edges) const r1 = graph_step->edges + 1;
    __typeof__(graph_step->edges) const r2 = graph_step->edges + 2;

    __typeof__(system->config.joints) const solid_ptr_s[] = {
        (__typeof__(system->config.joints)) &system->config.joints->solid1,
        (__typeof__(system->config.joints)) &system->config.joints->solid2,
    };
    __typeof__(system->joint_parameters_ptr) const param_ptr_s[] = {
        (__typeof__(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x1,
        (__typeof__(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x2
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
        math(1.0),
        math(-1.0)
    };
    float_type const sign = signs[graph_step->solution_index];

    uint32_t frame_index = start_index;

    while (frame_index < end_index) {
        float_type vec0_x; float_type vec0_y;
        compute_solid_point_diff(result, frame_index, &p01, &p00, &vec0_x, &vec0_y);
        float_type sq_vec0 = vec0_x * vec0_x + vec0_y * vec0_y;

        float_type vec2_x; float_type vec2_y;
        compute_solid_point(result, frame_index, &p22, &vec2_x, &vec2_y);
        float_type sq_vec2 = vec2_x * vec2_x + vec2_y * vec2_y;

        float_type vec1_x; float_type vec1_y;
        compute_solid_point(result, frame_index, &p12, &vec1_x, &vec1_y);
        float_type sq_vec1 = vec1_x * vec1_x + vec1_y * vec1_y;


        float_type const inv_01 = math(1.0) / math(sqrt)(sq_vec0 * sq_vec1);
        float_type const cos_angle = math(0.5) * (sq_vec0 + sq_vec1 - sq_vec2) * inv_01;
        float_type const sin_angle = sign * math(sqrt)(math(1.0) - cos_angle * cos_angle);

        float_type const align_x = (vec1_x * vec0_x + vec1_y * vec0_y) * inv_01;
        float_type const align_y = (vec0_y * vec1_x - vec0_x * vec1_y) * inv_01;

        *(result->_temp_arrays[2] + frame_index) = align_x * cos_angle - align_y * sin_angle;
        *(result->_temp_arrays[3] + frame_index) = align_x * sin_angle + align_y * cos_angle;

        ++frame_index;
    }

    load_point(result, start_index, end_index, &p00, result->_temp_arrays[0], result->_temp_arrays[1]);
    place_and_rotate_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], result->_temp_arrays[2], result->_temp_arrays[3], graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);

    paste_rr(result, start_index, end_index, &p01, &p12, &p22, graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);
}

/*
         0
        / \
       R0  R1
      /     \
     1 - P2- 2
*/
void solve_graph_rrp(System const * const system, ResolutionStep const * const step, Result * const result, uint32_t const start_index, uint32_t const end_index) {
    GraphStep const * const graph_step = &step->graph_step;

    __typeof__(graph_step->edges) const r0 = graph_step->edges + 0;
    __typeof__(graph_step->edges) const r1 = graph_step->edges + 1;
    __typeof__(graph_step->edges) const p2 = graph_step->edges + 2;

    __typeof__(system->config.joints) const solid_ptr_s[] = {
        (__typeof__(system->config.joints)) &system->config.joints->solid1,
        (__typeof__(system->config.joints)) &system->config.joints->solid2,
    };
    __typeof__(system->joint_parameters_ptr) const param_ptr_s[] = {
        (__typeof__(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x1,
        (__typeof__(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x2
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
        .solid_index = *(uint32_t*)(solid_ptr_s[!p2->orientation] + p2->joint_index),
        .px = *((float_type*)(param_ptr_s[!p2->orientation] + p2->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[!p2->orientation] + p2->joint_index) + 1)
    };

    PointDescription const p22 =  {
        .solid_index = *(uint32_t*)(solid_ptr_s[p2->orientation] + p2->joint_index),
        .px = *((float_type*)(param_ptr_s[p2->orientation] + p2->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[p2->orientation] + p2->joint_index) + 1)
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

    uint32_t frame_index = start_index;
    while (frame_index < end_index) {
        float_type x1; float_type y1;
        compute_prismatic_orientation(result, frame_index, &p12, &x1, &y1);
        float_type const nx = x1; float_type const ny = y1;

        float_type x2; float_type y2;
        compute_prismatic_orientation(result, frame_index, &p22, &x2, &y2);


        float_type const inv_mag = math(1.0) / math(sqrt)((x1 * x1 + y1 * y1) * (x2 * x2 * y2 * y2));
        float_type const prismatic_alignment_x = (x1 * x2 + y1 * y2) * inv_mag;
        float_type const prismatic_alignment_y = (x2 * y1 - y2 * x1) * inv_mag;

        move_to_prismatic_point(result, frame_index, &p12, &x1, &y1);
        move_to_prismatic_point(result, frame_index, &p22, &x2, &y2);

        float_type const vec2_x = - (x1 - prismatic_alignment_x * x2 - prismatic_alignment_y * y2);
        float_type const vec2_y = - (y1 - prismatic_alignment_y * x2 + prismatic_alignment_x * y2);

        float_type const cos_target = vec2_x * nx + vec2_y * ny;
        float_type const sin_target = vec2_y * nx - vec2_x * ny;

        float_type vec0_x; float_type vec0_y;
        compute_solid_point_diff(result, frame_index, &p01, &p00, &vec0_x, &vec0_y);

        float_type const sq_inv_vec0 = math(1.0) / (vec0_x * vec0_x + vec0_y * vec0_y) / (nx * nx + ny* ny);
        float_type const cos_0 = vec0_x * nx + vec0_y * ny;
        float_type const sin_0 = sign * (vec0_y * nx - vec0_x * ny);

        *(result->_temp_arrays[0] + frame_index) = sq_inv_vec0 * (cos_0 * cos_target - sin_0 * sin_target);
        *(result->_temp_arrays[1] + frame_index) = sq_inv_vec0 * (cos_0 * sin_target + sin_0 * cos_target);
        *(result->_temp_arrays[2] + frame_index) =(*(result->_temp_arrays[0] + frame_index) * prismatic_alignment_x - *(result->_temp_arrays[1] + frame_index) * prismatic_alignment_y);
        *(result->_temp_arrays[3] + frame_index) =(*(result->_temp_arrays[0] + frame_index) * prismatic_alignment_y + *(result->_temp_arrays[1] + frame_index) * prismatic_alignment_x);
        ++frame_index;
    }
    rotate_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);
    rotate_eq(result, start_index, end_index, result->_temp_arrays[2], result->_temp_arrays[3], graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);

    load_point(result, start_index, end_index, &p00, result->_temp_arrays[0], result->_temp_arrays[1]);
    place_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);

    load_point(result, start_index, end_index, &p01, result->_temp_arrays[0], result->_temp_arrays[1]);
    place_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);
}

/*
         0
        / \
       P0  P1
      /     \
     1 - R2- 2
*/
void solve_graph_ppr(System const * const system, ResolutionStep const * const step, Result * const result, uint32_t const start_index, uint32_t const end_index) {
    GraphStep const * const graph_step = &step->graph_step;

    __typeof__(graph_step->edges) const p0 = graph_step->edges + 0;
    __typeof__(graph_step->edges) const p1 = graph_step->edges + 1;
    __typeof__(graph_step->edges) const r2 = graph_step->edges + 2;

    __typeof__(system->config.joints) const solid_ptr_s[] = {
        (__typeof__(system->config.joints)) &system->config.joints->solid1,
        (__typeof__(system->config.joints)) &system->config.joints->solid2,
    };
    __typeof__(system->joint_parameters_ptr) const param_ptr_s[] = {
        (__typeof__(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x1,
        (__typeof__(system->joint_parameters_ptr)) &system->joint_parameters_ptr->x2
    };

    /* p{eq}{joint} on graph */
    PointDescription const p00 = {
        .solid_index = *(uint32_t*)(solid_ptr_s[!p0->orientation] + p0->joint_index),
        .px = *((float_type*)(param_ptr_s[!p0->orientation] + p0->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[!p0->orientation] + p0->joint_index) + 1)
    };

    PointDescription const p10 = {
        .solid_index = *(uint32_t*)(solid_ptr_s[p0->orientation] + p0->joint_index),
        .px = *((float_type*)(param_ptr_s[p0->orientation] + p0->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[p0->orientation] + p0->joint_index) + 1)
    };

    PointDescription const p01 =  {
        .solid_index = *(uint32_t*)(solid_ptr_s[!p1->orientation] + p1->joint_index),
        .px = *((float_type*)(param_ptr_s[!p1->orientation] + p1->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[!p1->orientation] + p1->joint_index) + 1)
    };

    PointDescription const p21 =  {
        .solid_index = *(uint32_t*)(solid_ptr_s[p1->orientation] + p1->joint_index),
        .px = *((float_type*)(param_ptr_s[p1->orientation] + p1->joint_index) + 0),
        .py = *((float_type*)(param_ptr_s[p1->orientation] + p1->joint_index) + 1)
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
    align_prismatic_eq(result, start_index, end_index, &p00, &p10, graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);


    load_point(result, start_index, end_index, &p21, result->_temp_arrays[0], result->_temp_arrays[1]);
    move_eq_to_point(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);
    align_prismatic_eq(result, start_index, end_index, &p01, &p21, graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);


    uint32_t frame_index = start_index;
    while (frame_index < end_index) {
        float_type vec_x;
        float_type vec_y;
        compute_solid_point_diff(result, frame_index, &p01, &p00, &vec_x, &vec_y);
        float_type tmp_x;
        float_type tmp_y;
        compute_solid_point_diff(result, frame_index, &p22, &p12, &tmp_x, &tmp_y);
        vec_x += tmp_x;
        vec_y += tmp_y;

        float_type n1_x; float_type n1_y;
        compute_prismatic_orientation(result, frame_index, &p00, &n1_x, &n1_y);

        float_type n2_x; float_type n2_y;
        compute_prismatic_orientation(result, frame_index, &p01, &n2_x, &n2_y);

        float_type const det_n0_n1 = n1_x * n2_y - n1_y * n2_x;
        float_type const k1 = (vec_x * n1_x + vec_y * n1_y) / det_n0_n1;
        float_type const k2 = -(vec_x * n2_x + vec_y * n2_y) / det_n0_n1;

        compute_solid_point(result, frame_index, &p00, result->_temp_arrays[0] + frame_index, result->_temp_arrays[1] + frame_index);
        *(result->_temp_arrays[0] + frame_index) +=  - k1 * n1_y;
        *(result->_temp_arrays[1] + frame_index) +=  k1 * n1_x;

        compute_solid_point(result, frame_index, &p00, result->_temp_arrays[2] + frame_index, result->_temp_arrays[3] + frame_index);
        *(result->_temp_arrays[2] + frame_index) +=  - k2 * n2_y;
        *(result->_temp_arrays[3] + frame_index) +=  k2 * n2_x;

        ++frame_index;
    }

    place_eq(result, start_index, end_index, result->_temp_arrays[0], result->_temp_arrays[1], graph_step->eq_indices[2] - graph_step->eq_indices[1], graph_step->eqs + graph_step->eq_indices[1]);
    place_eq(result, start_index, end_index, result->_temp_arrays[2], result->_temp_arrays[3], graph_step->eq_indices[3] - graph_step->eq_indices[2], graph_step->eqs + graph_step->eq_indices[2]);
}