#include "kinematics/kinematics.h"
#include "interface/interface.h"
#include "graph/graph_interface.h"
#include "stdio.h"
#include "internal/enums.h"
#include "kinematics/kinematics.h"
#include "stdlib.h"
#include <inttypes.h>
#include "processthreadsapi.h"
#include "realtimeapiset.h"
#include <pthread.h>
#define FRAME_COUNT (1 << 24)



int main() {
    KpSystem_f32 system;
    kp_allocate_system_f32(&system, 3, 3, 0);

    kp_configure_joint(&system.config, 0, JOINT_TYPE_REVOLUTE, 0, 1);
    kp_configure_joint(&system.config, 1, JOINT_TYPE_REVOLUTE, 0, 2);
    kp_configure_joint(&system.config, 2, JOINT_TYPE_REVOLUTE, 1, 2);

    KpResult_f32 results;
    kp_allocate_result_f32(&system.config, FRAME_COUNT, &results);

    system.joint_parameters_ptr[0].x1 = 0.0f;
    system.joint_parameters_ptr[0].x2 = 0.0f;
    system.joint_parameters_ptr[0].y1 = 0.0f;
    system.joint_parameters_ptr[0].y2 = 0.0f;

    system.joint_parameters_ptr[1].x1 = 1.0f;
    system.joint_parameters_ptr[1].x2 = 0.0f;
    system.joint_parameters_ptr[1].y1 = 0.0f;
    system.joint_parameters_ptr[1].y2 = 0.0f;

    system.joint_parameters_ptr[2].x1 = 1.0f;
    system.joint_parameters_ptr[2].x2 = 1.0f;
    system.joint_parameters_ptr[2].y1 = 0.0f;
    system.joint_parameters_ptr[2].y2 = 0.0f;

    ResolutionMode kinematics = {0};
    if (kp_determine_computation_order(&system.config, &kinematics) != KINEPY_SUCCESS) {
        printf("Failed\n");
        exit(1);
    }
    reset_results_f32(&system.config, &results, 0, FRAME_COUNT);
    // Warm up
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);

    uint64_t date;
    QueryProcessCycleTime(GetCurrentProcess(), &date);

    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);
    solve_graph_rrr_f32(&system, kinematics.steps.array, &results, 0, FRAME_COUNT);

    uint64_t final_date;
    QueryProcessCycleTime(GetCurrentProcess(), &final_date);

    printf("%llu\n", (final_date - date) / 4);

    kp_free_result_f32(&results);
    kp_free_system_f32(&system);
}