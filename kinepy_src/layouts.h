#ifndef LAYOUTS_H
#define LAYOUTS_H

#include "stdint.h"

typedef enum {
    KINEPY_SUCCESS,
    KINEPY_FAILURE,
    KINEPY_INVALID_INPUT,
    KINEPY_INVALID_INPUT_NEGATIVE_INERTIAL_VALUE,
    KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS,
    KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX,
    KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE,
    KINEPY_INVALID_INPUT_WRONG_RELATION_TYPE,
    KINEPY_INVALID_INPUT_UNMATCHED_RELATION_TYPE_AND_JOINT_TYPES,
    KINEPY_NO_GRAPH_FOUND,
    KINEPY_MALLOC_FAILED
} KinepyResult;

typedef struct {
    uint32_t count;
    uint32_t * joints;
} JointArray;

typedef struct {
    uint32_t joint_index;
    uint32_t orientation;
} GraphStepEdge;

typedef struct {
    uint32_t isostatic_graph;
    uint32_t * eq_indices; // |eq_indices| = isostatic_graph->vertex_count+1
    uint32_t * eqs; // |eqs| = eq_indices[isostatic_graph->vertex_count]
    GraphStepEdge * edges; // |edges| = isostatic_graph->edge_count
    uint8_t solution_index; // solution_index < isostatic_graph->solution_count
} GraphStep;

typedef struct {
    uint32_t joint_index;
    uint32_t first_eq_size;
    uint32_t second_eq_size;
    uint32_t * eqs;
} JointStep;

typedef enum {
    STEP_TYPE_GRAPH,
    STEP_TYPE_JOINT,
    STEP_TYPE_RELATION
} StepType;

typedef struct {
    StepType type;
    union {
        GraphStep graph_step;
        JointStep joint_step;
    };
} ResolutionStep;

typedef struct {
    uint32_t count;
    ResolutionStep * array;
} StepArray;

typedef struct {
    JointArray piloted_or_blocked_joints;
    StepArray steps;
} ResolutionMode;


#pragma region SolidDescrition
typedef struct { 
    float mass;
    float moment_of_inertia;
    float gx;
    float gy; 
} SolidDescription_s;

typedef struct { 
    uint32_t obj_count;
    float *mass_ptr;
    float *moment_of_inertia_ptr;
    float *gx_ptr;
    float *gy_ptr;
} SolidDescriptionArray_s;

typedef struct { 
    double mass;
    double moment_of_inertia;
    double gx;
    double gy; 
} SolidDescription_d;

typedef struct { 
    uint32_t obj_count;
    double *mass_ptr;
    double *moment_of_inertia_ptr;
    double *gx_ptr;
    double *gy_ptr; 
} SolidDescriptionArray_d;
#pragma endregion SolidDescrition

#pragma region JointDescription
typedef struct { 
    uint32_t solid1;
    uint32_t solid2;
    uint8_t type;
    float constraint1_x;
    float constraint1_y;
    float constraint2_x;
    float constraint2_y;
} JointDescription_s;

typedef struct { 
    uint32_t obj_count;
    uint32_t *solid1_ptr;
    uint32_t *solid2_ptr;
    uint8_t *type_ptr;
    float *constraint1_x_ptr;
    float *constraint1_y_ptr;
    float *constraint2_x_ptr;
    float *constraint2_y_ptr;
} JointDescriptionArray_s;

typedef struct { 
    uint32_t solid1;
    uint32_t solid2;
    uint8_t type;
    double constraint1_x;
    double constraint1_y;
    double constraint2_x;
    double constraint2_y;
} JointDescription_d;

typedef struct { 
    uint32_t obj_count;
    uint32_t *solid1_ptr;
    uint32_t *solid2_ptr;
    uint8_t *type_ptr;
    double *constraint1_x_ptr;
    double *constraint1_y_ptr;
    double *constraint2_x_ptr;
    double *constraint2_y_ptr; 
} JointDescriptionArray_d;
#pragma endregion JointDescription

#pragma region SolidResult
typedef struct { 
    float origin_x;
    float origin_y;
    float orientation_x;
    float orientation_y;
} SolidResult_s;

typedef struct {
    uint32_t frame_count;
    float *origin_x_ptr;
    float *origin_y_ptr;
    float *orientation_x_ptr;
    float *orientation_y_ptr;
    float *_dynamic_x_ptr;
    float *_dynamic_y_ptr;
    float *_dynamic_m_ptr; 
} SolidResultArray_s;

typedef struct { 
    double origin_x;
    double origin_y;
    double orientation_x;
    double orientation_y;
} SolidResult_d;

typedef struct {
    uint32_t frame_count;
    double *origin_x_ptr;
    double *origin_y_ptr;
    double *orientation_x_ptr;
    double *orientation_y_ptr;
    double *_dynamic_x_ptr;
    double *_dynamic_y_ptr;
    double *_dynamic_m_ptr; 
} SolidResultArray_d;

#pragma endregion SolidResult

#pragma region RelationDescription
typedef struct { 
    uint32_t joint1;
    uint32_t joint2;
    uint8_t type;
    float ratio;
    float v0; 
} RelationDescription_s;

typedef struct { 
    uint32_t obj_count;
    uint32_t *joint1_ptr
    ;uint32_t *joint2_ptr;
    uint8_t *type_ptr;
    float *ratio_ptr;
    float *v0_ptr; 
} RelationDescriptionArray_s;

typedef struct { 
    uint32_t joint1;
    uint32_t joint2;
    uint8_t type;
    double ratio;
    double v0; 
} RelationDescription_d;

typedef struct { 
    uint32_t obj_count;
    uint32_t *joint1_ptr;
    uint32_t *joint2_ptr;
    uint8_t *type_ptr;
    double *ratio_ptr;
    double *v0_ptr; 
} RelationDescriptionArray_d;

#pragma endregion RelationDescrption

#pragma region System
typedef struct {
    float angle;
    float length;
    float mass;
    float moment_of_inertia;
    float force;
    float torque;
    float dimensionless;

} UnitSystem_s;

typedef struct { 
    SolidDescriptionArray_s solid_description_array;
    JointDescriptionArray_s joint_description_array;
    SolidResultArray_s solid_result_array;
    RelationDescriptionArray_s relation_description_array;
    ResolutionMode kinematics;
    ResolutionMode dynamics;
    UnitSystem_s *unit_system; 
} System_s;

typedef struct {
    double angle;
    double length;
    double mass;
    double moment_of_inertia;
    double force;
    double torque;
    double dimensionless;
} UnitSystem_d;

typedef struct { 
    SolidDescriptionArray_d solid_description_array;
    JointDescriptionArray_d joint_description_array;
    SolidResultArray_d solid_result_array;
    RelationDescriptionArray_d relation_description_array;
    ResolutionMode kinematics;
    ResolutionMode dynamics;
    UnitSystem_d *unit_system; 
} System_d;

typedef struct {
    struct { 
        uint32_t obj_count;
        void *mass_ptr;
        void *moment_of_inertia_ptr;
        void *gx_ptr;
        void *gy_ptr; 
    } solid_description_array;
    struct { 
        uint32_t obj_count;
        uint32_t *solid1_ptr;
        uint32_t *solid2_ptr;
        uint8_t *type_ptr;
        void *constraint1_x_ptr;
        void *constraint1_y_ptr;
        void *constraint2_x_ptr;
        void *constraint2_y_ptr; 
    } joint_description_array;
    struct {
        uint32_t frame_count;
        void *origin_x_ptr;
        void *origin_y_ptr;
        void *orientation_x_ptr;
        void *orientation_y_ptr;
        void *_dynamic_x_ptr;
        void *_dynamic_y_ptr;
        void *_dynamic_m_ptr; 
    } solid_result_array;
    struct { 
        uint32_t obj_count;
        uint32_t *joint1_ptr;
        uint32_t *joint2_ptr;
        uint8_t *type_ptr;
        void *ratio_ptr;
        void *v0_ptr;
    } relation_description_array;
    ResolutionMode kinematics;
    ResolutionMode dynamics;
    void * unit_system;
} system_internal;
#pragma endregion System

uint32_t KINEPY_allocate_solid_descriptions_s(System_s *system, uint32_t obj_count);
void KINEPY_free_solid_descriptions_s(System_s *system);
void KINEPY_get_solid_description_s(System_s const *system, uint32_t obj_index, SolidDescription_s *output);
uint32_t KINEPY_set_solid_description_s(System_s *system, uint32_t obj_index, SolidDescription_s const *input);

uint32_t KINEPY_allocate_solid_descriptions_d(System_d *system, uint32_t obj_count);
void KINEPY_free_solid_descriptions_d(System_d *system);
void KINEPY_get_solid_description_d(System_d const *system, uint32_t obj_index, SolidDescription_d *output);
uint32_t KINEPY_set_solid_description_d(System_d *system, uint32_t obj_index, SolidDescription_d const *input);

uint32_t KINEPY_allocate_joint_descriptions_s(System_s *system, uint32_t obj_count);
void KINEPY_free_joint_descriptions_s(System_s *system);
void KINEPY_get_joint_description_s(System_s const *system, uint32_t obj_index, JointDescription_s *output);
uint32_t KINEPY_set_joint_description_s(System_s *system, uint32_t obj_index, JointDescription_s const *input);

uint32_t KINEPY_allocate_joint_descriptions_d(System_d *system, uint32_t obj_count);
void KINEPY_free_joint_descriptions_d(System_d *system);
void KINEPY_get_joint_description_d(System_d const *system, uint32_t obj_index, JointDescription_d *output);
uint32_t KINEPY_set_joint_description_d(System_d *system, uint32_t obj_index, JointDescription_d const *input);

uint32_t KINEPY_allocate_solid_results_s(System_s *system, uint32_t frame_count);
void KINEPY_free_solid_results_s(System_s *system);
void KINEPY_get_solid_result_s(System_s const *system, uint32_t obj_index, uint32_t frame_index, SolidResult_s *output);

uint32_t KINEPY_allocate_solid_results_d(System_d *system, uint32_t frame_count);
void KINEPY_free_solid_results_d(System_d *system);
void KINEPY_get_solid_result_d(System_d const *system, uint32_t obj_index, uint32_t frame_index, SolidResult_d *output);

uint32_t KINEPY_allocate_relation_descriptions_s(System_s *system, uint32_t obj_count);
void KINEPY_free_relation_descriptions_s(System_s *system);
void KINEPY_get_relation_description_s(System_s const *system, uint32_t obj_index, RelationDescription_s *output);
uint32_t KINEPY_set_relation_description_s(System_s *system, uint32_t obj_index, RelationDescription_s const *input);

uint32_t KINEPY_allocate_relation_descriptions_d(System_d *system, uint32_t obj_count);
void KINEPY_free_relation_descriptions_d(System_d *system);
void KINEPY_get_relation_description_d(System_d const *system, uint32_t obj_index, RelationDescription_d *output);
uint32_t KINEPY_set_relation_description_d(System_d *system, uint32_t obj_index, RelationDescription_d const *input);

typedef enum {
    JOINT_TYPE_EMPTY,
    JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_COUNT
} JointType;

typedef enum {
    RELATION_TYPE_EMPTY,
    RELATION_TYPE_GEAR,
    RELATION_TYPE_GEAR_RACK,
    RELATION_TYPE_DISTANT,
    RELATION_TYPE_EFFORTLESS,
    RELATION_TYPE_COUNT
} RelationType;

#endif //LAYOUTS_H
