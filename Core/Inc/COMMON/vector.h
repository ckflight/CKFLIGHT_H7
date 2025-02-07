
#ifndef INC_COMMON_VECTOR_H_
#define INC_COMMON_VECTOR_H_
#pragma once

//#include <stdbool.h>
#include "CK_DEFINITIONS.h"

typedef union vector2_u {
    float v[2];
    struct {
       float x, y;
    };
} vector2_t;

typedef union vector3_u {
    float v[3];
    struct {
       float x, y, z;
    };
} vector3_t;

typedef struct matrix33_s {
    float m[3][3];
} matrix33_t;

bool vector2Equal(const vector2_t *a, const vector2_t *b);
vector2_t *vector2Zero(vector2_t *v);
vector2_t *vector2Add(vector2_t *result, const vector2_t *a, const vector2_t *b);
vector2_t *vector2Sub(vector2_t *result, const vector2_t *a, const vector2_t *b);
vector2_t *vector2Scale(vector2_t *result, const vector2_t *v, const float k);
float vector2Dot(const vector2_t *a, const vector2_t *b);
float vector2Cross(const vector2_t *a, const vector2_t *b);
float vector2NormSq(const vector2_t *v);
float vector2Norm(const vector2_t *v);
vector2_t *vector2Normalize(vector2_t *result, const vector2_t *v);
vector2_t *vector2Rotate(vector2_t *result, const vector2_t *v, const float angle);

bool vector3Equal(const vector3_t *a, const vector3_t *b);
vector3_t *vector3Zero(vector3_t *v);
vector3_t *vector3Add(vector3_t *result, const vector3_t *a, const vector3_t *b);
vector3_t *vector3Sub(vector3_t *result, const vector3_t *a, const vector3_t *b);
vector3_t *vector3Scale(vector3_t *result, const vector3_t *v, const float k);
float vector3Dot(const vector3_t *a, const vector3_t *b);
vector3_t *vector3Cross(vector3_t *result, const vector3_t *a, const vector3_t *b);
float vector3NormSq(const vector3_t *v);
float vector3Norm(const vector3_t *v);
vector3_t *vector3Normalize(vector3_t *result, const vector3_t *v);

vector3_t *matrixVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);
vector3_t *matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);

matrix33_t *buildRotationMatrix(matrix33_t *result, const fp_angles_t *rpy);
vector3_t *applyRotationMatrix(vector3_t *v, const matrix33_t *rotationMatrix);

matrix33_t *yawToRotationMatrixZ(matrix33_t *result, const float yaw);

#endif
