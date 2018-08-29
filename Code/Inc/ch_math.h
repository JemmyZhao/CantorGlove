#ifndef __CH_MATH_H__
#define __CH_MATH_H__
#include <math.h>

typedef struct
{
	float x;
	float y;
  float z;
}vector3f_t;

typedef struct
{
	float a[3][3];
}matrix3f_t;

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}quaternion_t;

vector3f_t vec_plus(vector3f_t a, vector3f_t b);

vector3f_t vec_times(vector3f_t a, float k);

float vec_dot(vector3f_t a, vector3f_t b);

#endif



