#include "ch_math.h"

vector3f_t vec_plus(vector3f_t a, vector3f_t b)
{
	vector3f_t c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	
	return c;
}

vector3f_t vec_times(vector3f_t a, float k)
{
	vector3f_t b;
	b.x = a.x * k;
	b.y = a.y * k;
	b.z = a.z * k;
	return b;
}

float vec_dot(vector3f_t a, vector3f_t b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
