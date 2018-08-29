#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__
#include "ch_math.h"
#include "mpu9250.h"

#ifndef RAD2DEG
#define RAD2DEG      57.295779513082320f
#endif

#ifndef PI
#define PI  3.141592653589793f
#endif


extern vector3f_t acc;
extern vector3f_t gyro;
extern quaternion_t quat;
extern int16_t mpu_raw_buf[10];

void attitude_init(void);

void attitude_updata(void);

void read_mpu6500(void);
void gyro_calibration(void);
void print_mpu_data(void);
void print_attitude(void);
void print_mag(void);

vector3f_t quat2euler(quaternion_t q);

vector3f_t get_euler_deg(void);
vector3f_t get_euler_rad(void);
vector3f_t get_gyro(void);
vector3f_t get_acc(void);
vector3f_t get_mag(void);
quaternion_t get_quat(void);

#endif

