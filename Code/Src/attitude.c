#include "attitude.h"
#include "MadgwickAHRS.h"
#include <math.h>

quaternion_t quat;

vector3f_t acc_g;
vector3f_t gyro_rad;
vector3f_t mag;

vector3f_t euler_rad;
vector3f_t euler_deg;

uint8_t computing = 0;

void attitude_init(void)
{
	mpu9250_init();
	
	HAL_Delay(100);
}


void read_imu_data(void)
{
	
	mpu9250_read_all();
	
	acc_g.x = ax;
	acc_g.y = ay;
	acc_g.z = az;
	
	gyro_rad.x = gx / RAD2DEG;
	gyro_rad.y = gy / RAD2DEG;
	gyro_rad.z = gz / RAD2DEG;	
	
//	mag.x = mx;
//	mag.y = my;
//	mag.z = mz;
}


void print_mpu_data(void)
{
	if(computing == 1) return;
	printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\n", (int)(ax*1000), (int)(ay*1000), (int)(az*1000), (int)(gx*1000), (int)(gy*1000), (int)(gz*1000), (int)temperature);
}

void print_attitude(void)
{
	if(computing == 1) return;
	printf("%d  %d  %d \n", (int)euler_deg.x, (int)euler_deg.y, (int)euler_deg.z);
}

void print_mag(void)
{
	if(computing == 1) return;
	printf("%d  %d  %d \n", (int)(1000*mag.x), (int)(1000*mag.y), (int)(1000*mag.z));
}


//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

void attitude_updata(void)
{
	computing = 1;

	read_imu_data();
	
//	MadgwickAHRSupdate(gyro_rad.x, gyro_rad.y, gyro_rad.z, acc_g.x, acc_g.y, acc_g.z, mag.x, mag.y, mag.z);
	MadgwickAHRSupdateIMU(gyro_rad.x, gyro_rad.y, gyro_rad.z, acc_g.x, acc_g.y, acc_g.z);
	quat.q0 = q0;
	quat.q1 = q1;
	quat.q2 = q2;
	quat.q3 = q3;
	
	euler_rad = quat2euler(quat);
	euler_deg = vec_times(euler_rad, RAD2DEG);
	computing = 0;

}

//--------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------


vector3f_t quat2euler(quaternion_t q)
{
	vector3f_t euler;
	// roll (x-axis rotation)
	float sinr = +2.0 * (q.q0 * q.q1 + q.q2 * q.q3);
	float cosr = +1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2);
	euler.x = atan2f(sinr, cosr);

	// pitch (y-axis rotation)
	float sinp = +2.0 * (q.q0 * q.q2 - q.q3 * q.q1);
	if (fabsf(sinp) >= 1)
		euler.y = copysignf(PI / 2, sinp); // use 90 degrees if out of range
	else
		euler.y = asinf(sinp);

	// yaw (z-axis rotation)
	float siny = +2.0 * (q.q0 * q.q3 + q.q1 * q.q2);
	float cosy = +1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3);  
	euler.z = atan2f(siny, cosy);
	
	return euler;
}
	
vector3f_t get_euler_deg(void) {return euler_deg;}

vector3f_t get_euler_rad(void) {return euler_rad;}

vector3f_t get_gyro(void) {return gyro_rad;}
vector3f_t get_acc(void) {return acc_g;}
vector3f_t get_mag(void) {return mag;}
quaternion_t get_quat(void) {return quat;}
