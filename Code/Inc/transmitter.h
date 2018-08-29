#ifndef __TRANSMITTER_H
#define __TRANSMITTER_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include "ch_math.h"
#include "string.h"
#include "stm32f1xx.h"

#define TRANSMITTER_MAX_DATA_LEN 			255

typedef struct __attribute__((__packed__))
{
	uint8_t start;
	uint8_t len;
	uint8_t message_id;
	float data[TRANSMITTER_MAX_DATA_LEN];
	uint8_t stop;
}transmitter_msg_t;

#define IMU_PCK_LEN      44
#define IMU_PCK_MSG_ID   0x0a
typedef struct __attribute__((__packed__))
{
	float timestamps;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float q0;
	float q1;
	float q2;
	float q3;
}imu_pck;

extern transmitter_msg_t tx_msg;

void transmitter_init(void);

uint8_t transmitter_send_imu_data(vector3f_t acc, vector3f_t gyro, quaternion_t q);

#endif

