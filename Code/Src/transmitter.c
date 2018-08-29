#include "transmitter.h"

transmitter_msg_t tx_msg;

void transmitter_init(void)
{
	tx_msg.start = 0xfe;
	tx_msg.len = IMU_PCK_LEN;
	tx_msg.message_id = IMU_PCK_MSG_ID;
	tx_msg.stop = 0xfc;
}

uint8_t send_msg(void)
{
//	HAL_UART_Transmit(&huart1, &tx_msg.start, 1, 0);
//	HAL_UART_Transmit(&huart1, &tx_msg.len, 1, 0);
//  HAL_UART_Transmit(&huart1, &tx_msg.message_id, 1, 0);
//	for(int i = 0; i < tx_msg.len; i++)
//	{
//		HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_msg.data[i*4]), sizeof(float), 0);
//	}
//	HAL_UART_Transmit(&huart1, &tx_msg.stop, 1, 0);
	HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_msg), tx_msg.len + 3, 10);
	
	return tx_msg.len + 3;
}


uint8_t transmitter_send_imu_data(vector3f_t acc, vector3f_t gyro, quaternion_t q)
{
	imu_pck pck;
	pck.timestamps = 0;
	pck.ax = acc.x;
	pck.ay = acc.y;
	pck.az = acc.z;
	pck.gx = gyro.x;
	pck.gy = gyro.y;
	pck.gz = gyro.z;
	pck.q0 = q.q0;
	pck.q1 = q.q1;
	pck.q2 = q.q2;
	pck.q3 = q.q3;
	tx_msg.len = IMU_PCK_LEN;
	tx_msg.message_id = IMU_PCK_MSG_ID;
	memcpy(&tx_msg.data, &pck, sizeof(pck));
	tx_msg.data[IMU_PCK_LEN] = tx_msg.stop;
	return send_msg();
}
