#include "MPU9250.h"
#include <math.h>





#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))

#define __GPIO_SET(_PORT, _PIN)    (_PORT->BSRR = _PIN)
#define __GPIO_RST(_PORT, _PIN)    (_PORT->BRR  = _PIN)
#define __GPIO_TOG(_PORT, _PIN)    (_PORT->ODR ^= _PIN)
#define __GPIO_READ(_PORT, _PIN)   (_PORT->IDR  & _PIN)

uint16_t SPI_RW( SPI_TypeDef *SPIx, uint16_t WriteByte )
{
  while((SPIx->SR & SPI_FLAG_TXE) == (uint16_t)RESET);
  SPIx->DR = WriteByte;
  while((SPIx->SR & SPI_FLAG_RXNE) == (uint16_t)RESET);

  return SPIx->DR;
}

#define SPIx                        SPI2
#define SPIx_CLK_ENABLE()           __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIx_SPEED_HIGH             SPI_BAUDRATEPRESCALER_2
#define SPIx_SPEED_LOW              SPI_BAUDRATEPRESCALER_256

#define SPIx_CS_PIN                 GPIO_PIN_12
#define SPIx_CS_GPIO_PORT           GPIOB
#define SPIx_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_CS_AF                  GPIO_AF5_SPI2
#define SPIx_CS_H                   __GPIO_SET(SPIx_CS_GPIO_PORT, SPIx_CS_PIN)
#define SPIx_CS_L                   __GPIO_RST(SPIx_CS_GPIO_PORT, SPIx_CS_PIN)

#define SPIx_SCK_PIN                GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT          GPIOB
#define SPIx_SCK_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_SCK_AF                 GPIO_AF5_SPI2

#define SPIx_SDO_PIN                GPIO_PIN_14
#define SPIx_SDO_GPIO_PORT          GPIOB
#define SPIx_SDO_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_SDO_AF                 GPIO_AF5_SPI2

#define SPIx_SDI_PIN                GPIO_PIN_15
#define SPIx_SDI_GPIO_PORT          GPIOB
#define SPIx_SDI_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_SDI_AF                 GPIO_AF5_SPI2
/*====================================================================================================*/
/*====================================================================================================*/
/*====================================================================================================*/
/*====================================================================================================*
**?? : MPU6500_WriteReg
**?? : Write Reg
**?? : WriteAddr, WriteData
**?? : None
**?? : MPU6500_WriteReg(WriteAddr, WriteData);
**====================================================================================================*/
/*====================================================================================================*/
static void WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
  SPIx_CS_L;
  SPI_RW(SPIx, WriteAddr);
  SPI_RW(SPIx, WriteData);
  SPIx_CS_H;
}
/*====================================================================================================*/
/*====================================================================================================*
**?? : MPU6500_ReadReg
**?? : Read Reg
**?? : ReadAddr
**?? : ReadData
**?? : ReadData = MPU6500_ReadReg(ReadAddr);
**====================================================================================================*/
/*====================================================================================================*/
static uint8_t ReadReg( uint8_t ReadAddr )
{
  uint8_t ReadData = 0;

  SPIx_CS_L;
  SPI_RW(SPIx, 0x80 | ReadAddr);
  ReadData = SPI_RW(SPIx, 0xFF);
  SPIx_CS_H;

  return ReadData;
}
/*=====================================================================================================*/
/*=====================================================================================================*
**?? : MPU6500_ReadRegs
**?? : Read Regs
**?? : ReadAddr, *ReadBuf, Bytes
**?? : None
**?? : MPU6500_ReadRegs(MPU6500_ACCEL_XOUT_H, ReadBuf, 14);
**=====================================================================================================*/
/*=====================================================================================================*/
static void ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes )
{
  SPIx_CS_L;
  SPI_RW(SPIx, 0x80 | ReadAddr);
  for(uint8_t i = 0; i < Bytes; i++)
    ReadBuf[i] = SPI_RW(SPIx, 0xFF);
  SPIx_CS_H;
}

//！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！


// Wire.h read and write protocols
uint8_t writeByte(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t data)
{
	WriteReg(registerAddress, data);
//    return writeByteSPI(registerAddress, data);
}

uint8_t writeByteSPI(uint8_t registerAddress, uint8_t writeData)
{
  uint8_t returnVal;

  uint8_t reg = registerAddress;
	
  select();
	
	SPI_RW(SPIx, registerAddress);
  returnVal = SPI_RW(SPIx, writeData);

  deselect();

  return returnVal;
}



// Read a byte from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
uint8_t readByte(uint8_t deviceAddress, uint8_t registerAddress)
{
	return ReadReg(registerAddress);
//    return readByteSPI(registerAddress);
}


// Read a byte from the given register address using SPI
uint8_t readByteSPI(uint8_t registerAddress)
{
  return writeByteSPI(registerAddress | READ_FLAG, 0xFF /*0xFF is arbitrary*/);
}


// Select slave IC by asserting CS pin
void select()
{
  imu_cs_enable();
}

// Select slave IC by deasserting CS pin
void deselect()
{
  imu_cs_disable();
}

void delay_us(int u)
{
	for(int i = 0; i < u; i++)
	{
		for(int j  = 0; j < 18; j++);
	}
}

uint8_t readBytesSPI(uint8_t registerAddress, uint8_t count,
                           uint8_t * dest)
{
	uint8_t reg = registerAddress | READ_FLAG;
	
  select();
	
	HAL_SPI_TransmitReceive(&hspi2, &reg, &dest[0], 1, 10);

  uint8_t i;

  for (i = 0; i < count; i++)
  {
    HAL_SPI_TransmitReceive(&hspi2, &reg, &dest[i], 1, 10);
  }

  deselect();
	
	delay_us(50);
	
//	HAL_Delay(50);

  return i; // Return number of bytes written

}

uint8_t readBytes(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
//    return readBytesSPI(registerAddress, count, dest);
	ReadRegs(registerAddress, dest, count);
}

uint8_t readBytesDev(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
	SPIx_CS_L;
	SPI_RW(SPIx, deviceAddress);
  SPI_RW(SPIx, 0x80 | registerAddress);
  for(uint8_t i = 0; i < count; i++)
    dest[i] = SPI_RW(SPIx, 0xFF);
  SPIx_CS_H;
}







// TODO: Add setter methods for this hard coded stuff
// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
// Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mscale = MFS_16BITS;

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
uint8_t Mmode = M_100HZ;

// SPI chip select pin
int8_t _csPin;

int16_t tempCount;   // Temperature raw count output
uint32_t delt_t = 0; // Used to control display output rate

uint32_t count = 0, sumCount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
    // Scale resolutions per LSB for the sensors
float aRes, gRes, mRes;
    // Variables to hold latest sensor data values
float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;
    // Factory mag calibration and mag bias
float factoryMagCalibration[3] = {0, 0, 0}, factoryMagBias[3] = {0, 0, 0};
    // Bias corrections for gyro, accelerometer, and magnetometer
float gyroBias[3]  = {0, 0, 0},
      accelBias[3] = {0, 0, 0},
      magBias[3]   = {0, 0, 0},
      magScale[3]  = {0, 0, 0};
float selfTest[6];
// Stores the 16-bit signed accelerometer sensor output
int16_t accelCount[3];

//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================
			

void getMres()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

void getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
  // of data acquisition
  uint8_t rawData[7];
  // Wait for magnetometer data ready bit to be set
//	printf("mag\n");
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
  {
    // Read the six raw data and ST2 registers sequentially into data array
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
    if (!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = (((int16_t)rawData[1] << 8) | rawData[0]);
      // Data stored as little Endian
      destination[1] = (((int16_t)rawData[3] << 8) | rawData[2]);
      destination[2] = (((int16_t)rawData[5] << 8) | rawData[4]);
//			printf("ok\n");
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1];
}

// Calculate the time the last update took for use in the quaternion filters
// TODO: This doesn't really belong in this class.


void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  // TODO: Test this!! Likely doesn't work
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  HAL_Delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(10);

  // Read the x-, y-, and z-axis calibration values
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);

  // Return x-axis sensitivity adjustment values, etc.
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  HAL_Delay(10);

  // Configure the magnetometer for continuous read and highest resolution.
  // Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mmode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates.

  // Set magnetometer data resolution and sample ODR
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
  HAL_Delay(10);
}

void initMPU9250()
{
  // wake up device
  // Clear sleep mode bit (6), enable all sensors
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  HAL_Delay(100); // Wait for all registers to reset

  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  HAL_Delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively;
  // minimum HAL_Delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  // get current GYRO_CONFIG register value
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
  // GYRO_CONFIG
  // c =| 0x00;
  // Write new GYRO_CONFIG value to register
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c );

  // Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  // Write new ACCEL_CONFIG register value
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  // Get current ACCEL_CONFIG2 register value
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  // Write new ACCEL_CONFIG2 register value
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Arduino as master.
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  // Enable data ready (bit 0) interrupt
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
  HAL_Delay(100);
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * gyroBias, float * accelBias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, READ_FLAG);
  HAL_Delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  HAL_Delay(200);

  // Configure device for bias calculation
  // Disable all interrupts
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
  // Disable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
  // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  // Disable I2C master
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
  // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
  // Reset FIFO and DMP
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
  HAL_Delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);
  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // MPU-9150)
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
  HAL_Delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
  // Read FIFO sample count
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void MPU9250SelfTest(float * destination)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
  // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);
  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);
  // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);
  // Set full scale range for the accelerometer to 2 g
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS);

  // Get average current values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++)
  {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
  HAL_Delay(25);  // HAL_Delay a while to let the device stabilize

  // Get average self-test values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++)
  {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  HAL_Delay(25);  // HAL_Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  // X-axis accel self-test results
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);
  // Y-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
  // Z-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);
  // X-axis gyro self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);
  // Y-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);
  // Z-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);

  // Retrieve factory self-test value from self-test code reads
  // FT[Xa] factory trim calculation
  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
  // FT[Ya] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
  // FT[Za] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
  // FT[Xg] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
  // FT[Yg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
  // FT[Zg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    // Report percent differences
    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
      - 100.;
    // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
      - 100.;
  }
}

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void magCalMPU9250(float * bias_dest, float * scale_dest)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3]  = {0, 0, 0},
          mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000},
          mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
          mag_temp[3] = {0, 0, 0};

  // Make sure resolution has been calculated
  getMres();

  HAL_Delay(4000);

  // shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == M_8HZ)
  {
    sample_count = 128;
  }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {
    sample_count = 1500;
  }

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp[jj];
      }
      if (mag_temp[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp[jj];
      }
    }

    if (Mmode == M_8HZ)
    {
      HAL_Delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
    }
    if (Mmode == M_100HZ)
    {
      HAL_Delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    }
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // Save mag biases in G for main program
  bias_dest[0] = (float)mag_bias[0] * mRes * factoryMagCalibration[0];
  bias_dest[1] = (float)mag_bias[1] * mRes * factoryMagCalibration[1];
  bias_dest[2] = (float)mag_bias[2] * mRes * factoryMagCalibration[2];

  // Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

}


void magInit()
{
  // Reset registers to defaults, bit auto clears
  writeByteSPI(0x6B, 0x80);
  // Auto select the best available clock source
  writeByteSPI(0x6B, 0x01);
  // Enable X,Y, & Z axes of accel and gyro
  writeByteSPI(0x6C, 0x00);
  // Config disable FSYNC pin, set gyro/temp bandwidth to 184/188 Hz
  writeByteSPI(CONFIG, MPU_GyrLPS_184Hz);
  // Self tests off, gyro set to +/-2000 dps FS
  writeByteSPI(GYRO_CONFIG, MPU_GyrFS_2000dps);
  // Self test off, accel set to +/- 8g FS
  writeByteSPI(ACCEL_CONFIG, MPU_AccFS_8g);
  // Bypass DLPF and set accel bandwidth to 184 Hz
  writeByteSPI(ACCEL_CONFIG2, MPU_AccLPS_184Hz);
  // Configure INT pin (active high / push-pull / latch until read)
  writeByteSPI(0x37, 0x30);
  // Enable I2C master mode
  // TODO Why not do this 11-100 ms after power up?
  writeByteSPI(0x6A, 0x20);
  // Disable multi-master and set I2C master clock to 400 kHz
  //https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/ calls says
  // enabled multi-master... TODO Find out why
  writeByteSPI(0x24, 0x0D);
  // Set to write to slave address 0x0C
  writeByteSPI(0x25, 0x0C);
  // Point save 0 register at AK8963's control 2 (soft reset) register
  writeByteSPI(0x26, 0x0B);
  // Send 0x01 to AK8963 via slave 0 to trigger a soft restart
  writeByteSPI(0x63, 0x01);
  // Enable simple 1-byte I2C reads from slave 0
  writeByteSPI(0x27, 0x81);
  // Point save 0 register at AK8963's control 1 (mode) register
  writeByteSPI(0x26, 0x0A);
  // 16-bit continuous measurement mode 1
  writeByteSPI(0x63, 0x12);
  // Enable simple 1-byte I2C reads from slave 0
  writeByteSPI(0x27, 0x81);

  // TODO: Remove this code
  uint8_t ret = ak8963WhoAmI_SPI();
	
	send_byte(ret);

//  return(ret == 0x48;
}

// Write a null byte w/o CS assertion to get SPI hardware to idle high (mode 3)
void kickHardware()
{
	uint8_t reg = 0x00;
  HAL_SPI_Transmit(&hspi2, &reg, 1, 10);
}

void mpu9250_init()
{
  kickHardware();
	initMPU9250();
	initAK8963(&magBias[0]);
//	

//	
//	calibrateMPU9250(&gyroBias[0], &accelBias[0]);
	magInit();
//	
//	 writeByteSPI(0x25, 0x0C);
//  // Point save 0 register at AK8963's control 2 (soft reset) register
//  writeByteSPI(0x26, 0x0B);
//  // Send 0x01 to AK8963 via slave 0 to trigger a soft restart
//  writeByteSPI(0x63, 0x01);
//  // Enable simple 1-byte I2C reads from slave 0
//  writeByteSPI(0x27, 0x81);
//  // Point save 0 register at AK8963's control 1 (mode) register
//  writeByteSPI(0x26, 0x0A);
//  // 16-bit continuous measurement mode 1
//  writeByteSPI(0x63, 0x12);
//  // Enable simple 1-byte I2C reads from slave 0
//  writeByteSPI(0x27, 0x81);
//	
	getAres();
	getMres();
	getGres();
//#define MPU_InitRegNum 17

// uint8_t i = 0;
//    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
//        {0x80, MPUREG_PWR_MGMT_1},     // Reset Device
//        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
//        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
//        {BITS_DLPF_CFG_42HZ, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
//        {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
//        {0x08, MPUREG_ACCEL_CONFIG},   // +-4G
//        {0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
//        {0x30, MPUREG_INT_PIN_CFG},    //
//        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
//        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
//        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
//        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz
//        
//        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
//        //{0x09, MPUREG_I2C_SLV4_CTRL},
//        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay
// 
//        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
//        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
//        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte
// 
//        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
//        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
//        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
//        
//    };

// 
//    for(i=0; i<MPU_InitRegNum; i++) {
//        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
//        HAL_Delay(1);  //I2C must slow down the write speed, otherwise it won't work
//    }
// 
//    getAres();
//    getMres();
//    getGres();
    
    //AK8963_calib_Magnetometer();  //Can't load this function here , strange problem?
	
}

// Read the WHOAMI (WIA) register of the AK8963
// TODO: This method has side effects
uint8_t ak8963WhoAmI_SPI()
{
  uint8_t response, oldSlaveAddress, oldSlaveRegister, oldSlaveConfig;
  // Save state
  oldSlaveAddress  = readByteSPI(I2C_SLV0_ADDR);
  oldSlaveRegister = readByteSPI(I2C_SLV0_REG);
  oldSlaveConfig   = readByteSPI(I2C_SLV0_CTRL);


  // Set the I2C slave addres of AK8963 and set for read
  response = writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS|READ_FLAG);
  // I2C slave 0 register address from where to begin data transfer
  response = writeByteSPI(I2C_SLV0_REG, 0x00);
  // Enable 1-byte reads on slave 0
  response = writeByteSPI(I2C_SLV0_CTRL, 0x81);
	
	HAL_Delay(1);

  // Read WIA register
  response = writeByteSPI(WHO_AM_I_AK8963|READ_FLAG, 0x00);

  // Restore state
  writeByteSPI(I2C_SLV0_ADDR, oldSlaveAddress);
  writeByteSPI(I2C_SLV0_REG, oldSlaveRegister);
  writeByteSPI(I2C_SLV0_CTRL, oldSlaveConfig);
	
//	printf("add %d\n", oldSlaveAddress);
//	send_byte(oldSlaveAddress);

  return response;
}

void read_test(void)
{
	
}


void mpu9250_read_all(void)
{
//	uint8_t response[21];
//    int16_t bit_data;
//    float data;
//    int i;
//	
//	  float accelerometer_data[3];
//    float Temperature;
//    float gyroscope_data[3];
//    float Magnetometer[3];
// 
//    //Send I2C command at first
//    WriteReg(I2C_SLV0_ADDR,AK8963_ADDRESS|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
//    WriteReg(I2C_SLV0_REG, AK8963_XOUT_L); //I2C slave 0 register address from where to begin data transfer
//    WriteReg(I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
//    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
// 
//    //wait(0.001);
//    ReadRegs(ACCEL_XOUT_H,response,21);
//    //Get accelerometer value
//    for(i=0; i<3; i++) {
//        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
//        data=(float)bit_data;
//        accelerometer_data[i]=data * aRes;
//    }
//    //Get temperature
//    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
//    data=(float)bit_data;
//    Temperature=((data-21)/333.87)+21;
//    //Get gyroscop value
//    for(i=4; i<7; i++) {
//        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
//        data=(float)bit_data;
//        gyroscope_data[i-4]=data * gRes;
//    }
//    //Get Magnetometer value
//    for(i=7; i<10; i++) {
//        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
//        data=(float)bit_data;
//        Magnetometer[i-7]=data * mRes;
//    }

//		ax = accelerometer_data[0];
//		ay = accelerometer_data[1];
//		az = accelerometer_data[2];
//	
//		gx = gyroscope_data[0];
//		gy = gyroscope_data[1];
//		gz = gyroscope_data[2];
//	
//		mx = Magnetometer[0];
//		my = Magnetometer[1];
//		mz = Magnetometer[2];
//		temperature = Temperature;

   int16_t acc[3], gyro[3];
   readAccelData(acc);
	 readGyroData(gyro);
	 
	 ax = ((float)(acc[0])) * aRes;
	 ay = ((float)(acc[1])) * aRes;
	 az = ((float)(acc[2])) * aRes;
	 
	 gx = ((float)(gyro[0])) * gRes;
	 gy = ((float)(gyro[1])) * gRes;
	 gz = ((float)(gyro[2])) * gRes;
}

