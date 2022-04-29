/*
 * I2C_MPU6050.c
 *
 * Created: 4/15/2022 3:06:13 PM
 *  Author: Matthew
 */ 

#include "I2C_MPU6050.h"

// Configure MPU6050 settings
void Init_MPU6050(){
	i2c_init();     // init I2C interface
	_delay_ms(200);  // Wait for 200 ms.
	//
	MPU6050.tau = 0.004; //0.75 0.007
	float ControlUpdatePeriod = .001;		// 1kHz Max as per MPU6050 sampling limitations
	MPU6050.alpha = ((MPU6050.tau) / (MPU6050.tau + ControlUpdatePeriod));
	MPU6050.PreviousTheta = 0;
	MPU6050.dt = ControlUpdatePeriod;
	//
	unsigned char ret = i2c_start((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_WRITE);       // set device address and write mode
	if ( ret )
	{
		/* failed to issue start condition, possibly no device found */
		i2c_stop();
		LED_ON(PORTF, DEBUG_LED4);    // Turn on the red LED immediately to signal the read is not ok
		while(1) {;;}  // lock program here as sensor init failed
	}else{
		/* issuing start condition ok, device accessible */
		i2c_write(MPU6050_RA_PWR_MGMT_1); // go to register 107
		i2c_write(0x00); // set value to 0000 0000 and wake up sensor
		i2c_stop();      // set stop condition = release bus
		
		I2C_WriteConfigure(MPU6050_RA_SMPLRT_DIV, 0x08); // set value to 0000 1000 for 1000 Hz
		I2C_WriteConfigure(MPU6050_RA_ACCEL_CONFIG, 0x08); // set value to 0000 1000 for 4g, normal line tension is 2,7g
		I2C_WriteConfigure(MPU6050_RA_GYRO_CONFIG, 0x10); // set value to 0001 0000 for 1000 (deg/s) (Motor Max RPM is 100)
		I2C_WriteConfigure(MPU6050_RA_FIFO_EN, 0xF8); // set value to 1111 1000 for all sensors not slave
		I2C_WriteConfigure(MPU6050_RA_USER_CTRL,0x40); // set value to 0100 0000 FIFO enable
		
	}
	}

//Writes val to Reg
void I2C_WriteConfigure(uint8_t reg, uint8_t val){
			i2c_start( (MPU6050_ADDRESS_AD0_LOW<<1) + I2C_WRITE);
			i2c_write(reg); 
			i2c_write(val); 
			i2c_stop();      // set stop condition = release bus
	}

void I2C_ReadSensorVals(){

//Read X accel
i2c_start_wait((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_WRITE);    // set device address and write mode
i2c_write(MPU6050_RA_ACCEL_XOUT_H);                      // ACCEL_XOUT
i2c_rep_start((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_READ);      // set device address and read mode
MPU6050.rawXaccel = i2c_readAck();               // read one intermediate byte
MPU6050.rawXaccel = (MPU6050.rawXaccel<<8) | i2c_readNak(); // read last byte
i2c_stop();

//Read Z accel
i2c_start_wait((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_WRITE);    // set device address and write mode
i2c_write(MPU6050_RA_ACCEL_ZOUT_H);                      // ACCEL_ZOUT
i2c_rep_start((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_READ);      // set device address and read mode
MPU6050.rawZaccel = i2c_readAck();               // read one intermediate byte
MPU6050.rawZaccel = (MPU6050.rawZaccel<<8) | i2c_readNak(); // read last byte
i2c_stop();


//Read Y gyro

i2c_start_wait((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_WRITE);    // set device address and write mode
i2c_write(MPU6050_RA_GYRO_YOUT_H);                      // GYRO_YOUT
i2c_rep_start((MPU6050_ADDRESS_AD0_LOW<<1) + I2C_READ);      // set device address and read mode
MPU6050.rawYGyro = i2c_readAck();               // read one intermediate byte
MPU6050.rawYGyro = (MPU6050.rawYGyro<<8) | i2c_readNak(); // read last byte
i2c_stop();
}

//Convert Sensor Values//Convert Sensor Values
void ConvertSensorVals(){
	float RAD_TO_DEG = 180.0 / M_PI;
	
	MPU6050.Xaccel = (((int16_t) MPU6050.rawXaccel) / 32767.0) * 4.0;
	MPU6050.Zaccel = (((int16_t) MPU6050.rawZaccel) / 32767.0) * 4.0;
	
	MPU6050.YGyro = (((int16_t) MPU6050.rawYGyro) / 32767.0) * 1000.0;
	MPU6050.GyroTheta = MPU6050.YGyro * (1.0 / RAD_TO_DEG) * MPU6050.dt;
	
	MPU6050.theta = atan2((double) MPU6050.Xaccel, (double) MPU6050.Zaccel);
	
	MPU6050.CurrentTheta = (MPU6050.alpha * (MPU6050.PreviousTheta + MPU6050.GyroTheta )) + ((1 - MPU6050.alpha)*(MPU6050.theta));
	/*MPU6050.CurrentTheta= MPU6050.theta;*/
}