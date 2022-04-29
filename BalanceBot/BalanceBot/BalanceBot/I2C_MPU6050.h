/*
 * I2C_MPU6050.h
 *
 * Created: 4/15/2022 3:04:52 PM
 *  Author: Matthew
 */ 


#ifndef I2C_MPU6050_H_
#define I2C_MPU6050_H_

#define F_CPU 16000000UL

#include <util/delay.h>
#include "i2cmaster.h"
#include "MPU6050_REGISTERS.h"
#include "LED_TOGGLE.h"
#include <math.h>

struct MPU6050Vals{
	uint16_t rawXaccel, rawZaccel, rawYGyro;
	float Xaccel, Zaccel, YGyro, theta, PreviousTheta, GyroTheta, dt, alpha, tau, CurrentTheta;
	}MPU6050;
	


// Configure MPU6050 settings
void Init_MPU6050();

//Writes val to Reg
void I2C_WriteConfigure(uint8_t reg, uint8_t val);

void I2C_ReadSensorVals();

//Convert Sensor Values//Convert Sensor Values
void ConvertSensorVals();


#endif /* I2C_MPU6050_H_ */