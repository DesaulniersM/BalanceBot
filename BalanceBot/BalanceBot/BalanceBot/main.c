/*
 * BalanceBot.c
 *
 * Created: 4/15/2022 1:25:04 PM
 * Author : Matthew
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/fuse.h>

#include "LED_TOGGLE.h"
#include "I2C_MPU6050.h"
#include "BalanceBot.h"
#include "MotorPWM.h"
#include "Matrix.h"


#include "Timing.h"
#include "Task_Management.h"

//Function Prototypes
void Task_Balance();			// Runs the entire control loop of the balance bot
void Task_Calibrate();		// Handles calibration routines

//Task Declarations
Task_t task_Balance;			// Runs the entire control loop of the balance bot
Task_t task_Calibrate;		// Handles calibration routines
// Super Global

float Kp = 3015; //1500 875
float Kd = 0; 
float Ki = 0;
float error = 0;
float errorSum = 0;
float CalibratedBalancePoint;
float EEMEM EEPROM_ADDRESS;
int i = 0;
uint16_t pwm;


	
	

__fuse_t __fuse __attribute__((section (".fuse"))) =
{
	.low = 0b01011111,
	.high = 0b11010111,
	.extended = 0b11110100,
};

void InitializeModules(){
	
	initialize_LED(DDRF, PORTF0);
	initialize_LED(DDRF, PORTF1);
	
	BalanceBotInit();
	Initialize_Timing();		//Starts Timer 0, used to count millis/micros to keep track of tasks and other timing functions
	Motor_PWM_Init();
	Initialize_Task(&task_Balance, MPU6050.dt, Task_Balance);
	Initialize_Task(&task_Calibrate, -1, Task_Calibrate);
	Init_MPU6050();
	
	
}

int main(void)
{
	

	InitializeModules();
	CalibratedBalancePoint = eeprom_read_float(&EEPROM_ADDRESS);
	task_Balance.is_active = true;
	task_Calibrate.is_active = false;



while (1)
{
	Task_Run_If_Ready(&task_Balance);
	Task_Run_If_Ready(&task_Calibrate);	
}
return 0;
}


// **********Task functions********** 

void Task_Balance(){
	
	
	
	
	// Take Measurement
	I2C_ReadSensorVals();
	ConvertSensorVals();
	BalanceBot.y[0][0] = MPU6050.CurrentTheta + M_PI;//took away pi
	
	// Estimate States
	//x_est = Ad * x_est + Bd * ulast
	BalanceBot.x_est = add( multiply(BalanceBot.Ad, BalanceBot.x_est, 3,3,3,1), multiply(BalanceBot.Bd, BalanceBot.uLast,3,1,1,1) ,3,1);
	//y_est = C * x_est
	BalanceBot.y_est[0][0] = BalanceBot.x_est[0][1];
	//measError = y - y_est
	BalanceBot.measError = subtract(BalanceBot.y, BalanceBot.y_est, 1,1);
	//x_est = x_est + L*measError
	BalanceBot.x_est = add(BalanceBot.x_est, multiply(BalanceBot.L, BalanceBot.measError, 3,1,1,1), 3,1);
	//Controller
	//xd = desired state
	//stateError = xd - x_est
	BalanceBot.stateError = subtract(BalanceBot.xd, BalanceBot.x_est, 3,1);
	//u = K*stateError
	BalanceBot.u = multiply(BalanceBot.K, BalanceBot.stateError, 1,3,3,1);
	//ulast = u
	BalanceBot.uLast = BalanceBot.u;
	/*uLast = 3;*/
	uint16_t pwm = ( BalanceBot.uLast[0][0] / 6.0) * 255;
	Motor_PWM_Left(200);
	Motor_PWM_Right(200);
	Motor_PWM_Enable(true);	
	
	MPU6050.PreviousTheta = MPU6050.CurrentTheta;
	
	
	/*uint16_t pwm = 100;*/
	//pwm = (ulast / 6) * 255
// 	error = ((4.3*M_PI/ 180.0) + 0.0) - MPU6050.CurrentTheta;
// 	//errorSum += error;
// 	//if(errorSum)
// 	int16_t pwm = Kp*(error) - Kd*(MPU6050.CurrentTheta-MPU6050.PreviousTheta)/MPU6050.dt + Ki*(errorSum)*MPU6050.dt;
	

	
	
	
}

void Task_Calibrate(){
	
	

	I2C_ReadSensorVals();
	ConvertSensorVals();
	
	float AverageTheta = 0;
	int i = 1;
	while (i < 1000)
	{
		I2C_ReadSensorVals();
		ConvertSensorVals();
		
		AverageTheta = (AverageTheta * (i - 1) + (MPU6050.CurrentTheta)) / i;
		MPU6050.PreviousTheta = MPU6050.CurrentTheta;
		i++;
	}

	float BalanceTheta = AverageTheta;
	


	Motor_PWM_Left(100);
	Motor_PWM_Right(100);
	Motor_PWM_Enable(true);
		
 	if(eeprom_is_ready()){
 	eeprom_update_float(&EEPROM_ADDRESS, BalanceTheta);
 	}
 	
		_delay_ms(10);
 	CalibratedBalancePoint = eeprom_read_float(&EEPROM_ADDRESS);
 	
 
	
 	if ( fabs(CalibratedBalancePoint) < 1.72)
 	{
 		Motor_PWM_Enable(false);
 	}
	

	
}