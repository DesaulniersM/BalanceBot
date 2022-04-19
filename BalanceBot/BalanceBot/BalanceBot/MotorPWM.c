#include "MotorPWM.h"

/**
 * Function MotorPWM_Init initializes the motor PWM on Timer 1 for PWM based voltage control of the motors.
 * The Motor PWM system shall initialize in the disabled state for safety reasons. You should specifically enable
 * Motor PWM outputs only as necessary.
 * @param [uint16_t] MAX_PWM is the maximum PWM value to use. This controls the PWM frequency.
 */

uint8_t MAX_PWM_COUNT = 255;			// Maximum PWM Count for all timers


void Motor_PWM_Init( ){
 
    DDRB &= 0b10011111;         //Set pins PB5 (AIN1) and PB6 (BIN1) to low
	DDRC &= 0b01111111;			//Set pin PC7 (AIN2) to low
	DDRD &= 0b01111111;			//Set pin PD7 (BIN2) to low
	
    

    //Setting Output Compare Match Mode to Toggle on compare match
    TCCR1A |= (1<<COM1A1);
    TCCR1A |= (1<<COM1B1);
	
	
	TCCR4A |= (1<<COM4A1);
	TCCR4C |= (1<<COM4D1);


    Set_MAX_Motor_PWM();


    //Set mode to Phase and frequency correct and clock prescalar to 256
    // Sets Max ICR value to 8191
    TCCR1B |= (1<<WGM13) | (1<<CS10);
	
	//Set mode to Phase and frequency correct and clock prescalar to 256
	TCCR4A |= (1<<PWM4A);
	TCCR4B |= (1<<CS40);
	TCCR4C |= (1<<PWM4D);
	TCCR4D |= (1<<WGM40);
}



/**
 * Function MotorPWM_Enable enables or disables the motor PWM outputs.
 * @param [bool] enable (true set enable, false set disable)
 */
void Motor_PWM_Enable( bool enable ){
    
    if(enable){
        DDRB |= (1<<DDB5) | (1<<DDB6);
		DDRC |= (1<<DDC7);
		DDRD |= (1<<DDD7);
        return;
    }
    DDRB &= 0b10011111;         //Set pins PB5 (AIN1) and PB6 (BIN1) to low
    DDRC &= 0b01111111;			//Set pin PC7 (AIN2) to low
    DDRD &= 0b01111111;			//Set pin PD7 (BIN2) to low
}

/**
 * Function Is_Motor_PWM_Enabled returns if the motor PWM is enabled for output.
 * @param [bool] true if enabled, false if disabled
 */
bool Is_Motor_PWM_Enabled(){
    if(((DDRB & 0b01100000 ) == 0b01100000) && (DDRC & 0b10000000) && (DDRD & 0b10000000)){return true;}
    return false;
}

/**
 * Function Motor_PWM_Left sets the PWM duty cycle for the left motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Left( int16_t pwm ){
	int16_t sign = pwm;
	pwm = abs(pwm);
	
	if(pwm > MAX_PWM_COUNT){pwm = MAX_PWM_COUNT;}
	union {
		uint16_t word;
		uint8_t arr[2];
	}WordToBytes;
	
	WordToBytes.word = pwm; // / ICR_Split.word;
	
	//if (WordToBytes.arr[0] != 200){WordToBytes.word = 100;}

    if(sign < 0){
		
        OCR4A = WordToBytes.arr[0];
		//if(OCR4A == 200){LED_ON(DDRF, PORTF0);}
		OCR1AH = 0;
		OCR1AL = 0;
    }else{
		
        OCR4A = 0;
        OCR1AH = 0;
        OCR1AL = WordToBytes.arr[0];
		//if(OCR1AL == 200){LED_ON(DDRF, PORTF0);}
    }
}

/**
 * Function Motor_PWM_Right sets the PWM duty cycle for the right motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Right( int16_t pwm ){
	int16_t sign = pwm;
	pwm = abs(pwm);
	
	if(pwm>MAX_PWM_COUNT){pwm = MAX_PWM_COUNT;}
	union {
		uint16_t word;
		uint8_t arr[2];
	}WordToBytes;
	
	WordToBytes.word = pwm; // / ICR_Split.word;
	if(sign < 0){
		
		OCR4D = WordToBytes.arr[0];
		OCR1BH = 0;
		OCR1BL = 0;
		
		}else{
		OCR4D = 0;
		OCR1BH = 0;
		OCR1BL = WordToBytes.arr[0];
		
	}
}

/**
 * Function Set_MAX_Motor_PWM sets the maximum pwm count. This function sets the timer counts to zero because
 * the ICR1 can cause undesired behaviors if change dynamically below the current counts.  See page 128 of the
 * atmega32U4 datasheat.
 */
void Set_MAX_Motor_PWM( ){
    ICR1L = MAX_PWM_COUNT;		//Timer 1 TOP
	OCR4C = MAX_PWM_COUNT;		//Timer 4 TOP
}