#include "MotorPWM.h"

/**
 * Function MotorPWM_Init initializes the motor PWM on Timer 1 for PWM based voltage control of the motors.
 * The Motor PWM system shall initialize in the disabled state for safety reasons. You should specifically enable
 * Motor PWM outputs only as necessary.
 * @param [uint16_t] MAX_PWM is the maximum PWM value to use. This controls the PWM frequency.
 */
void Motor_PWM_Init( uint16_t MAX_PWM ){
 
    DDRB &= 0b10011111;         //Set pins PB5 and PB6 to low

    //Set Motor direction pins
    DDRB |= (1<<DDB1) | ( 1<< DDB2);

    //Setting Output Compare Match Mode to Toggle on compare match
    TCCR1A |= (1<<COM1A1);
    TCCR1A |= (1<<COM1B1);


    Set_MAX_Motor_PWM(MAX_PWM);


    //Set mode to Phase and frequency correct and clock prescalar to 256
    // Sets Max ICR value to 8191
    TCCR1B |= (1<<WGM13) | (1<<CS10);
}

/**
 * Function MotorPWM_Enable enables or disables the motor PWM outputs.
 * @param [bool] enable (true set enable, false set disable)
 */
void Motor_PWM_Enable( bool enable ){
    
    if(enable){
        DDRB |= (1<<DDB5) | (1<<DDB6);
        return;
    }
    DDRB &= 0b10011111;     //Set pins PB5 and PB6 to low
}

/**
 * Function Is_Motor_PWM_Enabled returns if the motor PWM is enabled for output.
 * @param [bool] true if enabled, false if disabled
 */
bool Is_Motor_PWM_Enabled(){
    if((DDRB & 0b01100000 ) == 0b01100000){return true;}
    return false;
}

/**
 * Function Motor_PWM_Left sets the PWM duty cycle for the left motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Left( int16_t pwm ){
    if(pwm < 0){
        PORTB |= (1<<PORTB2);
    }else{
        PORTB &= 0b11111011;
    }
    pwm = abs(pwm);
    union {
        uint16_t word;
        uint8_t arr[2];
    }WordToBytes;
    
    WordToBytes.word = pwm; // / ICR_Split.word;

    OCR1BH = WordToBytes.arr[1];
    OCR1BL = WordToBytes.arr[0];
}

/**
 * Function Motor_PWM_Right sets the PWM duty cycle for the right motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Right( int16_t pwm ){
    // FOR MOTOR PHASE PINS:
        // LOW IS FORWARD
        // HIGH IS REVERSE
            if(pwm < 0){
                PORTB |= (1<<PORTB1);
            }else{
                PORTB &= 0b11111101;
            }
    pwm = abs(pwm);
    union {
        uint16_t word;
        uint8_t arr[2];
    }WordToBytes;
    
    WordToBytes.word = pwm; /// ICR_Split.word;

    OCR1AH = WordToBytes.arr[1];
    OCR1AL = WordToBytes.arr[0];
}

/**
 * Function Get_Motor_PWM_Left returns the current PWM duty cycle for the left motor. If disabled it returns what the
 * PWM duty cycle would be.
 * @return [int16_t] duty-cycle for the left motor's pwm
 */
int16_t Get_Motor_PWM_Left(){
    union {
        uint16_t word;
        uint8_t arr[2];
    }WordToBytes;

    WordToBytes.arr[1] = OCR1BH;
    WordToBytes.arr[0] = OCR1BL;
    int16_t pwm = WordToBytes.word;
    if((PORTB & 0b00000100) == 0b00000100){
        pwm = -pwm;
    }
    return pwm;
}

/**
 * Function Get_Motor_PWM_Right returns the current PWM duty cycle for the right motor. If disabled it returns what the
 * PWM duty cycle would be.
 * @return [int16_t] duty-cycle for the right motor's pwm
 */
int16_t Get_Motor_PWM_Right(){
    union {
        uint16_t word;
        uint8_t arr[2];
    }WordToBytes;

    WordToBytes.arr[1] = OCR1AH;
    WordToBytes.arr[0] = OCR1AL;
    int16_t pwm = WordToBytes.word;
    if((PORTB & 0b00000010) == 0b00000010){
        pwm = -pwm;
    }

    return pwm;
}

/**
 * Function Get_MAX_Motor_PWM() returns the PWM count that corresponds to 100 percent duty cycle (all on), this is the
 * same as the value written into ICR1 as (TOP).
 */
uint16_t Get_MAX_Motor_PWM(){
    union {
                    uint16_t word;
                    uint8_t arr[2];
                }ICR_Split;

    ICR_Split.arr[1] = ICR1H;
    ICR_Split.arr[0] = ICR1L;

    return ICR_Split.word;
}

/**
 * Function Set_MAX_Motor_PWM sets the maximum pwm count. This function sets the timer counts to zero because
 * the ICR1 can cause undesired behaviors if change dynamically below the current counts.  See page 128 of the
 * atmega32U4 datasheat.
 */
void Set_MAX_Motor_PWM( uint16_t MAX_PWM ){
    union {
                    uint16_t word;
                    uint8_t arr[2];
                }ICR_Split;
    ICR_Split.word = MAX_PWM;
    ICR1H = ICR_Split.arr[1];
    // uint16_t test = (ICR_Split.arr[1] << 8) | (ICR_Split.arr[0] << 0);
    // if(test == MAX_PWM){LED_ON();}
    ICR1L = ICR_Split.arr[0];
}