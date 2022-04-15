#include "Filter.h"
#include <avr/io.h>
#define initialize_LED()  DDRC |= (1 << DDC7)    // Make pin 13 be an output.

#define LED_ON()      PORTC |= (1 << PORTC7)   // Turn the LED on.

#define LED_TOGGLE()  PORTC ^= (1 << PORTC7)   // Toggles the led state

#define LED_OFF()     PORTC &= ~(1 << PORTC7)  // Turn the LED off.
/**
 * Function Filter_Init initializes the filter given two float arrays and the order of the filter.  Note that the
 * size of the array will be one larger than the order. (First order systems have two coefficients).
 *
 *  1/A_0*( SUM( B_i * input_i )  -   SUM( A_i * output_i) )
 *         i=0..N                    i=1..N
 *
 *  Note a 5-point moving average filter has coefficients:
 *      numerator_coeffs   = { 5 0 0 0 0 };
 *      denominator_coeffs = { 1 1 1 1 1 };
 *      order = 4;
 *
 * @param p_filt pointer to the filter object
 * @param numerator_coeffs The numerator coefficients (B/beta traditionally)
 * @param denominator_coeffs The denominator coefficients (A/alpha traditionally)
 * @param order The filter order
 */
void  Filter_Init ( Filter_Data_t* p_filt, float* numerator_coeffs, float* denominator_coeffs, uint8_t order ){

    rb_initialize_F(&p_filt->numerator);
    rb_initialize_F(&p_filt->denominator);
    rb_initialize_F(&p_filt->in_list);
    rb_initialize_F(&p_filt->out_list);

    for(int i = 0; i <= order; i++ ){
        rb_push_back_F(&(p_filt->numerator), numerator_coeffs[i]);
        rb_push_back_F(&(p_filt->denominator), denominator_coeffs[i]);
        rb_push_back_F(&(p_filt->in_list), 0.0f);
        rb_push_back_F(&(p_filt->out_list), 0.0f);
	}
}

/**
 * Function Filter_ShiftBy shifts the input list and output list to keep the filter in the same frame. This especially
 * useful when initializing the filter to the current value or handling wrapping/overflow issues.
 * @param p_filt
 * @param shift_amount
 */
void  Filter_ShiftBy( Filter_Data_t* p_filt, float shift_amount ){
    float val;
    for (int i = 0; i<rb_length_F(&p_filt->in_list); i++){
        val = rb_pop_front_F(&p_filt->in_list) + shift_amount;
        rb_push_back_F(&p_filt->in_list, val);
        val = rb_pop_front_F(&p_filt->out_list) + shift_amount;
        rb_push_back_F(&p_filt->out_list, val);
    }
    //return;
}

/**
 * Function Filter_SetTo sets the initial values for the input and output lists to a constant defined value. This
 * helps to initialize or re-initialize the filter as desired.
 * @param p_filt Pointer to a Filter_Data sturcture
 * @param amount The value to re-initialize the filter to.
 */
void Filter_SetTo( Filter_Data_t* p_filt, float amount ){
    for (int i = 0; i<rb_length_F(&p_filt->in_list); i++){
        rb_pop_front_F(&p_filt->in_list);
        rb_push_back_F(&p_filt->in_list, amount);
        rb_pop_front_F(&p_filt->out_list);
        rb_push_back_F(&p_filt->out_list, amount);
    }
    //return;
}

/**
 * Function Filter_Value adds a new value to the filter and returns the new output.
 * @param p_filt pointer to the filter object
 * @param value the new measurement or value
 * @return The newly filtered value
 */
float Filter_Value( Filter_Data_t* p_filt, float value){
    // float newOut = 0;
    // float imd = 0;
    
    // //float test = rb_pop_back_F(&p_filt->in_list);
    // //if(test == 4.0){LED_TOGGLE();}

    // // pop oldest input
    // rb_pop_back_F(&p_filt->in_list);
    
    // rb_push_front_F(&p_filt->in_list, value);

    // for (int i = 0; i<rb_length_F(&p_filt->in_list); i++){
    //     float N = rb_get_F(&(p_filt->numerator), i);
    //     float I = rb_get_F(&(p_filt->in_list), i);
    //     if(i==0){
    //         imd += (N * I);
    //     } else{
    //         float D = rb_get_F(&(p_filt->denominator), i); //was i-1
    //         float O = rb_get_F(&(p_filt->out_list), i-1);
    //         imd += ((N * I) - (D * O));
    //     }
    // }
    // newOut = imd / rb_get_F(&(p_filt->denominator), 0);
    // rb_push_front_F(&(p_filt->out_list), newOut);
    // rb_pop_back_F(&(p_filt->out_list));
    // return newOut;


    rb_push_front_F(&p_filt->in_list, value);
    float newOut = rb_get_F(&p_filt->numerator,0) * rb_get_F(&p_filt->in_list,0);

    for( uint8_t i = 1; i < rb_length_F(&p_filt->numerator); i ++ )
    {
        newOut += rb_get_F(&p_filt->numerator,i) * rb_get_F(&p_filt->in_list,i);
        newOut -= rb_get_F(&p_filt->denominator,i) * rb_get_F(&p_filt->out_list,i-1);
    }

    newOut /= rb_get_F(&p_filt->denominator,0);

    rb_push_front_F(&p_filt->out_list, newOut);

    return newOut;
}

/**
 * Function Filter_Last_Output returns the most up-to-date filtered value without updating the filter.
 * @return The latest filtered value
 */
float Filter_Last_Output( Filter_Data_t* p_filt ){
//    float lastOut = rb_pop_back_F(&p_filt->out_list);
//    rb_push_back_F(&p_filt->out_list, lastOut);
    float lastOut = rb_get_F(&p_filt->out_list, 0);
    return lastOut;
}
