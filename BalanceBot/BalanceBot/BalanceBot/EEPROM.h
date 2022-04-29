/*
 * EEPROM.h
 *
 * Created: 4/17/2022 1:55:05 PM
 *  Author: Matthew
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/io.h>

// CAUTION !!!!!!!!!!!!!!!!!!!!!!!!
// This library will currently only work for the first 256 bytes of memory. There is no reason for this other than a need to finish quickly and only store 1 float 
// for this project. When the Address inputs are of type uint16_t this problem will have been solved.

void EEPROM_Init();

void EEPROM_WriteByte(unsigned int Address, uint8_t Data);

uint8_t EEPROM_ReadByte(unsigned int Address);

void EEPROM_WriteFloat(unsigned int StartAddress, float Data);

float EEPROM_ReadFloat(unsigned int StartAddress);







#endif /* EEPROM_H_ */