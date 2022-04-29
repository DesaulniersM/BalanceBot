/*
 * EEPROM.c
 *
 * Created: 4/17/2022 1:59:34 PM
 *  Author: Matthew
 */ 


#include "EEPROM.h"


void EEPROM_Init(){
	
}

void EEPROM_WriteByte(unsigned int Address, unsigned char Data){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set up address and Data Registers */
	EEAR = Address;
	EEDR = Data;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
}

uint8_t EEPROM_ReadByte(unsigned int Address){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set up address register */
	EEAR = Address;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from Data Register */
	return EEDR;
}

void EEPROM_WriteFloat(unsigned int StartAddress, float* Data){
	void *DataPointer = Data;
	for (uint8_t i = 0; i<4; i++)
	{
		EEPROM_WriteByte(StartAddress + i, DataPointer);
		DataPointer++;
	}
}

float EEPROM_ReadFloat(unsigned int StartAddress){
	float RegisterValue;
	void *DataPointer = &RegisterValue;
	for(uint8_t i = 0; i<4; i++){
		*DataPointer = EEPROM_ReadByte(StartAddress + i);
		DataPointer++;
	}
}