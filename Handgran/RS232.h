/*
 * RS232.h
 *
 * Created: 13/03/2022 22:22:15
 *  Author: Katharina Böhm-Klamt
 */ 


#define RS232_H_





#include <avr/interrupt.h>

//****************************************************************************************************

void my_delay(uint16_t __ms);

#define IRCMD_HEADER				0xF0
#define PROTCOLL_INVERT_8

struct S_SER_IN
{
	uint8_t     cmd;            // Serieller Input
	uint8_t     id;             // Serieller Input
	uint8_t		val;			// Serieller Input valid
}; 


void RX_ClearBuffer();
uint8_t RX_GetNextCommand(struct S_SER_IN* serial_in);
uint8_t RX_GetNext(uint8_t* in);	

#ifdef DEBUG_STUDIO
	void RXPush(uint8_t val);
#endif 



uint8_t TX_BufferEmpty();	
void TX_SendChar(uint8_t c); 
void TX_SendCommand(uint8_t cmd, uint8_t id, uint8_t val);


//====================================================================================================