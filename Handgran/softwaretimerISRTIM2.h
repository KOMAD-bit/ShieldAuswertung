/*
 * softwaretimerISRTIM2.h
 *
 * Created: 02/02/2022 14:00:17
 *  Author: Katharina Böhm-Klamt
 */ 

#include <avr/interrupt.h>

#ifndef SOFTWARETIMERISRTIM2_H_
#define SOFTWARETIMERISRTIM2_H_


void soft_timer_reset(uint8_t n);		
uint8_t soft_timer_is_stop(uint8_t n);  
void soft_chk_do();			

#define SOFT_MAX_TIMER 4

// Flags: [ 7 6 7 4 3 AltDisp Led ]
uint8_t soft_enable;			// Enable Timer (master)
uint8_t soft_enable_fsync;		// Enable sync function
uint8_t soft_enable_fasync;		// Enable async function


struct SOFT_ZEITEN
{
	uint16_t 	ms;				
	uint16_t	ms_max;			
	void		(*fsync)();		
	void		(*fasync)();	
} soft_s_zeiten[SOFT_MAX_TIMER];


#endif /* SOFTWARETIMERISRTIM2_H_ */