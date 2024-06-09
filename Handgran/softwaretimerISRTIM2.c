/*
 * softwaretimerISRTIM2.c
 *
 * Created: 02/02/2022 14:01:39
 *  Author: Katharina Böhm-Klamt
 */ 
#include "softwaretimerISRTIM2.h"

uint8_t soft_async_stop;


#define soft_stop(E,N)  E ? ( soft_async_stop |= ( 1<<N )) : ( soft_async_stop &= ~( 1<<N ))
#define check_stop(N) (soft_async_stop & (1<<N) )

void soft_timer_reset(uint8_t n) { soft_async_stop &= ~( 1<<n );  soft_s_zeiten[n].ms=0; }

uint8_t soft_timer_is_stop(uint8_t n)   // flag of async timer n set?
{
	return check_stop(n);
}

void soft_chk_do()
{
	uint8_t _bit = 0x01;
	for (uint8_t _n=0; _n<SOFT_MAX_TIMER; _n++)
	{
		if (soft_enable & _bit)
		{
			if (soft_enable_fasync & _bit)
			{
				if (soft_async_stop & _bit)
				{
					soft_s_zeiten[_n].fasync();
					soft_async_stop &= ~(_bit);
				}
			}
		}
		_bit <<= 1;
	}	// for _n
}

ISR(TIMER2_OVF_vect)
{
	uint8_t _bit = 0x01;
	for (uint8_t _n=0; _n<SOFT_MAX_TIMER; _n++)
	{
		if (soft_enable & _bit)
		{
			if (++soft_s_zeiten[_n].ms >= soft_s_zeiten[_n].ms_max)
			{
				soft_s_zeiten[_n].ms = 0;
				if (soft_enable_fsync & _bit)
				{
					soft_s_zeiten[_n].fsync();
				}
				if (soft_enable_fasync & _bit)
				{
					soft_async_stop |= _bit;
				}
			}
		}
		_bit <<= 1;
	}
	
	
}




//====================================================================================================
