/*
 * RS232.c
 *
 * Created: 13/03/2022 22:23:27
 *  Author: Katharina Böhm-Klamt
 */ 
#include "RS232.h"

#define IRcomm_on()  TCCR1B = ( (1<<WGM12) | (1<<CS10) )
#define  IRon() TCCR1A |= (1<<COM1B0)
#define IRoff() TCCR1A &=~(1<<COM1B0)
#define IRcomm_off() TCCR1B = ( (1<<WGM12) | (1<<CS10) )

#define RX_BUFFER_SIZE 64
#define RX_BUFFER_MASK ( RX_BUFFER_SIZE-1 )
#define RX_BUFFER_EMPTY 1


#ifdef  PROTCOLL_CHECKSUM_4

struct USART_buffer
{
	uint8_t data[RX_BUFFER_SIZE];		// data bytes
	uint8_t ind_read;					// actual read position
	uint8_t ind_read_1;					// actual read position+1
	uint8_t ind_read_2;					// actual read position+2
	uint8_t ind_read_3;					// actual read position+3
	uint8_t ind_write;					// actual write position
	uint8_t cnt;						// anzahl bytes in buffer
} rx_buffer = {{},0,1,2,3,0,0};

ISR(USART_RXC_vect)
{
	uint8_t next = ( rx_buffer.ind_write+1 ) & RX_BUFFER_MASK;
	if ( (UCSRA & ( (1<<FE ) | (1<<PE ) | (1<<DOR) ) ) ==0 )
	{
		if (next == rx_buffer.ind_read)  
		{
			rx_buffer.ind_read = rx_buffer.ind_read_1;		// alle anderen +1
			rx_buffer.ind_read_1 = rx_buffer.ind_read_2;	// alle anderen +1
			rx_buffer.ind_read_2 = rx_buffer.ind_read_3;	// alle anderen +1
			rx_buffer.ind_read_3 = ( rx_buffer.ind_read_3+1)  & RX_BUFFER_MASK;
			rx_buffer.cnt--;	
		}
		rx_buffer.data[rx_buffer.ind_write]=UDR0;
		rx_buffer.ind_write = next;
		rx_buffer.cnt ++;
	}
	else next = UDR;  
}

uint8_t RX_GetNext(uint8_t* in)
{
	cli();	
	uint8_t err =0;
	if (rx_buffer.ind_write == rx_buffer.ind_read)
	{
		err= RX_BUFFER_EMPTY;
	}
	else
	{
		*in = rx_buffer.data[rx_buffer.ind_read];		
		rx_buffer.ind_read = rx_buffer.ind_read_1;		// alle anderen +1
		rx_buffer.ind_read_1 = rx_buffer.ind_read_2;	// alle anderen +1
		rx_buffer.ind_read_2 = rx_buffer.ind_read_3;	// alle anderen +1
		rx_buffer.ind_read_3 = ( rx_buffer.ind_read_3 +1)  & RX_BUFFER_MASK;
		rx_buffer.cnt --;	
	}
	sei();
	return err;
}

void RX_ClearBuffer()
{
	cli();
	rx_buffer.ind_read =0;
	rx_buffer.ind_read_1 =1;
	rx_buffer.ind_read_2 =2;
	rx_buffer.ind_read_3 =3;
	rx_buffer.ind_write =0;
	rx_buffer.cnt =0;
	sei();
}

uint8_t RX_GetNextCommand(uint8_t* cmd, uint8_t* id, uint8_t* val)
{
	if (rx_buffer.cnt>=4) 
	{
		if (!RX_GetNext(cmd)) 
		{
			if ( (*cmd & 0xF8) == IRCMD_HEADER )		
			{
					
				uint8_t chksum = *cmd;
				chksum = chksum ^ rx_buffer.data[rx_buffer.ind_read];
				chksum = chksum ^ rx_buffer.data[rx_buffer.ind_read_1];
				chksum = chksum ^ rx_buffer.data[rx_buffer.ind_read_2];
				if (!chksum)	
				{
					RX_GetNext(id);
					RX_GetNext(val);
					RX_GetNext(&chksum);
					return 0;	!
				}
			}
		}
			
	}
	return RX_BUFFER_EMPTY;
}
#endif


#ifdef  PROTCOLL_INVERT_8
struct USART_buffer
{
	uint8_t data[RX_BUFFER_SIZE];		// data bytes
	uint8_t ind_read;					// actual read position
	uint8_t ind_read_1;					// actual read position+1
	uint8_t ind_read_2;					// actual read position+2
	uint8_t ind_read_3;					// actual read position+3
	uint8_t ind_read_4;					// actual read position+1
	uint8_t ind_read_5;					// actual read position+2
	uint8_t ind_read_6;					// actual read position+3
	uint8_t ind_read_7;					// actual read position+3
	
	uint8_t ind_write;					// actual write position
	uint8_t cnt;						// anzahl bytes in buffer
	} rx_buffer = {{},0,1,2,3,4,5,6,7,0,0};

	#ifdef DEBUG_STUDIO
	void RXPush(uint8_t val)
	{
		uint8_t next = ( rx_buffer.ind_write+1 ) & RX_BUFFER_MASK;
		
		{
			if (next == rx_buffer.ind_read)  
			{
				rx_buffer.ind_read = rx_buffer.ind_read_1;		// alle anderen +1
				rx_buffer.ind_read_1 = rx_buffer.ind_read_2;	// alle anderen +1
				rx_buffer.ind_read_2 = rx_buffer.ind_read_3;	// alle anderen +1
				rx_buffer.ind_read_3 = rx_buffer.ind_read_4;	// alle anderen +1
				rx_buffer.ind_read_4 = rx_buffer.ind_read_5;	// alle anderen +1
				rx_buffer.ind_read_5 = rx_buffer.ind_read_6;	// alle anderen +1
				rx_buffer.ind_read_6 = rx_buffer.ind_read_7;	// alle anderen +1
				rx_buffer.ind_read_7 = ( rx_buffer.ind_read_7+1)  & RX_BUFFER_MASK;
				rx_buffer.cnt--;	
			}
			rx_buffer.data[rx_buffer.ind_write]= val;
			rx_buffer.ind_write = next;
			rx_buffer.cnt ++;
		}
	}
	#endif
	
	ISR(USART_RXC_vect)
	{
		uint8_t next = ( rx_buffer.ind_write+1 ) & RX_BUFFER_MASK;
		if ( (UCSRA & ( (1<<FE ) | (1<<PE ) | (1<<DOR) ) ) ==0 )
		{
			if (next == rx_buffer.ind_read)  
			{
				rx_buffer.ind_read = rx_buffer.ind_read_1;		// alle anderen +1
				rx_buffer.ind_read_1 = rx_buffer.ind_read_2;	// alle anderen +1
				rx_buffer.ind_read_2 = rx_buffer.ind_read_3;	// alle anderen +1
				rx_buffer.ind_read_3 = rx_buffer.ind_read_4;	// alle anderen +1
				rx_buffer.ind_read_4 = rx_buffer.ind_read_5;	// alle anderen +1
				rx_buffer.ind_read_5 = rx_buffer.ind_read_6;	// alle anderen +1
				rx_buffer.ind_read_6 = rx_buffer.ind_read_7;	// alle anderen +1
				rx_buffer.ind_read_7 = ( rx_buffer.ind_read_7+1)  & RX_BUFFER_MASK;
				rx_buffer.cnt--;	
			}
			rx_buffer.data[rx_buffer.ind_write]=UDR;
			rx_buffer.ind_write = next;
			rx_buffer.cnt ++;
		}
		else next = UDR;  
	}

	uint8_t RX_GetNext(uint8_t* in)
	{
		cli();	
		uint8_t err =0;
		if (rx_buffer.ind_write == rx_buffer.ind_read)
		{
			err= RX_BUFFER_EMPTY;
		}
		else
		{
			*in = rx_buffer.data[rx_buffer.ind_read];		// erstes byte holen
			rx_buffer.ind_read = rx_buffer.ind_read_1;		// alle anderen +1
			rx_buffer.ind_read_1 = rx_buffer.ind_read_2;	// alle anderen +1
			rx_buffer.ind_read_2 = rx_buffer.ind_read_3;	// alle anderen +1
			rx_buffer.ind_read_3 = rx_buffer.ind_read_4;	// alle anderen +1
			rx_buffer.ind_read_4 = rx_buffer.ind_read_5;	// alle anderen +1
			rx_buffer.ind_read_5 = rx_buffer.ind_read_6;	// alle anderen +1
			rx_buffer.ind_read_6 = rx_buffer.ind_read_7;	// alle anderen +1
			rx_buffer.ind_read_7 = ( rx_buffer.ind_read_7 +1)  & RX_BUFFER_MASK;
			rx_buffer.cnt --;	
		}
		sei();
		return err;
	}

	void RX_ClearBuffer()
	{
		cli();
		rx_buffer.ind_read =0;
		rx_buffer.ind_read_1 =1;
		rx_buffer.ind_read_2 =2;
		rx_buffer.ind_read_3 =3;
		rx_buffer.ind_read_4 =4;
		rx_buffer.ind_read_5 =5;
		rx_buffer.ind_read_6 =6;
		rx_buffer.ind_read_7 =7;
		rx_buffer.ind_write =0;
		rx_buffer.cnt =0;
		sei();
	}

	uint8_t RX_GetNextCommand(struct S_SER_IN* serial_in)
	{
		uint8_t chk_ff_0;
		
		if (rx_buffer.cnt>=8) 
		{
			if (!RX_GetNext(&chk_ff_0)) 
			{
				if ( chk_ff_0  == 0xff )		
				{
					if (rx_buffer.data[rx_buffer.ind_read] == 0xff)
					{
						if (rx_buffer.data[rx_buffer.ind_read_1] == (uint8_t)~rx_buffer.data[rx_buffer.ind_read_2] )	
						{
							if (rx_buffer.data[rx_buffer.ind_read_3] == (uint8_t)~rx_buffer.data[rx_buffer.ind_read_4] ) 
							{
								if (rx_buffer.data[rx_buffer.ind_read_5] == (uint8_t)~rx_buffer.data[rx_buffer.ind_read_6] ) 
								{
									RX_GetNext(&chk_ff_0);
									RX_GetNext(&(serial_in->cmd));
									RX_GetNext(&chk_ff_0); 
									RX_GetNext(&(serial_in->id));
									RX_GetNext(&chk_ff_0); 
									RX_GetNext(&(serial_in->val));
									RX_GetNext(&chk_ff_0); 
									return 0;	
								} 							
							} 			
						}			
					}					
				}
			} // if getnext			
		} // if 8 bytes
		return RX_BUFFER_EMPTY;
	}
#endif

#define TX_BUFFER_SIZE 64
#define TX_BUFFER_MASK ( TX_BUFFER_SIZE-1 )
#define TX_BUFFER_EMPTY 1

struct USART_TX_buffer
{
uint8_t data[TX_BUFFER_SIZE];		// data bytes
uint8_t ind_read;					// actual read position
uint8_t ind_write;					// actual write position
} tx_buffer = {{},0,0};


ISR(USART_TXC_vect)
{
	if (tx_buffer.ind_write != tx_buffer.ind_read) 
	{
		UDR = tx_buffer.data[tx_buffer.ind_read];						
		tx_buffer.ind_read = ( tx_buffer.ind_read+1) & TX_BUFFER_MASK;	
	}
	else
	{
		IRcomm_off(); 
		IRoff();
	}
}

uint8_t TX_BufferEmpty()
{
	return (tx_buffer.ind_write == tx_buffer.ind_read);
}


void TX_SendChar(uint8_t c)
{
	uint8_t next = ( tx_buffer.ind_write+1 ) & TX_BUFFER_MASK;
	if (next == tx_buffer.ind_read) return;
	
	cli();	
	IRcomm_on();
	IRon();
	if ((tx_buffer.ind_write != tx_buffer.ind_read) || ((UCSRA & (1<<UDRE) )==0))	
	{
		tx_buffer.data[tx_buffer.ind_write]=c;
		tx_buffer.ind_write = ( tx_buffer.ind_write+1 ) & TX_BUFFER_MASK;
	}
	else UDR = c;	
	sei();	
}

#ifdef  PROTCOLL_CHECKSUM_4
// --- Protokoll 4-Bytes 
void TX_SendCommand(uint8_t cmd, uint8_t id, uint8_t val)
{
	TX_SendChar(cmd);
	TX_SendChar(id);
	TX_SendChar(val);
	TX_SendChar(cmd ^ id ^ val);
			
	while ((tx_buffer.ind_write != tx_buffer.ind_read) || ((UCSRA & (1<<UDRE) )==0)) {}	// warte bis alls leer
}
#endif 
#ifdef  PROTCOLL_INVERT_8
// --- Protokoll 4-Bytes
void TX_SendCommand(uint8_t cmd, uint8_t id, uint8_t val)
{
	TX_SendChar(0xff);
	TX_SendChar(0xff);
	TX_SendChar(cmd);
	TX_SendChar(~cmd);
	TX_SendChar(id);
	TX_SendChar(~id);
	TX_SendChar(val);
	TX_SendChar(~val);
	
	while ((tx_buffer.ind_write != tx_buffer.ind_read) || ((UCSRA & (1<<UDRE) )==0)) {}	// warte bis alls leer
}
#endif


#ifdef DEBUG_STUDIO
void my_delay(uint16_t __ms)
{
}
#else
void my_delay(uint16_t __ms)
{

	uint16_t __count;
	
	uint16_t _ticks  = (uint16_t) (__ms);
	while(_ticks)
	{
		
		__count = 3920;
		__asm__ volatile (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"
		: "=w" (__count)
		: "0" (__count)
		);
		_ticks --;
	}
}
#endif 



//====================================================================================================
