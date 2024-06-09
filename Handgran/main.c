/*
 * Handgranatae.c
 *
 * Created: 04.01.2024 10:16:56
 * Author : Katharina Böhm-Klamt
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>

#include "RS232.h"
#include "twimaster.h"
#include "Commands.h"
#include "softwaretimerISRTIM2.h"
#include "LCD.h"


#define Pieps	4
#define Trigger	2
#define LED		7
#define Game	6
	


void shot(void);
void do1000ms(void);
void do5000ms(void);
void trigger(void);

void Init_Register(void)
{
	// Input/Output Ports initialization
cli();
	// PORT B 
	PORTB = 0x00; 
	DDRB  = 0xC;  // Pins : 0 0 0 0 1 1 O O
	//                      7 6 5 4 3 2 1 0
	// PB7 = XTAL ----------^ | | | | | | |
	// PB6 = XTAL ------------^ | | | | | |
	// PB5 = SCK  --------------^ | | | | |
	// PB4 = MISO ----------------^ | | | |
	// PB3 = MOSI ------------------^ | | |
	// PB2 = MOD----------------------^ | |
	// PB1 = ---------------------------^ |
	// PB0 = -----------------------------^

	// Port C initialization
	PORTC = 0x00;	//
	DDRC  = 0x70;	//      0 1 1 1 0 O O 0
	//                      7 6 5 4 3 2 1 0
	// PC7 = ---------------^ | | | | | | |
	// PC6 = Reset------------^ | | | | | |
	// PC5 = SDA LCD -----------^ | | | | |
	// PC4 = SCL LCD -------------^ | | | |
	// PC3 = -----------------------^ | | |
	// PC2 = -------------------------^ | |
	// PC1 = ---------------------------^ |
	// PC0 = -----------------------------^


	// Port D initialization
	PORTD = 0x00;
	DDRD  = 0xD2;  //       1 1 O 1 O O 1 0
	//                      7 6 5 4 3 2 1 0
	// PD7 = Shock----------^ | | | | | | |
	// PD6 = Rumble-----------^ | | | | | |
	// PD5 = -------------------^ | | | | |
	// PD4 = Speak----------------^ | | | |
	// PD3 = BAT_ALERT--------------^ | | |
	// PD2 = Taster SW1---------------^ | |
	// PD1 = TX USART Out---------------^ |
	// PD0 = RX USART In -----------------^
	

	
	TCCR1B = (1<<WGM12)  |(1<<CS10); //
	TCCR1A |= (1<<COM1B0);
	OCR1A=230;			
	
	
	
	TCCR2 = 0x03;
	TCNT2 =0x00;
	TIMSK = (1<<TOIE2);
		UCSRA=0x00;
		UCSRB=0xD8; 
		UCSRC=0x86;
		
		UBRRH=BAUD_H;
		UBRRL=BAUD_L;
		
		

		SFIOR = (1<<PUD);	
		SPCR = 0x00;
		
		
		)
		soft_s_zeiten[0].ms_max = 6; //1;
		soft_s_zeiten[0].fsync = trigger; // do5ms;

		// timer 1 = led-alternate display
		soft_s_zeiten[1].ms_max = 1024; //255;
		soft_s_zeiten[1].fasync = do1000ms; //display_alternate_toggle;
		
		soft_s_zeiten[2].ms_max = 99; // 25;
		soft_s_zeiten[2].fasync = shot;
		
		soft_s_zeiten[3].ms_max = 5120; //1275;
		soft_s_zeiten[3].fasync = do5000ms;

		soft_enable_fsync	= 0b00000001;
		soft_enable_fasync	= 0b00001110;
		soft_enable			= 0b00001111;

	
	sei();
}


#define shd_pieps_off()		PORTD &=~(1<<Pieps)
#define shd_pieps_on()		PORTD |=(1<<Pieps)
#define shd_led_off()		PORTD &=~(1<<LED)
#define shd_led_on()		PORTD |=(1<<LED)
#define shd_game_yes()		PORTD |=(1<<Game)
#define shd_game_no()		PORTD &=~(1<<Game)


uint16_t triggerpressed = 0;
uint16_t	munition = 0;
uint8_t		d1=0;
uint8_t		d2=0;
uint8_t		d3=0;
struct S_SETTINGS
{
	
	uint16_t max_ammo;		//maximal shots
	uint8_t  Player_NR;		//PlayerNumber
	uint8_t	 Delay;
	} E_gamesettings EEMEM = { 0, 8, 0 };
	struct S_SETTINGS gamesettings;	// Im Programm verwendete Variablen

		// State definiition
uint8_t     current_state;		// aktueller Zustand
#define     STATE_ADMIN	0x00	// admin modus
#define		STATE_GAME	0x01	// normaler Spielmodus
#define		STATE_OUT	0x02	// gun gesperrt (keien Mun oder keien Energy)

#define TRIGGERPRESSED    (1<<PD2)


#define DevSSD1306	0x78;
void    setup_i2c()
{
	i2c_write(0x80);
	i2c_write(0xAE);                    // Display Off

	i2c_write(0x00 | 0x0);            // low col = 0
	i2c_write(0x10 | 0x0);           // hi col = 0
	i2c_write(0x40 | 0x0);            // line #0

	i2c_write(0x80);
	i2c_write(0x81);                   // Set Contrast 0x81
	i2c_write(0xFF);					//je höher desto höher Kontrast
	
	// flips display
	i2c_write(0xA1);                    // Segremap - 0xA1
	
	i2c_write(0x80);
	i2c_write(0xC8);                    // COMSCAN DEC 0xC8 C0
	i2c_write(0xA6);                    // Normal Display 0xA6 (Invert A7)
	
	//	i2c_write(0xA4);                // DISPLAY ALL ON RESUME - 0xA4
	i2c_write(0xA8);                    // Set Multiplex 0xA8
	i2c_write(0x3F);                    // 1/64 Duty Cycle

	i2c_write(0xD3);                    // Set Display Offset 0xD3
	i2c_write(0x00);                     // no offset

	i2c_write(0xD5);                    // Set Display Clk Div 0xD5
	i2c_write(0x80);                    // Recommneded resistor ratio 0x80


	i2c_write(0xD9);                  // Set Precharge 0xd9
	i2c_write(0x11);

	i2c_write(0xDA);                    // Set COM Pins0xDA
	
	i2c_write(0x12);

	i2c_write(0xDB);                 // Set VCOM Detect - 0xDB
	i2c_write(0x40);

	
	
	i2c_write(0x20);                    // Set Memory Addressing Mode
	i2c_write(0x02);                    // 0x00 - Horizontal

	//	i2c_write(0x40 | 0x0);              // Set start line at line 0 - 0x40

	i2c_write(0x8D);                    // Charge Pump -0x8D
	i2c_write(0x14);

	//	i2c_write(0xA4);              //--turn on all pixels - A5. Regular mode A4
	i2c_write(0xAF);                //--turn on oled panel - AF
}

void Write_EEPROM()
{
	cli();
	eeprom_write_block(&gamesettings,&E_gamesettings,sizeof(gamesettings));
	sei();
}

//----------------------------------------------------------------------------------------------------
void Read_EEPROM()
{
	cli();
	eeprom_read_block(&gamesettings,&E_gamesettings,sizeof(gamesettings));
	sei();
	munition = gamesettings.max_ammo;
}


int main(void)
{
		Init_Register();
		i2c_init();
		my_delay(50);

	restart:
	Read_EEPROM();
	RX_ClearBuffer();
	i2c_init();
	my_delay(50);
	cleardisplay();
	digitdraw(1);
	my_delay(1000);
	current_state =STATE_GAME;
	d1=gamesettings.Delay*1000/3;
	d2=gamesettings.Delay*1000/6;
	d3=gamesettings.Delay*1000/12;
	shd_game_yes();
	my_delay(500);
	shd_game_no();
	digitdraw(2);
	/* Replace with your application code */
	while (1)
	{		soft_chk_do();

		TX_SendCommand(IRCMD_FIRE, 13, IRCMD_NOP);
		my_delay(1000);
		
		
		if (!RX_GetNextCommand(&serial_in))
		{
			
			switch (serial_in.cmd)
			{
				case IRCMD_REFILL_ENERGY:{
					munition = serial_in.id;
					munition = munition << 8;
					munition += serial_in.val;
					current_state = STATE_GAME;
					shd_game_yes();
					my_delay(500);
					shd_game_no();

				}
				
				
				
				break;
				
				case IRCMD_REFILL_BOTH_EN:{
					munition = serial_in.id;
					munition = munition << 8;
					munition += serial_in.val;
					current_state = STATE_GAME;
					shd_game_yes();
					my_delay(500);
					shd_game_no();
				}
				
				break;
				
				
				
				case IRCMD_SET_ADMIN:
				if (serial_in.id==IRCMD_NOP)
				{
					if ( (serial_in.val == IRVAL_ADMIN))
					{
						current_state =  STATE_ADMIN;
						cleardisplay();
						admin();
						my_delay(1000);
						
					}
					else
					if (!(serial_in.val & IRVAL_ADMIN) && (current_state==STATE_ADMIN))
					{
						
						current_state =(1<<STATE_GAME);
					}
				}
				break;
				
				
				// nur im ADMIN Mode
				#pragma region SIGNAL AMDIN
				
				case IRCMD_HIT_GUN:
				if(serial_in.id == serial_in.val)
				{
					gamesettings.Delay = serial_in.id;
					digitdraw(gamesettings.Delay);
					my_delay(500);
					
				}
				break;
				
				case IRCMD_SET_PLAYER:			// player Nr setzen
				//if (current_state == (1<<STATE_ADMIN) )
				if (serial_in.id == serial_in.val)
				{
					gamesettings.Player_NR = serial_in.id;
					digitdraw(gamesettings.Player_NR); my_delay(500);
					
				}
				break;
				
				case IRCMD_EEPROM_BURN:			// Setze Player - nur wenn id==val!
				//	if (current_state ==  (1<<STATE_ADMIN) )
				if (serial_in.id == serial_in.val && serial_in.val == IRCMD_NOP)
				{
					Write_EEPROM();
					my_delay(500);
					cleardisplay();
					admin();
					my_delay(500);
					goto restart;
				}
				break;
				
				
				#pragma endregion SIGNAL ADMIN

			}
		}

	}
}

void shot(void){
	if(triggerpressed == TRIGGERPRESSED){
		if(current_state==STATE_GAME)		{
			
			for(int i=0; i<3; i++){
				shd_pieps_on();
				shd_led_on();
				my_delay(d1/2);
				shd_pieps_off();
				shd_led_off();
				my_delay(d1/2);
			}
			TX_SendCommand(IRCMD_FIRE, gamesettings.Player_NR, IRCMD_NOP);
			for(int i=0; i<6; i++){
				shd_pieps_on();
				shd_led_on();
				my_delay(d2/2);
				shd_pieps_off();
				shd_led_off();
				my_delay(d2/2);
			}
			TX_SendCommand(IRCMD_FIRE, gamesettings.Player_NR, IRCMD_NOP);
			for(int i=0; i<12; i++){
				shd_pieps_on();
				shd_led_on();
				my_delay(d3/2);
				shd_pieps_off();
				shd_led_off();
				my_delay(d3/2);
			}
			TX_SendCommand(IRCMD_FIRE, gamesettings.Player_NR, IRCMD_NOP);
			shd_pieps_on();
			shd_led_on();
			current_state=STATE_OUT;
			
		}
	}
}

void do1000ms(void){}

void do5000ms(void){}

void trigger(void)
{
	triggerpressed = (PIND & TRIGGERPRESSED);
}




