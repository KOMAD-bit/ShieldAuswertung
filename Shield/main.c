/*
 * Shield_8.c
 *
 * Created: 17/03/2022 14:19:34
 * Author : Katharina Boehm-Klamt 17.05.1985 
 
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


// define Ports

#define Rumble	7 
#define Shock	6
#define Pieps	4
#define Switch1	(1<<PD2)

// Shortcuts
#define shd_rumble_on()		PORTD |= (1<<Rumble)
#define shd_rumble_off()	PORTD &= ~(1<<Rumble)

#define shd_pieps_off()		PORTD &=~(1<<Pieps)
#define shd_pieps_on()		PORTD |=(1<<Pieps)

#define shd_shock_on()		PORTD |= (1<<Shock)
#define shd_shock_off()		PORTD &= ~(1<<Shock)

void empty(void);
void do100ms(void);
void do1000ms(void);
void do5000ms(void);
void handle_on_hit(uint16_t hit_by);
//----------------------------------------------------------------------------------------------------
uint8_t heartcalculation(uint8_t startenergy, uint8_t energy);

// State definiition
uint8_t     current_state;	// aktueller Zustand
#define     STATE_ADMIN	0	// Admin
#define		STATE_DELAY 1	// Wartezeit nach Treffer, reload etc
#define		STATE_GAME	2	// normaler Spielzustand
#define		STATE_OUT	3	// Energy=0
#define		STATE_CHECK 4	// testet LED´s Rumble IR Sender
#define		STATE_READ  5

#define     MaxGamesets 61			// Max. Anzahl möglcher Spielier
uint8_t     a_treffer[MaxGamesets];	// Trefferspeicher - von wem getroffem

// #define		RUMBLETIME    5;	// in 1/10 sekunden
uint8_t     rumble_time = 0;   
uint8_t		blink_time	= 0;	
uint8_t		blink_count = 0;	
uint8_t		INV;				

uint8_t 	energy;				
uint8_t		startenergy;
uint8_t		heart;
uint8_t		teiler;

uint16_t	delay_time = 3;		
uint8_t		batalertisr= 0;		
uint8_t		helpbat	   = 0;
uint16_t	baterievoltage=31;		
uint8_t		helpswlcd  = 0; 
uint8_t		displayonoff=1;
uint8_t     askingifread=3 ;
uint8_t		read;
uint8_t		hitcategorie ;


struct S_SETTINGS
{
	uint8_t	 max_energy;		
	uint8_t  PlayerNr;			
	uint8_t  Delay_Hit;			
	uint8_t  Delay_Reload;		
	uint8_t  Rumbletime;		
	uint8_t  Blinktime;			
	} E_gamesettings EEMEM = { 0, 8, 0, 1, 4, 0 };

	struct S_SETTINGS gamesettings;	
uint8_t matrix[MaxGamesets] EEMEM = {0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0  ,0 };

uint8_t  EEMEM isread  = 0;

void Init_Register(void)
{
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
		soft_s_zeiten[0].ms_max = 6; 
		soft_s_zeiten[0].fsync = empty; 

		
		soft_s_zeiten[1].ms_max = 1024; 
		soft_s_zeiten[1].fasync = do1000ms; 
		
		soft_s_zeiten[2].ms_max = 99; 
		soft_s_zeiten[2].fasync = do100ms;
		
		soft_s_zeiten[3].ms_max = 5120; 
		soft_s_zeiten[3].fasync = do5000ms;

		soft_enable_fsync	= 0b00000001;
		soft_enable_fasync	= 0b00001110;
		soft_enable			= 0b00001111;

	
	sei();
}

/**************************** I2C LCD **********************************/
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


 //****************************************************************************************************
 //					BAT ALert
 //****************************************************************************************************
#define PRESSED_BATALERT (1<<PD3)

#define LC709203F_I2CADDR_DEFAULT 0x0B     // i2c address
#define LC709203F_CMD_THERMISTORB 0x06     // Read/write thermistor B
#define LC709203F_CMD_INITRSOC 0x07        // Initialize RSOC calculation
#define LC709203F_CMD_CELLTEMPERATURE 0x08 // Read/write batt temp
#define LC709203F_CMD_CELLVOLTAGE 0x09     // Read batt voltage
#define LC709203F_CMD_APA 0x0B             // APA
#define LC709203F_CMD_RSOC 0x0D            // Read state of charge
#define LC709203F_CMD_CELLITE 0x0F         // Read batt indicator to empty
#define LC709203F_CMD_ICVERSION 0x11       // Read IC version
#define LC709203F_CMD_BATTPROF 0x12        // Set the battery profile
#define LC709203F_CMD_ALARMRSOC 0x13       // Alarm on percent 
#define LC709203F_CMD_ALARMVOLT 0x14       // Alarm on voltage 
#define LC709203F_CMD_POWERMODE 0x15       // Sets sleep/power 
#define LC709203F_CMD_STATUSBIT 0x16       // Temp obtaining method
#define LC709203F_CMD_PARAMETER 0x1A       // Batt profile code


/****************checksum for i2c batalert chip***********************/
static uint8_t crc8(uint8_t*databuffer, uint8_t len)
{
	uint8_t crc = 0x00;
	uint8_t remainder = 0;
	uint8_t bit = 0;
	uint8_t byte= 0;
	
	for(byte =0; byte<len; ++byte)
	{
		remainder ^= (databuffer[byte]<< ((8* sizeof(crc))-8));
		
		for(bit = 8; bit>0; --bit)
		{
			if(remainder & (1<< ((8*sizeof(crc))-1)))
			{
				remainder = (remainder <<1) ^0x07;
			}
			else
			{
				remainder = (remainder<<1);
			}
		}
	}
	return (remainder);
}

void i2cBatteriewatch(uint8_t Komando, uint8_t high, uint8_t low){
	uint8_t fr[4]={ LC709203F_I2CADDR_DEFAULT*2, Komando, low, high};

	unsigned char ret =i2c_start(0x16);
	if (ret)
	{i2c_stop();}
	else	{
		
		i2c_write(Komando);
		i2c_write(low);
		i2c_write(high);
		i2c_write(crc8(fr,4));
		i2c_stop();
		
		
	}
}

static readbatterie(uint8_t Komando, uint16_t data1){
	uint8_t re [6] ={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	re[0] = 0x16;
	re [1]= Komando;
	re[2] = re[0] | 0x1; ;
	i2c_start(re[0]);
	
	i2c_write(re[1]);
	i2c_start(re[2]);
	
	re[3]=i2c_readAck();
	re[4]=i2c_readAck();
	re[5]=i2c_readNak();
	
	uint8_t crc = crc8(re, 5);
	
	data1 = re[4];
	data1 = data1<<=8;
	data1|=re[3];
	i2c_stop();
	return data1;
	
}

void setup_bat()
{
	i2cBatteriewatch(LC709203F_CMD_POWERMODE,0x00, 0x01);					
	i2cBatteriewatch(LC709203F_CMD_APA, 0x00, 0x42);							// set APA 0x44
	i2cBatteriewatch(LC709203F_CMD_BATTPROF,0x00,0x00);						//bat profil
	//i2cBatteriewatch(LC709203F_CMD_INITRSOC, 0xAA, 0x55);
	i2cBatteriewatch(LC709203F_CMD_STATUSBIT,0x00,0x00);					//statusbit set 0x00 i2c mode 0x01 constant of thermistor
	i2cBatteriewatch(LC709203F_CMD_CELLTEMPERATURE,0x0B, 0xA6);
	//i2cBatteriewatch(LC709203F_CMD_ALARMVOLT,0x0F, 0xA0);					// 4 volt batteri alert
	i2cBatteriewatch(LC709203F_CMD_ALARMVOLT,0x0C,0x80);					// 3.2 Volt
	i2cBatteriewatch(LC709203F_CMD_POWERMODE, 0x00, 0x01);
}



void Init_Gameset()
{
	
	cli();
	eeprom_read_block(&gamesettings,&E_gamesettings,sizeof(gamesettings));
	sei();
	
	cli();
	askingifread = eeprom_read_word(&isread);
	sei();
	// clear treffer array
	if(askingifread==1)
	{
		
		uint8_t temp;
		for(temp = 0; temp < MaxGamesets; temp ++)
		{
			a_treffer[temp] = 0;
			
 			eeprom_update_byte(&matrix[temp], 0);
 			
 	
		}
			cli();
			eeprom_write_block(a_treffer, matrix, sizeof(a_treffer));
			eeprom_write_byte(&isread, 0);
			sei();
	}
	if(askingifread==0)
	{
		cli();
		eeprom_read_block(a_treffer, matrix , sizeof(a_treffer));
		sei();
		read = a_treffer[1];
		digitdraw(read);
		digitdraw(300);
		
		uint8_t temp; 
	
	}
	energy = gamesettings.max_energy;
	startenergy = energy;
	teiler = startenergy/3;
	
	heart = heartcalculation(startenergy, energy);
	
	shd_rumble_on();
	shd_pieps_on();
	shd_shock_on();
	my_delay(1000);
	shd_rumble_off();
	shd_pieps_off();
	shd_shock_off();
}

void do_eeprog_mode()
{
	
	cli();
	eeprom_write_block(&gamesettings,&E_gamesettings,sizeof(gamesettings));
	sei();
}

/*****************************************MAIN PROGRAM **********************************************/	
																									
																									
/****************************************************************************************************/
int main(void)
{
	Init_Register();
	i2c_init();
	
	

	restart: 
		RX_ClearBuffer();
		Init_Gameset();
		setup_bat();
		cleardisplay();
		playerdraw();
		my_delay(500);
		digitdraw(gamesettings.PlayerNr);
	//	my_delay(1000);
	//	bat();
	//	my_delay(1000);
		//digitdraw(readbatterie(0x09,0x00)/100);
		my_delay(1000);
		digitdraw(energy);
	
		
		current_state=STATE_GAME;
    /* Replace with your application code */
    while (1) 
    {
		soft_chk_do();
					

		if(!RX_GetNextCommand(&serial_in))
		{ 
			switch(serial_in.cmd)
			{
				case IRCMD_FIRE:
				{
					if ( current_state == (1<<STATE_GAME) ) // nur im game-state
					{
						//if (energy) // Ignore hits when at zero energy
						if (serial_in.id != gamesettings.PlayerNr && serial_in.val==IRCMD_NOP ) // wenn nicht Selbsttreffer
						{
							handle_on_hit(serial_in.id);
						}
					}
				}
				break;
				
				case IRCMD_REFILL_ENERGY:
				{
					if ( current_state & (1<<STATE_GAME | 1<<STATE_OUT ) )
					{
						if (serial_in.val==0)		//Refill=0 : nergy aus settings holen
						{
							energy = gamesettings.max_energy;
							startenergy =energy;
							teiler = startenergy/3;
							heart=heartcalculation(startenergy,energy);
							
						}
						else  // refill >0 : energy = value ; in settings auch ändern
						{
							energy = serial_in.val;
							startenergy = energy;
							teiler = startenergy/3;
							heart=heartcalculation(startenergy,energy);
						}
						delay_time = gamesettings.Delay_Reload*10;
						digitdraw(delay_time);
						current_state = (1<<STATE_DELAY); // in spielmodus zurück setzen
						digitdraw(energy);
					}
					else
					if ( current_state == (1<<STATE_ADMIN) )
					{
						gamesettings.max_energy = serial_in.val;
						digitdraw(energy); my_delay(500);
						digitdraw(gamesettings.max_energy);	my_delay(500);
						if(helpbat ==3){batalert();	}
					}				
				}
				break;
				
				case IRCMD_REFILL_BOTH_EN:
				{
					if ( current_state & (1<<STATE_GAME | 1<<STATE_OUT ) )
					{
						if (serial_in.val==0)		//Refill=0 : nergy aus settings holen
						{
							energy = gamesettings.max_energy;
							startenergy =energy;
							teiler = startenergy/3;
							heart=heartcalculation(startenergy,energy);
							
						}
						else  // refill >0 : energy = value ; in settings auch ändern
						{
							energy = serial_in.val;
							startenergy =energy;
							teiler = startenergy/3;
							heart=heartcalculation(startenergy,energy);
						}
						delay_time = gamesettings.Delay_Reload*10;
						digitdraw(delay_time/10);
						current_state = (1<<STATE_DELAY); // in spielmodus zurück setzen
						digitdraw(energy);
						}
					else
						if ( current_state == (1<<STATE_ADMIN) )
						{
							gamesettings.max_energy = serial_in.val;
							digitdraw(energy); my_delay(500);
							digitdraw(gamesettings.max_energy);	my_delay(500);
							if(helpbat ==3){batalert();	}

						}				
				}
				break;
				
				case IRCMD_HITMEM:
				{			
					if((serial_in.id == gamesettings.PlayerNr)&&(serial_in.val == 1))
							{
								
								eeprom_write_word(&isread, 1);
								askingifread = eeprom_read_word(&isread);
								my_delay(50);
								current_state=STATE_READ;
								digitdraw(current_state);
								my_delay(50);
									while (current_state == 5)
									{
										for(uint8_t hit_by = 1; hit_by<55; hit_by++)
										{	uint8_t a = a_treffer[hit_by];
											TX_SendCommand(gamesettings.PlayerNr, hit_by, a);
										}
									}
							}				
				}
				break;
				
				case IRCMD_CLEARHITMATRIX:
				{
					if(serial_in.id == IRCMD_CLEARHITMATRIX)
										{
											uint8_t temp;
											for(temp = 0; temp < MaxGamesets; temp ++)
											{
												a_treffer[temp] = 0;
												
												eeprom_update_byte(&matrix[temp], 0);
												
												
											}
											cli();
											eeprom_write_block(a_treffer, matrix, sizeof(a_treffer));
											eeprom_write_byte(&isread, 0);
											sei();
											admin();
											my_delay(1000);
											goto restart;
										}
					break;
				}
				
				case IRCMD_SET_ADMIN:
				{
						if (serial_in.id==IRCMD_NOP)
						{
							// adminmode on
							if ( (serial_in.val & IRVAL_ADMIN) /* && current_state!=(1<<STATE_ADMIN)*/ )
							{
								current_state = (1<<STATE_ADMIN);
								admin();
								my_delay(1000);
						
							}
							//adminmode off
							else if ( !(serial_in.val & IRVAL_ADMIN) && (current_state & (1<<STATE_ADMIN)) )
							{
							
								current_state = (1<<STATE_GAME);
								digitdraw(energy);
								my_delay(1000);
							}
						}
				}
				break;				
				
				
				// nur im ADMIN Mode
				#pragma region SIGNAL AMDIN
				
				case IRCMD_SET_PLAYER:			// player Nr setzen
				{
					if (current_state == (1<<STATE_ADMIN) )
					if (serial_in.id == serial_in.val)
					{
						gamesettings.PlayerNr = serial_in.id;
						playerdraw();		  my_delay(500);
						digitdraw(gamesettings.PlayerNr); my_delay(1000);
					
					}
				}
				break;
				case IRCMD_EEPROM_BURN:			// Setze Player - nur wenn id==val!
				{
					if (current_state == (1<<STATE_ADMIN) )
					if (serial_in.id == serial_in.val && serial_in.val == IRCMD_NOP)
					{
						do_eeprog_mode();
						goto restart;
					}
				}
				break;
				case IRCMD_SETDELAYS: // id=delay after hit (1/10), val=delay after reload (s)
				{
					if (current_state == (1<<STATE_ADMIN) )
					{
						gamesettings.Delay_Hit = serial_in.id;
						gamesettings.Delay_Reload = serial_in.val;
						delay();				my_delay(500);
						digitdraw(gamesettings.Delay_Hit);		my_delay(1000);
					
						digitdraw(gamesettings.Delay_Reload);	my_delay(1000);
					}
				}
				break;
				case IRCMD_RUN_CHECK: // id=delay after hit (1/10), val=delay after reload (s)
				{
					if (current_state & (1<<STATE_ADMIN) )
					{
						if (current_state & (1<<STATE_CHECK))
						{
							current_state = (1<<STATE_ADMIN);
						
						}
						else current_state |= (1<<STATE_CHECK);
					}
					
				}
				break;
				#pragma enregion SIGNAL ADMIN
								
			}
		}

    }
}





//****************************************************************************************************
//					SOFT - Timer interrupt routinen
//****************************************************************************************************


void empty(void){ }
void do100ms(void)
{
		
		// delay time handlen
		if (current_state==(1<<STATE_DELAY))
		{
			if (delay_time) delay_time--;
			
			if (delay_time)
			{
			//	display_value((delay_time+9)/10);											// Anzeige in s (0.x) als 1
			//	TX_SendCommand(IRCMD_STOG_INFO,gamesettings.PlayerNr, 0x80| delay_time );	// Sende time an gun + delay time | delay_time
			}
			else // spielzustand reaktivieren (auch an Gun senden)
			{
				TX_SendCommand(IRCMD_STOG_INFO,gamesettings.PlayerNr,0x00);	// Reactivate gun
				//digitdraw(energy);
				if(helpbat ==3){batalert();	}		 
				current_state=(1<<STATE_GAME);
				heart=heartcalculation(startenergy,energy);
				TX_SendCommand(IRCMD_SHIELD_ENERGY,gamesettings.PlayerNr,energy); // energy sende, sodass aktuelles Display
				TX_SendCommand(IRCMD_INVULNERABLE, gamesettings.PlayerNr,heart);
				shd_rumble_off();
				shd_pieps_off();
				shd_shock_off();

			}
		}
		else if (current_state & (1<<STATE_CHECK))
		{

		}

}
void do1000ms(void)
{
	
	if(current_state == 8){TX_SendCommand(IRCMD_STOG_INFO, gamesettings.PlayerNr, 0x80);}
		

}
void do5000ms(void)
{
		// alle 5 Sekunden shield Energy an GUN senden

			//heart=ceil((energy)/teiler);
			if(current_state==4){
			TX_SendCommand(IRCMD_SHIELD_ENERGY,gamesettings.PlayerNr,energy);
			TX_SendCommand(IRCMD_INVULNERABLE, gamesettings.PlayerNr,heart);

	}
	
	/*	if(helpbat==0)
		{
			batalertisr= readbatterie(0x09,0x00)/100; my_delay(5);
			if(batalertisr<baterievoltage)
			{
				batalert();
				batalertisr=baterievoltage+1;
				helpbat=3;
			}
		}*/
}
// Treffer an Gun übertragen 
void handle_on_hit(uint16_t hit_by) // hit_by = treffer durch Spieler Nr
{  
  if(INV==0)//27032020 eingebaut Invulerable
  {   
    if ( hit_by<MaxGamesets) // nur bei gültigen Hits
    {
				// count hits by
			if (a_treffer[hit_by] != 0xFF)
			{
				if(energy>0)
				{
							if(hit_by==12 || hit_by==11)		{if(energy>1){a_treffer[hit_by] =	a_treffer[hit_by]+2;}else{a_treffer[hit_by]= a_treffer[hit_by]+1;}}
							if(hit_by == 52 || hit_by==51 || hit_by==53 || hit_by==54 )	
							{
								if(energy>2){a_treffer[hit_by] =a_treffer[hit_by]+3;}
								else 
								{ if(energy==2){a_treffer[hit_by] = a_treffer[hit_by]+2;}
									else{a_treffer[hit_by] = a_treffer[hit_by]+1;}
								}
							}
							if((hit_by<=10) || (hit_by>=55))	{a_treffer[hit_by] =	a_treffer[hit_by]+1;}
							if((hit_by>=13) && (hit_by<=50))	{a_treffer[hit_by] =	a_treffer[hit_by]+1;}	
				}
								
			}
				
			if(hit_by==12 || hit_by==11)		{if(energy>1){energy=energy-2;}else{energy=0;}}
			if(hit_by == 52 || hit_by==51 || hit_by==53 || hit_by==54 )		{if(energy>2){energy=energy-3;}else{energy=0;}}
			if((hit_by<=10) || (hit_by>=55))	{if(energy>1){energy=energy-1;}else{energy=0;}}
			if((hit_by>=13) && (hit_by<=50))	{if(energy>1){energy=energy-1;}else{energy=0;}}
						
	
		
		
		uint8_t b = a_treffer[hit_by];
		eeprom_write_word(&matrix[hit_by], b);
		digitdraw(energy);
		if(helpbat ==3){batalert();	}
		
			
		heart = heartcalculation(startenergy,energy);
		TX_SendCommand(IRCMD_SHIELD_ENERGY,gamesettings.PlayerNr,energy);	// zur Gun senden
		TX_SendCommand(IRCMD_INVULNERABLE, gamesettings.PlayerNr,heart);		//HErzlübertragung 
		
		if (energy)	// Delay bis wieder Treffbar, nur wenn noch Energy vorhanden 
		{
				if(hit_by==12 || hit_by==11|| hit_by == 52 || hit_by==51 || hit_by==53 || hit_by==54 ){
					shd_rumble_on();
					shd_shock_on();
					shd_pieps_on();
					rumble_time =2*gamesettings.Rumbletime;
					delay_time =2;
					current_state = (1<<STATE_DELAY);
	
				}
				else
				{
					shd_rumble_on();
					shd_shock_on();
					shd_pieps_on();
					rumble_time = gamesettings.Rumbletime;
					//
					
					delay_time = gamesettings.Delay_Hit;
					current_state = (1<<STATE_DELAY);
				}
			
		}
		else // keine Energy: in Timeout gehen
		{
			TX_SendCommand(IRCMD_STOG_INFO,gamesettings.PlayerNr,0x80);	// timeout wenn energy=0
			current_state = (1<<STATE_OUT);
			strich();
			shd_pieps_on();
			shd_rumble_on();
			shd_shock_on();
			my_delay(3000);
			shd_shock_off();
			shd_pieps_off();
			shd_rumble_off();
		}
		
		RX_ClearBuffer(); // Empangspufferlöschen damit nicht inzwischenzeit Schüsse eingelangen sind
  	}
	  
}
	if(INV==1)
	{
		
	}
}

 uint8_t heartcalculation(uint8_t startenergy, uint8_t energy){
	 
	 if(energy == 0){heart =0;}
		 else{
			 if(energy==startenergy)
			 {teiler = startenergy/3;}
			 uint8_t x = ((energy)*10/teiler);
			 if(x<1)
			 return 0;
			 if((0<x) && (x<11))
			 return 1;
			 if((10<x) && (x<21))
			 return 2;
			 if(20<x)
			 return 3;
			 }
	 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 }
