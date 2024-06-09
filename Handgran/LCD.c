/*
 * CFile1.c
 *
 * Created: 17/03/2022 17:39:42
 *  Author: Katharina Böhm-Klamt
*/

#include <inttypes.h>

#include "LCD.h"
#include "twimaster.h"
#define DevSSD1306 0x78

static uint16_t bufferpage [64]={
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static	int Pages [8]={ 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7};
	 
void drawBufferPage(uint16_t *buffp, int page, int position){
unsigned char ret = i2c_start(DevSSD1306+I2C_WRITE);       // set device address and write mode+I2C_WRITE
	if ( ret ) { i2c_stop();}
	else
	{
		
		setup_i2c();
		i2c_write(page);
		i2c_write(position);          // hi col = 0
		i2c_write(0x40 | 0x0);            // line #0
		uint8_t twbrbackup = TWBR;
		TWBR = 12;

		for (uint16_t i=0; i<64; i++){

			i2c_start_wait(DevSSD1306+I2C_WRITE);   // set device address and write mode
			
			i2c_write(0x40);                        // set display RAM display start line register
			i2c_write(buffp[i]);
			i2c_stop();
			TWBR = twbrbackup;
			
		}
		i2c_stop();
	}
		
}

void clearbuffer(int x, int y) 
{
	for(int i=0; i<y; i++) {bufferpage[i] =0x00;}
}
void cleardisplay(){
	clearbuffer(0,64);
	for(int i =0; i<=7; i++){
		drawBufferPage(bufferpage,Pages[i], 0x10);
		drawBufferPage(bufferpage, Pages[i], 0x14);
	}
}

void off(){
	unsigned char ret = i2c_start(DevSSD1306+I2C_WRITE);       // set device address and write mode+I2C_WRITE
	if ( ret ) { i2c_stop();}
	else{
		i2c_write(0xAF);
		TWBR = 12;
	}
		i2c_stop();

}

void horizontalline(int x, int y){
		for(int i = x; i<y; i++){
		bufferpage[i]=0xF0;}
}
void verticalline(int x, int y){
	for(int i = x; i<y; i++){
	bufferpage[i]=0xFF;}
}

/*******************************GIVE THE DIGITS CASES (0-9)*******************/
void position1(int dez, int leftstart, int leftend, int rightstart, int rightend, int fulll, int fullr, int position)
{
		switch (dez)
		{
			case 0:{
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);
				
				clearbuffer(fulll,fullr);
				verticalline(leftstart,leftend);
				verticalline(rightstart,rightend);
				for(int i=0xB2; i<=0xB6;i++){
				drawBufferPage(bufferpage, i, position);}
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);

				break;
			}
			
			case 1:{
				clearbuffer(fulll,fullr);
				for(int i=0xB1; i<=0xB7;i++){
				drawBufferPage(bufferpage, i, position);}
				
				verticalline(rightstart,rightend);
				for(int i=0xB1; i<=0xB7;i++){
					drawBufferPage(bufferpage, i, position);
				}
			break;}
			case 2:{
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);

				clearbuffer(fulll,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB2; i<=0xB4;i++){
				drawBufferPage(bufferpage, i, position);}
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				
				clearbuffer(fulll,fullr);
				verticalline(leftstart,leftend);
				for(int i=0xB5; i<=0xB6;i++){
				drawBufferPage(bufferpage, i, position);}
				
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);
			break;}
			
			case 3:{clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);

				clearbuffer(fulll,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB2; i<=0xB6;i++){
				drawBufferPage(bufferpage, i, position);}
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);
			break;}
			
			case 4:{
				
				clearbuffer(fulll,fullr);
				drawBufferPage(bufferpage, 0xB7, position);
				verticalline(leftstart,leftend);
				for(int i=0xB1; i<=0xB4;i++){
				drawBufferPage(bufferpage, i, position);}
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				
				clearbuffer(fulll,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB5; i<=0xB7;i++){
				drawBufferPage(bufferpage, i, position);}
			break;}
			
			case 5:{
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);

				clearbuffer(fulll,fullr);
				verticalline(leftstart,leftend);
				for(int i=0xB2; i<=0xB4;i++){
				drawBufferPage(bufferpage, i, position);}
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				
				clearbuffer(fulll,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB5; i<=0xB6;i++){
				drawBufferPage(bufferpage, i, position);}
				
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);
			break;}
			
			case 6:{
				
				clearbuffer(fulll,fullr);
				//for(int i=0xB1; i<=0xB7;i++){
				//drawBufferPage(bufferpage, i);}
				
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);

				clearbuffer(fulll,fullr);
				verticalline(leftstart,leftend);
				for(int i=0xB2; i<=0xB6;i++){
				drawBufferPage(bufferpage, i, position);}
				
				//clearbuffer(0,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB4; i<=0xB6;i++){
				drawBufferPage(bufferpage, i, position);}
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);
			break;}
			
			case 7:{
				
				clearbuffer(fulll,fullr);
				drawBufferPage(bufferpage, 0xB7, position);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);
				clearbuffer(fulll,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB2; i<=0xB7;i++){
					drawBufferPage(bufferpage, i, position);
				}
				

			break;	}
			
			case 8:{
				clearbuffer(fulll,fullr);
				
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);
				clearbuffer(fulll,fullr);
				verticalline(leftstart,leftend);
				verticalline(rightstart,rightend);
				for(int i=0xB2; i<=0xB6;i++){
					drawBufferPage(bufferpage, i, position);
				}
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);
				break;
			}
			
			case 9:{
				clearbuffer(fulll,fullr);
				for(int i=0xB1; i<=0xB7;i++){
				drawBufferPage(bufferpage, i, position);}
				
				clearbuffer(fulll,fullr);
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB1, position);
				
				clearbuffer(fulll,fullr);
				verticalline(rightstart,rightend);
				for(int i=0xB2; i<=0xB6;i++){
					drawBufferPage(bufferpage, i, position);
				}
				verticalline(leftstart,leftend);
				for(int i=0xB2; i<=0xB3;i++){
					drawBufferPage(bufferpage, i, position);
				}
				horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB4, position);
				
				clearbuffer(fulll,fullr);
				//horizontalline(leftend,rightstart);
				drawBufferPage(bufferpage, 0xB7, position);
				break;
			}
		}
	
}

/*************draw numbers (0-99)*************************/
void digitdraw(int energy){
		int b=energy/10;
		int c= energy-(b*10);
		
		if (b<10 && b>0)
		{
			position1(b,6,10,53,57,0,64, 0x10);
			position1(c,6,10,53,57,0,64, 0x14);
		}
		if (b>99)
		{
			position1(9,6,10,53,57,0,64, 0x10);
			position1(9,6,10,53,57,0,64, 0x14);
		}
		if(b==0)
		{
			cleardisplay();
			position1(c,6,10,53,57,0,64, 0x14);
		}
}
void playerdraw(){
	
	// P
	clearbuffer(0,64);
	
	horizontalline(10,53);
	drawBufferPage(bufferpage, 0xB1, 0x10);
	clearbuffer(0,64);
	verticalline(6,10);
	for(int i=0xB2; i<=0xB7;i++){
		drawBufferPage(bufferpage, i, 0x10);
	}
	verticalline(53,57);
	for(int i=0xB2; i<=0xB3;i++){
		drawBufferPage(bufferpage, i, 0x10);
	}
	horizontalline(10,53);
	drawBufferPage(bufferpage, 0xB4, 0x10);
	
	//l
	clearbuffer(0,64);

	verticalline(6,10);
	for(int i=0xB1; i<=0xB7;i++){
		drawBufferPage(bufferpage, i, 0x14);
	}
	
	horizontalline(10,53);
	drawBufferPage(bufferpage, 0xB7, 0x14);
}

void batalert()
{
	clearbuffer(0,64);
	for(int i=8; i<=19; i++){
		bufferpage[i] = 0x42;
	}
	bufferpage[7]=0x7E;
	bufferpage[6]=0x18;
	bufferpage[20]=0x7E;
	bufferpage[21]=0x7E;
	
	drawBufferPage(bufferpage, 0xB0, 0x10);
}

void bat()
{
	//B
	clearbuffer(0,64);
	
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB1, 0x10);
	clearbuffer(0,64);
	verticalline(6,12);
	verticalline(53,57);
	for(int i=0xB2; i<=0xB6;i++){
		drawBufferPage(bufferpage, i, 0x10);
	}
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB4, 0x10);
	clearbuffer(0,64);
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB7, 0x10);
	
	
	
	//T
	clearbuffer(0,64);
	for(int i=0xB1; i<=0xB7;i++){
	drawBufferPage(bufferpage, i, 0x14);}
	
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB1, 0x14);
	
	clearbuffer(0,64);
	verticalline(30,36);
	for(int i=0xB2; i<=0xB7;i++){
	drawBufferPage(bufferpage, i, 0x14);}
	
	
}

void admin()
{
	//A
	clearbuffer(0,64);
	
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB1, 0x10);
	clearbuffer(0,64);
	verticalline(6,12);
	verticalline(51,57);
	for(int i=0xB2; i<=0xB6;i++){
		drawBufferPage(bufferpage, i, 0x10);
	}
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB4, 0x10);
	
	clearbuffer(0,64);
	for(int i=0xB1; i<=0xB7;i++){
	drawBufferPage(bufferpage, i, 0x14);}
}

void delay(){
	
	
	clearbuffer(0,64);
	for(int i=0xB1; i<=0xB7;i++){
	drawBufferPage(bufferpage, i, 0x10);}
	//d
	clearbuffer(0,64);
	
	
	verticalline(51,57);
	for(int i=0xB2; i<=0xB6;i++){
		drawBufferPage(bufferpage, i, 0x14);
	}
	verticalline(6,12);
	for(int i=0xB4; i<=0xB7;i++){
		drawBufferPage(bufferpage, i, 0x14);
	}
	clearbuffer(0,64);
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB4, 0x14);
	horizontalline(12,53);
	drawBufferPage(bufferpage, 0xB7, 0x14);
	
}

void readmatrix()
{
	clearbuffer(0,64);
	for(int i =5; i<=7; i++){
		drawBufferPage(bufferpage,Pages[i], 0x10);
		drawBufferPage(bufferpage, Pages[i], 0x14);
	}	
	drawBufferPage(bufferpage,Pages[0], 0x10);
	drawBufferPage(bufferpage,Pages[0], 0x14);
	drawBufferPage(bufferpage,Pages[1], 0x10);
	drawBufferPage(bufferpage,Pages[1], 0x14);

	uint16_t bufferpagea [64]={
		0x00, 0xFF, 0xFF, 0xFF, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x00, 0x00, 0x00, 
		0xF0, 0xFC, 0xC6, 0xC3, 0xC3, 0xC3, 0xC6, 0xFC, 0xF0, 0x00, 0x00, 0x00, 
		0xF0, 0xFC, 0x06, 0x03, 0x03, 0x03, 0x06, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 
		0xF0, 0xFC, 0x06, 0x03, 0x03, 0x03, 0x06, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00};
	
	drawBufferPage(bufferpagea, 0xB3, 0x10);
		uint16_t bufferpageb [64]={
			0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x0F, 0x3F, 0x60, 0xC0, 0xC0, 0xC0, 0x60, 0x30, 0x18, 0x00, 0x00, 0x00,
			0x0F, 0x3F, 0x60, 0xC0, 0xC0, 0xC0, 0x60, 0x30, 0xFF, 0xFF, 0x00, 0x00,
			0x0F, 0x3F, 0x60, 0xC0, 0xC0, 0xC0, 0x60, 0x30, 0xFF, 0xFF, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00};
	
	drawBufferPage(bufferpageb, 0xB4, 0x10);	
	
	uint16_t bufferpagec [64]={
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00};
	drawBufferPage(bufferpagec,0xB2, 0x10);
	
}

void strich(){
	clearbuffer(0,64);
	horizontalline(1,53);
	drawBufferPage(bufferpage, 0xB0, 0x10);
	drawBufferPage(bufferpage, 0xB0, 0x14);

}