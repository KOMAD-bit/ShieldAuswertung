/*
 * LCD.h
 *
 * Created: 17/03/2022 17:39:32
 *  Author: Katharina Böhm-Klamt
 */ 


#ifndef LCD_H_
#define LCD_H_


 
 void drawBufferPage(uint16_t *buffp, int page, int position);
 void clearbuffer(int x, int y);
 void cleardisplay();
 void horizontalline(int x, int y);
 void verticalline(int x, int y);
 void position1(int dez, int leftstart, int leftend, int rightstart, int rightend, int fulll, int fullr, int position);
 void digitdraw(int energy);
 void playerdraw();
 void batalert();
 void admin();
 void off();
 void bat();
 void admin();
 void delay();
 void readmatrix();
 void strich();



#endif /* LCD_H_ */