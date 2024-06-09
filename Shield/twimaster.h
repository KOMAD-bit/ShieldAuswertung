/*
 * twimaster.h
 *
 * Created: 13/03/2022 22:24:47
 *  Author: Katharina Böhm-Klamt
 */ 


#ifndef TWIMASTER_H_
#define TWIMASTER_H_



#include <avr/io.h>


#ifdef DOXYGEN

#endif 

/**@{*/

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

#include <avr/io.h>

#define I2C_READ    1

#define I2C_WRITE   0


extern void i2c_init(void);

extern void i2c_stop(void);



extern signed char i2c_start(unsigned char addr);

extern unsigned char i2c_rep_start(unsigned char addr);


extern void i2c_start_wait(unsigned char addr);

 
 

extern unsigned char i2c_write(unsigned char data);


extern unsigned char i2c_readAck(void);


extern unsigned char i2c_readNak(void);


extern unsigned char i2c_read(unsigned char ack);
#define i2c_read(ack)  (ack) ? i2c_readAck() : i2c_readNak(); 


/**@}*/





#endif /* TWIMASTER_H_ */