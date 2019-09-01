#ifndef _I2C_H
#define _I2C_H
#include<LPC17xx.h>
#include"NVIC.h"
#include"GPIO.h"
#include"PLL.h"

/*
#define I2C0		10
#define I2C1		11
#define I2C2		12
*/
#define MASTER 0
#define SLAVE 1
#define TX 2
#define RX 3

void Config_I2C(unsigned short I2Cx,unsigned short Mode,unsigned short tran_rec);

#endif