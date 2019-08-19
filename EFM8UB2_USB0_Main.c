//-----------------------------------------------------------------------------
// F3xx_USB_Main.c
//-----------------------------------------------------------------------------
// Copyright 2005 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This application will communicate with a PC across the USB interface.
// The device will appear to be a mouse, and will manipulate the cursor
// on screen.
//
// How To Test:    See Readme.txt
//
//
// FID:            3XX000006
// Target:         C8051F32x/C8051F340
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
//                 Silicon Laboratories IDE version 2.6
// Command Line:   See Readme.txt
// Project Name:   F3xx_BlinkyExample
//
//
// Release 1.1
//    -Added feature reports for dimming controls
//    -Added PCA dimmer functionality
//    -16 NOV 2006
// Release 1.0
//    -Initial Revision (PD)
//    -07 DEC 2005
//
//-----------------------------------------------------------------------------
// Header Files
//-----------------------------------------------------------------------------

//#include "c8051f3xx.h"
#include <SI_EFM8UB2_Register_Enums.h>
#include <SI_EFM8UB2_Defs.h>
#include "EFM8UB2_USB0_Register.h"
#include "EFM8UB2_Blink_Control.h"
#include "EFM8UB2_USB0_InterruptServiceRoutine.h"
#include "EFM8UB2_USB0_Descriptor.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------
extern void check_I2C(void);

extern void EEPROM_ReadArray (unsigned char SM_addr , unsigned char src_addr,unsigned char* dest_addr,unsigned char len);
extern void EEPROM_WordWrite(unsigned char SM_addr , unsigned char addr, unsigned char* src_addr,unsigned char len );
extern void enter_DefaultMode_from_RESET(void);

int  cc=0;
void delay2(unsigned int p) {
	unsigned int i,j;
	for (i=0; i<p; i++) {
		for (j=0; j<32767; j++);
	}
}
//-----------------------------------------------------------------------------
// Main Routine
//-----------------------------------------------------------------------------
void main(void)
{
	unsigned char bat_buff[8];
   unsigned char batv1, batv2;

   enter_DefaultMode_from_RESET();

   PCA0MD &= ~0x40;
   HFO0CN |= 0x03;

 //  System_Init ();
   Usb_Init ();

   check_I2C();
  // EA = 1;
   IE |=IE_EA__ENABLED;
   BLINK_SELECTOR=0;
   while (1)
   {
	 EEPROM_ReadArray( 0x14, 0x01, &bat_buff, 2);		//Read Bat State
	 cc = (bat_buff[0]& 0x03);
	 cc = ((bat_buff[0]& 0x30) >>2)|cc;
	 cc = ((bat_buff[1]& 0x03) <<4)|cc;
	 if(bat_buff[0] & 0x01){
		 bat_buff[1]=0x10;
		 EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
		 EEPROM_ReadArray( 0x16,0x0d, &batv1, 1);		//Read Bat1 State
		 BLINK_SELECTOR = batv1;
//		 SendPacket (IN_BLINK_SELECTORID);
	 }else {
		 BLINK_SELECTOR = batv1 = 0;
//		 SendPacket (IN_BLINK_SELECTORID);
	 }
//	 SendPacket (IN_BLINK_SELECTORID);
//	 delay2(10);

	 EEPROM_ReadArray( 0x14, 0x01, &bat_buff, 2);		//Read Bat State
	 cc = (bat_buff[0]& 0x03);
	 cc = ((bat_buff[0]& 0x30) >>2)|cc;
	 cc = ((bat_buff[1]& 0x03) <<4)|cc;
	 if(bat_buff[0] & 0x02){
		 bat_buff[1]=0x20;
		 EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
		 EEPROM_ReadArray( 0x16,0x0d, &batv2, 1);		//Read Bat1 State
		 BLINK_SELECTOR = batv2;
//		 SendPacket (IN_BLINK_SELECTORID);
	 }else {
		 BLINK_SELECTOR = batv2 = 0;
//		 SendPacket (IN_BLINK_SELECTORID);
	 }
	 BLINK_SELECTOR = (batv1+batv2)/2;
	 SendPacket (IN_BLINK_SELECTORID);
	 delay2(20);
   }
}

