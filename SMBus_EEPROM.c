//-----------------------------------------------------------------------------
// SMBus_EEPROM.c
//-----------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//
// Program Description:
//
// This example demonstrates how the EFM8UB2 SMBus interface can communicate
// with a 256 byte I2C Serial EEPROM (Microchip 24LC02B).
// - Interrupt-driven SMBus implementation
// - Only master states defined (no slave or arbitration)
// - Timer1 used as SMBus clock source
// - Timer2 used by SMBus for SCL low timeout detection
// - SCL frequency defined by <SMB_FREQUENCY> constant
// - Pinout:
//    P1.2 -> SDA (SMBus)
//    P1.3 -> SCL (SMBus)
//
//    P1.6 -> LED
//
//    all other port pins unused
//
// How To Test:
//
// 1) Place the switch in "AEM" mode.
// 2) Connect the device to a 24LC02B serial EEPROM (see the EEPROM datasheet
//    for the pinout information).
// 3) Connect the EFM8UB2 STK board to a PC using a mini USB cable.
// 4) Compile and download code to the EFM8UB2 STK board.
//    In Simplicity Studio IDE, select Run -> Debug from the menu bar,
//    click the Debug button in the quick menu, or press F11.
// 5) Run the code.
//    In Simplicity Studio IDE, select Run -> Resume from the menu bar,
//    click the Resume button in the quick menu, or press F8.
//         a) the test will indicate proper communication with the EEPROM by
//            turning on the LED0 at the end the end of the test
//         b) the test can also be verified by running to the if statements
//            in main and checking the sent and received values by adding
//            the variables to the Watch Window
//
//
// Target:         EFM8UB2
// Tool chain:     Simplicity Studio / Keil C51 9.51
// Command Line:   None
//
// Release 1.0 (BL)
//    - Initial Release
//    - 13 JAN 2015
//

//-----------------------------------------------------------------------------
// Includes and Device-Specific Parameters
//-----------------------------------------------------------------------------

#include <SI_EFM8UB2_Register_Enums.h>
#include "InitDevice.h"

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

// System clock frequency in Hz
#define  SYSCLK         12000000

#define  SMB_FREQUENCY  30000          // Target SCL clock rate
                                       // This example supports between 10kHz
                                       // and 100kHz

#define  WRITE          0x00           // SMBus WRITE command
#define  READ           0x01           // SMBus READ command

// Device addresses (7 bits, lsb is a don't care)
//#define  EEPROM_ADDR    0xA0           // Device address for slave target
                                       // Note: This address is specified
                                       // in the Microchip 24LC02B
                                       // datasheet.
#define  EEPROM_ADDR    0x14

// SMBus Buffer Size
#define  SMB_BUFF_SIZE  0x08           // Defines the maximum number of bytes
                                       // that can be sent or received in a
                                       // single transfer

// Status vector - top 4 bits only
#define  SMB_MTSTA      0xE0           // (MT) start transmitted
#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
#define  SMB_MRDB       0x80           // (MR) data byte received
// End status vector definition

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
uint8_t* pSMB_DATA_IN;           // Global pointer for SMBus data
                                       // All receive data is written here

uint8_t SMB_SINGLEBYTE_OUT;      // Global holder for single byte writes.

uint8_t* pSMB_DATA_OUT;          // Global pointer for SMBus data.
                                       // All transmit data is read from here

uint8_t SMB_DATA_LEN;            // Global holder for number of bytes
                                       // to send or receive in the current
                                       // SMBus transfer.

uint8_t WORD_ADDR;               // Global holder for the EEPROM word
                                       // address that will be accessed in
                                       // the next transfer

uint8_t TARGET;                  // Target SMBus slave address

bit SMB_BUSY = 0;                      // Software flag to indicate when the
                                       // EEPROM_ByteRead() or
                                       // EEPROM_ByteWrite()
                                       // functions have claimed the SMBus

bit SMB_RW;                            // Software flag to indicate the
                                       // direction of the current transfer

bit SMB_SENDWORDADDR;                  // When set, this flag causes the ISR
                                       // to send the 8-bit <WORD_ADDR>
                                       // after sending the slave address.

bit SMB_RANDOMREAD;                    // When set, this flag causes the ISR
                                       // to send a START signal after sending
                                       // the word address.
                                       // For the 24LC02B EEPROM, a random read
                                       // (a read from a particular address in
                                       // memory) starts as a write then
                                       // changes to a read after the repeated
                                       // start is sent. The ISR handles this
                                       // switchover if the <SMB_RANDOMREAD>
                                       // bit is set.

bit SMB_ACKPOLL;                       // When set, this flag causes the ISR
                                       // to send a repeated START until the
                                       // slave has acknowledged its address

// 16-bit SI_SFR declarations

SI_SBIT(SDA, SFR_P0, 0);                  // SW1 ='0' means switch pressed
SI_SBIT(SCL, SFR_P0, 1);                  // SW2 ='0' means switch pressed
SI_SBIT(LED, SFR_P1, 6);                      // LED='0' means ON

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
//void EEPROM_ByteWrite(uint8_t addr, uint8_t dat);
//void EEPROM_WriteArray(uint8_t dest_addr, uint8_t* src_addr,
//                       uint8_t len);
//uint8_t EEPROM_ByteRead(uint8_t addr);
//void EEPROM_ReadArray(uint8_t* dest_addr, uint8_t src_addr,
//                     uint8_t len);
void EEPROM_WordWrite(unsigned char SM_addr, unsigned char addr, unsigned char* src_addr,unsigned char len );
void EEPROM_ReadArray ( unsigned char SM_addr,unsigned char src_addr,unsigned char* dest_addr,unsigned char len);
extern void System_Init (void);
extern void Usb_Init (void);
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
//
// Main routine performs all configuration tasks, then loops forever sending
// and receiving SMBus data to the slave EEPROM.
void check_I2C(){
	int i;
	// If slave is holding SDA low because of an improper SMBus reset or error
	   while(!SDA)
	   {
	      // Provide clock pulses to allow the slave to advance out
	      // of its current state. This will allow it to release SDA.
	      XBR1 = 0x40;                     // Enable Crossbar
	      SCL = 0;                         // Drive the clock low
	      for(i = 0; i < 255; i++);        // Hold the clock low
	      SCL = 1;                         // Release the clock
	      while(!SCL);                     // Wait for open-drain
	                                       // clock output to rise
	      for(i = 0; i < 10; i++);         // Hold the clock high
	      XBR1 = 0x00;                     // Disable Crossbar
	   }
}
/*
void main (void)
{
   unsigned char in_buff[8] = {0};              // Incoming data buffer
   unsigned char out_buff[8] = "ABCDEFG";       // Outgoing data buffer

   unsigned long bat;

   uint8_t temp_char;            // Temporary variable
   bit error_flag = 0;                 // Flag for checking EEPROM contents
   uint8_t i;                    // Temporary counter variable

   PCA0MD &= ~0x40;                    // WDTE = 0 (disable watchdog timer)

   HFO0CN |= 0x03;

   check_I2C();


   enter_DefaultMode_from_RESET();
   System_Init ();
   Usb_Init ();


   in_buff[0]=0x00;
   in_buff[1]=0x10;

   EEPROM_WordWrite(0x14,0x01,in_buff,2);
   EEPROM_ReadArray ( 0x14,0x01,&out_buff,2);



   while(1);

}*/
//-----------------------------------------------------------------------------
// Support Functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// EEPROM_ByteWrite ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t addr - address to write in the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t dat - data to write to the address <addr> in the EEPROM
//                        range is full range of character: 0 to 255
//
// This function writes the value in <dat> to location <addr> in the EEPROM
// then polls the EEPROM until the write is complete.
//
void EEPROM_ByteWrite(uint8_t addr, uint8_t dat)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // Mark next transfer as a write
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 0;                 // Do not send a START signal after
                                       // the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling (The ISR
                                       // will automatically restart the
                                       // transfer if the slave does not
                                       // acknoledge its address.

   // Specify the Outgoing Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   SMB_SINGLEBYTE_OUT = dat;           // Store <dat> (local variable) in a
                                       // global variable so the ISR can read
                                       // it after this function exits

   // The outgoing data pointer points to the <dat> variable
   pSMB_DATA_OUT = &SMB_SINGLEBYTE_OUT;

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;

}


//------------------------------------------------------------------------------------
// EEPROM_ByteWrite ()
//------------------------------------------------------------------------------------
//
// This function writes the value in <dat> to location <addr> in the EEPROM then
// polls the EEPROM until the write is complete.
//
void EEPROM_WordWrite(unsigned char SM_addr, unsigned char addr, unsigned char* src_addr,unsigned char len )
{
   while (SMB_BUSY);                         // Wait for SMBus to be free.
   SMB_BUSY = 1;                             // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   //TARGET = SLAVE_ADDR;                     // Set target slave address
	TARGET = SM_addr;
   SMB_RW = WRITE;                           // Mark next transfer as a write
   SMB_SENDWORDADDR = 1;                     // Send Word Address after Slave Address
   SMB_RANDOMREAD = 0;                       // Do not send a START signal after
                                             // the word address
   SMB_ACKPOLL = 1;                          // Enable Acknowledge Polling (The ISR
                                             // will automatically restart the
                                             // transfer if the slave does not
                                             // acknoledge its address.

   // Specify the Outgoing Data
   WORD_ADDR = addr;                         // Set the target address in the
                                             // EEPROM's internal memory space


   pSMB_DATA_OUT = src_addr;      // The outgoing data pointer points to
                                             // the <dat> variable.

   SMB_DATA_LEN = len;                         // Specify to ISR that the next transfer
                                             // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;

}


//------------------------------------------------------------------------------------
// EEPROM_ReadArray ()
//------------------------------------------------------------------------------------
// Reads up to 256 data bytes from the EEPROM slave specified by the <EEPROM_ADDR>
// constant.
//
void EEPROM_ReadArray ( unsigned char SM_addr,unsigned char src_addr,unsigned char* dest_addr,
                       unsigned char len)
{
   while (SMB_BUSY);                         // Wait for SMBus to be free.
   SMB_BUSY = 1;                             // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   //TARGET = SLAVE_ADDR;                     // Set target slave address
	TARGET = SM_addr;
   SMB_RW = WRITE;                           // A random read starts as a write
                                             // then changes to a read after
                                             // the repeated start is sent. The
                                             // ISR handles this switchover if
                                             // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;                     // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                       // Send a START after the word address
   SMB_ACKPOLL = 1;                          // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = src_addr;                     // Set the target address in the
                                             // EEPROM's internal memory space

   pSMB_DATA_IN = (unsigned char*) dest_addr;// Set the the incoming data pointer


   SMB_DATA_LEN = len;                       // Specify to ISR that the next transfer
                                             // will contain <len> data bytes


   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                          // Wait until data is read

}


/*
//-----------------------------------------------------------------------------
// EEPROM_WriteArray ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t dest_addr - beginning address to write to in the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t* src_addr - pointer to the array of data to be written
//                        range is full range of character: 0 to 255
//
//   3) uint8_t len - length of the array to be written to the EEPROM
//                        range is full range of character: 0 to 255
//
// Writes <len> data bytes to the EEPROM slave specified by the <EEPROM_ADDR>
// constant.
//
void EEPROM_WriteArray(uint8_t dest_addr, uint8_t* src_addr,
                       uint8_t len)
{
   uint8_t i;
   uint8_t* pData = (uint8_t*) src_addr;

   for( i = 0; i < len; i++ ){
      EEPROM_ByteWrite(dest_addr++, *pData++);
   }

}

//-----------------------------------------------------------------------------
// EEPROM_ByteRead ()
//-----------------------------------------------------------------------------
//
// Return Value :
//   1) uint8_t data - data read from address <addr> in the EEPROM
//                        range is full range of character: 0 to 255
//
// Parameters   :
//   1) uint8_t addr - address to read data from the EEPROM
//                        range is full range of character: 0 to 255
//
// This function returns a single byte from location <addr> in the EEPROM then
// polls the <SMB_BUSY> flag until the read is complete.
//
uint8_t EEPROM_ByteRead(uint8_t addr)
{
   uint8_t retval;               // Holds the return value

   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // A random read starts as a write
                                       // then changes to a read after
                                       // the repeated start is sent. The
                                       // ISR handles this switchover if
                                       // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                 // Send a START after the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   pSMB_DATA_IN = &retval;             // The incoming data pointer points to
                                       // the <retval> variable.

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                    // Wait until data is read

   return retval;

}

//-----------------------------------------------------------------------------
// EEPROM_ReadArray ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t* dest_addr - pointer to the array that will be filled
//                                 with the data from the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t src_addr - beginning address to read data from the EEPROM
//                        range is full range of character: 0 to 255
//
//   3) uint8_t len - length of the array to be read from the EEPROM
//                        range is full range of character: 0 to 255
//
// Reads up to 256 data bytes from the EEPROM slave specified by the
// <EEPROM_ADDR> constant.
//
void EEPROM_ReadArray (uint8_t* dest_addr, uint8_t src_addr,
                       uint8_t len)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // A random read starts as a write
                                       // then changes to a read after
                                       // the repeated start is sent. The
                                       // ISR handles this switchover if
                                       // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                 // Send a START after the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = src_addr;               // Set the target address in the
                                       // EEPROM's internal memory space

   // Set the the incoming data pointer
   pSMB_DATA_IN = (uint8_t*) dest_addr;

   SMB_DATA_LEN = len;                 // Specify to ISR that the next transfer
                                       // will contain <len> data bytes

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                    // Wait until data is read

}
*/


//-----------------------------------------------------------------------------
// T0_Wait_ms
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t ms - number of milliseconds to wait
//                        range is full range of character: 0 to 255
//
// Configure Timer0 to wait for <ms> milliseconds using SYSCLK as its time
// base.
//
void T0_Waitms (uint8_t ms)
{
   TCON &= ~0x30;                      // Stop Timer0; Clear TCON_TF0
   TMOD &= ~0x0f;                      // 16-bit free run mode
   TMOD |=  0x01;

   CKCON0 |= 0x04;                      // Timer0 counts SYSCLKs

   while (ms) {
      TCON_TR0 = 0;                         // Stop Timer0
      TH0 = ((-SYSCLK/1000) >> 8);     // Overflow in 1ms
      TL0 = ((-SYSCLK/1000) & 0xFF);
      TCON_TF0 = 0;                         // Clear overflow indicator
      TCON_TR0 = 1;                         // Start Timer0
      while (!TCON_TF0);                    // Wait for overflow
      ms--;                            // Update ms counter
   }

   TCON_TR0 = 0;                            // Stop Timer0
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
