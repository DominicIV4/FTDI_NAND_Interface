/* 
 * This file is part of the ftdi-nand-flash-reader distribution (https://github.com/maehw/ftdi-nand-flash-reader).
 * Copyright (c) 2018 maehw.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * \file bitbang_ft2232.c
 * \brief NAND flash reader based on FTDI FT2232 IC in bit-bang IO mode
 * Interfacing NAND flash devices with an x8 I/O interface for address and data.
 * Additionally the signals Chip Enable (nCE), Write Enable (nWE), Read Enable (nRE), 
 * Address Latch Enable (ALE), Command Latch Enable (CLE), Write Protect (nWP) 
 * and Ready/Busy (RDY) on the control bus are used.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <conio.h> 
// #include "ftdi.h"
#include <libftdi1/ftdi.h> // HAD TO CHANGE THIS INCLUDE LINE :(

#include <math.h>

#define NV_VT_SWEEP_START 128
#define NV_VT_SWEEP_END   255
#define PV_VT_SWEEP_START 0
#define PV_VT_SWEEP_END   127


/* FTDI FT2232H VID and PID */
#define FT2232H_VID 0x0403
#define FT2232H_PID 0x6010
 
#define SIZE 8194
#define BLOCK_SIZE 1222

 #define EXPECTED_STATUS 0xE0
/* Pins on ADBUS0..7 (I/O bus) */
#define PIN_DIO0 0x01
#define PIN_DIO1 0x02
#define PIN_DIO2 0x04
#define PIN_DIO3 0x08
#define PIN_DIO4 0x10
#define PIN_DIO5 0x20
#define PIN_DIO6 0x40
#define PIN_DIO7 0x80
#define IOBUS_BITMASK_WRITE 0xFF
#define IOBUS_BITMASK_READ  0x00

/* Pins on BDBUS0..7 (control bus) */
#define PIN_CLE  0x01
#define PIN_ALE  0x02
#define PIN_nCE  0x04
#define PIN_nWE  0x08
#define PIN_nRE  0x10
#define PIN_nWP  0x20
#define PIN_RDY  0x40 /* READY / nBUSY output signal */
#define PIN_DQS  0x80
#define CONTROLBUS_BITMASK 0xBF /* 0b1011 1111 = 0xBF */
#define CONTROLBUS_BITMASK_WRITE 0xBF /* 0b1011 1111 = 0xBF */
#define CONTROLBUS_BITMASK_READ 0x3F /* 0b0011 1111 = 0x3F */

#define STATUSREG_IO0  0x01

#define REALWORLD_DELAY 10 /* 10 usec */

#define PAGE_SIZE 16384
#define PAGE_SIZE_NOSPARE 14416

const unsigned char CMD_READID = 0x90; /* read ID register */
const unsigned char CMD_READ1[2] = { 0x00, 0x30 }; /* page read */
const unsigned char CMD_BLOCKERASE[2] = { 0x60, 0xD0 }; /* block erase */
const unsigned char CMD_READSTATUS = 0x70; /* read status */
const unsigned char CMD_PAGEPROGRAM[2] = { 0x80, 0x15 }; /* program page */
const unsigned char CMD_RESET = 0xFF;
const unsigned char CMD_READ_PARAM_PAGE = 0xEC;
 const unsigned char CMD_READ_OFFSET_PREFIX = 0x2E;
const unsigned char CMD_LP = 0x01; // lower page 01h
const unsigned char CMD_UP = 0x02; // upper page 02h
const unsigned char CMD_XP = 0x03; // (e)Xtra page 03h

typedef enum { OFF=0, ON=1 } onoff_t;
typedef enum { IOBUS_IN=0, IOBUS_OUT=1 } iobus_inout_t;
typedef enum { CONTROLBUS_IN=0, CONTROLBUS_OUT=1 } controlbus_inout_t;

unsigned char iobus_value;
unsigned char controlbus_value;

struct ftdi_context *nandflash_iobus, *nandflash_controlbus;

unsigned char LP_buffer_read[8194];
unsigned char UP_buffer_read[8194];
unsigned char XP_buffer_read[8194];


 void controlbus_reset_value()
 {
     controlbus_value = 0x0c;
 }
 
 void controlbus_pin_set(unsigned char pin, onoff_t val)
 {
     if(val == ON)
         controlbus_value |= pin;
     else
         controlbus_value &= (unsigned char)0xFF ^ pin;
 }
 
 void controlbus_update_output()
 {
     unsigned char buf[1]; /* buffer for FTDI function needs to be an array */
     buf[0] = controlbus_value;
     ftdi_write_data(nandflash_controlbus, buf, 1);
 }
 
 void controlbus_set_direction(controlbus_inout_t inout)
 {
     if( inout == CONTROLBUS_OUT )
         ftdi_set_bitmode(nandflash_controlbus, CONTROLBUS_BITMASK_WRITE, BITMODE_BITBANG);
     else if( inout == CONTROLBUS_IN )
         ftdi_set_bitmode(nandflash_controlbus, CONTROLBUS_BITMASK_READ, BITMODE_BITBANG);        
 }
 
 void toggle_pin_dqs() {
     // Check current state of PIN_DQS
     if (controlbus_value & PIN_DQS) {
         // If PIN_DQS is ON, turn it OFF
         controlbus_pin_set(PIN_DQS, OFF);
     } else {
         // If PIN_DQS is OFF, turn it ON
         controlbus_pin_set(PIN_DQS, ON);
     }
     // Update the control bus output to apply the change
     controlbus_update_output();
 }
 
 void toggle_pin_RE() {
     // Check current state of PIN_DQS
     if (controlbus_value & PIN_nRE) {
         // If PIN_DQS is ON, turn it OFF
         controlbus_pin_set(PIN_nRE, OFF);
     } else {
         // If PIN_DQS is OFF, turn it ON
         controlbus_pin_set(PIN_nRE, ON);
     }
     // Update the control bus output to apply the change
     controlbus_update_output();
 }
 
 
//  void test_controlbus()
//  {
//      #define CONTROLBUS_TEST_DELAY 1000000 /* 1 sec */
 
//      for(int i = 0; i<10;i++)
//      { 
//          //printf(" RE Toggled\n");
//          toggle_pin_RE();
//          usleep(CONTROLBUS_TEST_DELAY*2);
//      }
 
//      //printf("  CLE on\n");
//      controlbus_pin_set(PIN_CLE, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  ALE on\n");
//      controlbus_pin_set(PIN_ALE, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nCE on\n");
//      controlbus_pin_set(PIN_nCE, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nWE on\n");
//      controlbus_pin_set(PIN_nWE, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
 
//      //printf("  nRE on\n");
//      controlbus_pin_set(PIN_nRE, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nWP on\n");
//      controlbus_pin_set(PIN_nWP, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  DQS on\n");
//      controlbus_pin_set(PIN_DQS, ON);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
 
//      //printf("  CLE off\n");
//      controlbus_pin_set(PIN_CLE, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  ALE off\n");
//      controlbus_pin_set(PIN_ALE, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nCE off\n");
//      controlbus_pin_set(PIN_nCE, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nWE off\n");
//      controlbus_pin_set(PIN_nWE, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nRE off\n");
//      controlbus_pin_set(PIN_nRE, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  nWP off\n");
//      controlbus_pin_set(PIN_nWP, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
 
//      //printf("  DQS off\n");
//      controlbus_pin_set(PIN_DQS, OFF);
//      controlbus_update_output(nandflash_controlbus);
//      usleep(CONTROLBUS_TEST_DELAY);
//  }
 
 void iobus_set_direction(iobus_inout_t inout)
 {
     if( inout == IOBUS_OUT )
         ftdi_set_bitmode(nandflash_iobus, IOBUS_BITMASK_WRITE, BITMODE_BITBANG);
     else if( inout == IOBUS_IN )
         ftdi_set_bitmode(nandflash_iobus, IOBUS_BITMASK_READ, BITMODE_BITBANG);        
 }
 
 
 void iobus_reset_value()
 {
     iobus_value = 0x00;
 }
 
 void iobus_pin_set(unsigned char pin, onoff_t val)
 {
     if(val == ON)
         iobus_value |= pin;
     else
         iobus_value &= (unsigned char)0xFF ^ pin;
 }
 
 void iobus_set_value(unsigned char value)
 {
     iobus_value = value;
 }
 
 void iobus_update_output()
 {
     unsigned char buf[1]; /* buffer for FTDI function needs to be an array */
     buf[0] = iobus_value;
     ftdi_write_data(nandflash_iobus, buf, 1);
 }
 
 unsigned char iobus_read_input()
 {
     unsigned char buf; 
     //ftdi_read_data(nandflash_iobus, buf, 1); /* buffer for FTDI function needed to be an array */
     ftdi_read_pins(nandflash_iobus, &buf);
     return buf;
 }
 
 unsigned char controlbus_read_input()
 {
     unsigned char buf;
     //ftdi_read_data(nandflash_controlbus, buf, 1); /* buffer for FTDI function needed to be an array */
     ftdi_read_pins(nandflash_controlbus, &buf);
     return buf;
 }
 
 
//  void test_iobus()
//  {
//      #define IOBUS_TEST_DELAY 1000000 /* 1 sec */
 
//      //printf("  DIO0 on\n");
//      iobus_pin_set(PIN_DIO0, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO1 on\n");
//      iobus_pin_set(PIN_DIO1, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO2 on\n");
//      iobus_pin_set(PIN_DIO2, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO3 on\n");
//      iobus_pin_set(PIN_DIO3, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO4 on\n");
//      iobus_pin_set(PIN_DIO4, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO5 on\n");
//      iobus_pin_set(PIN_DIO5, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO6 on\n");
//      iobus_pin_set(PIN_DIO6, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      //printf("  DIO7 on\n");
//      iobus_pin_set(PIN_DIO7, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
 
//      iobus_pin_set(PIN_DIO0, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO1, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO2, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO3, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO4, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO5, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO6, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      iobus_pin_set(PIN_DIO7, OFF);
//      iobus_update_output(nandflash_iobus);
//      usleep(IOBUS_TEST_DELAY);
 
//      usleep(5 * IOBUS_TEST_DELAY);
//      iobus_set_value(0xFF);
//      iobus_update_output();
//      usleep(5 * IOBUS_TEST_DELAY);
//      iobus_set_value(0xAA);
//      iobus_update_output();
//      usleep(5 * IOBUS_TEST_DELAY);
//      iobus_set_value(0x55);
//      iobus_update_output();
//      usleep(5 * IOBUS_TEST_DELAY);
//      iobus_set_value(0x00);
//      iobus_update_output();
 
     
//      iobus_pin_set(PIN_DIO0, ON);
//      iobus_pin_set(PIN_DIO2, ON);
//      iobus_pin_set(PIN_DIO4, ON);
//      iobus_pin_set(PIN_DIO6, ON);
//      iobus_update_output(nandflash_iobus);
//      usleep(2* 100000);
 
//  }
 
 /* "Command Input bus operation is used to give a command to the memory device. Command are accepted with Chip
 Enable low, Command Latch Enable High, Address Latch Enable low and Read Enable High and latched on the rising
 edge of Write Enable. Moreover for commands that starts a modify operation (write/erase) the Write Protect pin must be
 high."" */
int latch_command(unsigned char command)
 {

    controlbus_set_direction(CONTROLBUS_OUT);
     controlbus_pin_set(PIN_DQS, ON);
     controlbus_pin_set(PIN_nRE, ON);
     controlbus_pin_set(PIN_nCE, OFF);
     controlbus_pin_set(PIN_nWP, ON); /* nWP low provides HW protection against undesired modify (program / erase) operations */
     controlbus_update_output();
     /* check if ALE is low and nRE is high */
     if( controlbus_value & PIN_nCE )
     {
         //printf(stderr, "latch_command requires nCE pin to be low\n");
         return EXIT_FAILURE;
     }
     else if( ~controlbus_value & PIN_nRE )
     {
         //printf(stderr, "latch_command requires nRE pin to be high\n");
         return EXIT_FAILURE;
     }
 
     else if( ~controlbus_value & PIN_DQS )
     {
         //printf(stderr, "latch_command requires DQS pin to be high\n");
         return EXIT_FAILURE;
     }
 
     controlbus_pin_set(PIN_CLE, ON);
     controlbus_pin_set(PIN_ALE, OFF);
     controlbus_update_output();
 
     usleep(REALWORLD_DELAY*1);
 
     // toggle nWE low
     ////printf("  setting nWE low\n");
     controlbus_pin_set(PIN_nWE, OFF);
     controlbus_update_output();
 
     // change I/O pins
     ////printf("  setting I/O bus to command\n");
     iobus_set_direction(IOBUS_OUT);
     iobus_set_value(command);
    //  printf("COMMAND TO NAND: %02X\n", command);
     iobus_update_output();
 
     usleep(REALWORLD_DELAY*1);
 
     // toggle nWE back high (acts as clock to latch the command!)
     ////printf("  setting nWE high\n");
     controlbus_pin_set(PIN_nWE, ON);
     controlbus_update_output();
 
     usleep(10*REALWORLD_DELAY);
    // after toggling nWE high to latch the command...
    controlbus_pin_set(PIN_CLE, OFF);
    controlbus_pin_set(PIN_ALE, OFF);
    controlbus_pin_set(PIN_nWE, ON);
    controlbus_pin_set(PIN_nRE, ON);
    controlbus_update_output();
     // toggle CLE low

 
     return 0;
 }
 
 /** 
  * "Address Input bus operation allows the insertion of the memory address. 
  * Five cycles are required to input the addresses for the 4Gbit devices. 
  * Addresses are accepted with Chip Enable low, Address Latch Enable High, Command Latch Enable low and 
  * Read Enable High and latched on the rising edge of Write Enable.
  * Moreover for commands that starts a modifying operation (write/erase) the Write Protect pin must be high. 
  * See Figure 5 and Table 13 for details of the timings requirements.
  * Addresses are always applied on IO7:0 regardless of the bus configuration (x8 or x16).""
  */
 int latch_address(unsigned char address[], unsigned int addr_length)
 {
     unsigned int addr_idx = 0;
     controlbus_set_direction(CONTROLBUS_OUT);

     /* check if ALE is low and nRE is high */
     if( controlbus_value & PIN_nCE )
     {
         //printf(stderr, "latch_address requires nCE pin to be low\n");
         return EXIT_FAILURE;
     }
 
     else if( ~controlbus_value & PIN_nRE )
     {
         //printf(stderr, "latch_address requires nRE pin to be high\n");
         return EXIT_FAILURE;
     }
 
     else if( ~controlbus_value & PIN_DQS )
     {
         //printf(stderr, "latch_address requires DQS pin to be high\n");
         return EXIT_FAILURE;
     }
 
     /* toggle ALE high (activates the latching of the IO inputs inside
      * the Address Register on the Rising edge of nWE. */
     controlbus_pin_set(PIN_ALE, ON);
     controlbus_pin_set(PIN_CLE, OFF);
     controlbus_update_output();
    iobus_set_direction(IOBUS_OUT);
     for(addr_idx = 0; addr_idx < addr_length; addr_idx++)
     {
         // toggle nWE low
         controlbus_pin_set(PIN_nWE, OFF);
         controlbus_update_output();
         usleep(10*REALWORLD_DELAY);
 
         // change I/O pins
         iobus_set_value(address[addr_idx]);
        //  printf("ADDRESS TO NAND: %02X\n", address[addr_idx]);
         iobus_update_output();
         usleep(10*REALWORLD_DELAY); /* TODO: assure setup i2+1 */
 
         // toggle nWE back high (acts as clock to latch the current address byte!)
         controlbus_pin_set(PIN_nWE, ON);
         controlbus_update_output();
         usleep(10*REALWORLD_DELAY); /* TODO: assure hold i2+1 */
     }
 
     // toggle ALE low
     controlbus_pin_set(PIN_ALE, OFF);
     controlbus_pin_set(PIN_CLE, ON);
     controlbus_update_output();

     // wait for ALE to nRE Level tAR before nRE is taken low (nanoseconds!)
 
     return 0;
 }
 
 /* Data Output bus operation allows to read data from the memory array and to 
  * check the status register content, the EDC register content and the ID data.
  * Data can be serially shifted out by toggling the Read Enable pin with Chip 
  * Enable low, Write Enable High, Address Latch Enable low, and Command Latch 
  * Enable low. */
int latch_register(unsigned char reg[], unsigned int reg_length)
 {
    controlbus_set_direction(CONTROLBUS_OUT);
    iobus_set_direction(IOBUS_IN);

     unsigned int addr_idx = 0;
     controlbus_pin_set(PIN_DQS, OFF);
     controlbus_pin_set(PIN_nRE, ON);
     controlbus_pin_set(PIN_nCE, OFF);
     controlbus_pin_set(PIN_CLE, OFF);
     controlbus_pin_set(PIN_ALE, OFF);
     controlbus_pin_set(PIN_nWE, ON);
     controlbus_pin_set(PIN_nWP, ON); /* nWP low provides HW protection against undesired modify (program / erase) operations */
     controlbus_update_output();
     usleep(REALWORLD_DELAY);
     /* check if ALE is low and nRE is high */
     if( controlbus_value & PIN_nCE )
     {
         //printf(stderr, "latch_address requires nCE pin to be low\n");
         return EXIT_FAILURE;
     }
     else if( ~controlbus_value & PIN_nWE )
     {
         //printf(stderr, "latch_address requires nWE pin to be high\n");
         return EXIT_FAILURE;
     }
     else if( controlbus_value & PIN_ALE )
     {
         //printf(stderr, "latch_address requires ALE pin to be low\n");
         return EXIT_FAILURE;
     }
 
     else if( controlbus_value & PIN_CLE )
     {
         //printf(stderr, "latch_address requires CLE pin to be low\n");
         return EXIT_FAILURE;
     }
     
     else if(~controlbus_value & PIN_nRE)
     {
         //printf(stderr, "latch_register RE pin to start with high\n");
         return EXIT_FAILURE;
     }
 
    iobus_set_direction(IOBUS_IN);
    controlbus_set_direction(CONTROLBUS_OUT);   // keep driving control lin
    //printf("CONTROL BUS = %02X\n", controlbus_read_input());

     for(addr_idx = 0; addr_idx < reg_length; addr_idx++)
     {
         /* toggle nRE ; acts like a clock to latch out the data;
          * data is valid tREA after the rising/falling edge of nRE 
        //   * (also increments the internal column address counter by one) */

          
// The below loop is an active readout of the IO bus the progresses when pressing 'g'
//         while (1)
// {
//     uint8_t val = iobus_read_input();

//     printf("\rIOBUS: 0x%02X (%c)   ", val, 
//            (val >= 32 && val <= 126) ? val : '.');

//     fflush(stdout);   // force update immediately

//     if (_kbhit())
//     {
//         char c = _getch();
//         if (c == 'G' || c == 'g')
//         {
//             break;
//         }
//     }

//     Sleep(10);  // optional: prevent 100% CPU usage
// }


        toggle_pin_RE();
        //printf("%02X = %c \n", iobus_read_input(), iobus_read_input());
        usleep(REALWORLD_DELAY*2);
        reg[addr_idx] = iobus_read_input();
        usleep(REALWORLD_DELAY); /* TODO: assure tREA i2+1 */
     }
    
     controlbus_set_direction(CONTROLBUS_OUT);
 
     return 0;
 }

 void check_ID_register(unsigned char* ID_register)
 {
     unsigned char ID_register_exp[5] = { 0xAD, 0xDC, 0x10, 0x95, 0x54 };
 
     /* output the retrieved ID register content */
     //printf("actual ID register:   0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
         // ID_register[0], ID_register[1], ID_register[2],
         // ID_register[3], ID_register[4] ); 
 
     /* output the expected ID register content */
     //printf("expected ID register: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
         // ID_register_exp[0], ID_register_exp[1], ID_register_exp[2],
         // ID_register_exp[3], ID_register_exp[4] ); 
 
     if( strncmp( (char *)ID_register_exp, (char *)ID_register, 5 ) == 0 )
     {
         //printf("PASS: ID register did match\n");
     }
     else
     {
         //printf("FAIL: ID register did not match\n");
     }
 }
 

/* Address Cycle Map calculations */
void get_address(unsigned int* address, unsigned char* page_address)
{
    // unsigned char page_address[]={0x00, 0x00, 0x00, 0x00, 0x00};
    // {<column address>,<page address>,<plane address>,<block address>}
    page_address[0] = (unsigned char)(address[0]) & 0xFF; // CA 7-0
    page_address[1] = (unsigned char)(address[0]>>8) & 0x7F; // CA 14-8
    page_address[2] = (unsigned char)(address[1]) & 0xFF; // PA 7-0
    page_address[3] = (unsigned char)(address[1]>>8) & 0x03; // BA 15-13 PS 12-10 PA 9-8
    page_address[3] = page_address[3] | ((unsigned char)(address[2]<<2) & 0x1C); 
    page_address[3] = page_address[3] | ((unsigned char)(address[3]<<5) & 0XE0);
    page_address[4] = (unsigned char)(address[3]>>3) & 0x7F;
    page_address[5] = 0x00;

    //printf("  Address cycles are: %d %d %d %d\n", address[0], address[1], address[2], address[3] );
     
}


 /* Address Cycle Map calculations */
 void get_address_cycle_map_x8(uint32_t mem_address, unsigned char* addr_cylces)
 {
     addr_cylces[0] = (unsigned char)(  mem_address & 0x0000000000FF);
     addr_cylces[1] = (unsigned char)( (mem_address & 0x000000007F00) >> 8 );
     addr_cylces[2] = (unsigned char)( (mem_address & 0x000000FF0000) >> 15 );
     addr_cylces[3] = (unsigned char)( (mem_address & 0x0000FF000000) >> 23 );
     addr_cylces[4] = (unsigned char)( (mem_address & 0x00FF00000000) >> 31 );
     addr_cylces[5] = (unsigned char)( (mem_address & 0x010000000000) >> 39 );
 }
 
 void read_page(unsigned char* addr_cylces,  unsigned int data_len, unsigned int prefix_cmd)
 {
     unsigned char mem_large_block[data_len]; /* page content */
 
 
     for(int i=0;i<data_len;i++)
     {
         mem_large_block[i] = 0x11;
     }
 
     // printf("  Address cycles are: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
     //     addr_cylces[0], addr_cylces[1], /* column address */
     //     addr_cylces[2], addr_cylces[3], addr_cylces[4], addr_cylces[5] ); /* row address */

     usleep(10*REALWORLD_DELAY);
     latch_command(prefix_cmd); // prefix cmd for TLC page in cell
     usleep(10*REALWORLD_DELAY);
 
     //printf("Latching first command byte to read a page...\n");
     latch_command(CMD_READ1[0]);
 
     //printf("Latching address cycles...\n");
     latch_address(addr_cylces, 6);
 
     //printf("Latching second command byte to read a page...\n");
     latch_command(CMD_READ1[1]);
 
     // busy-wait for high level at the busy line
     //printf("Checking for busy line...\n");
     unsigned char controlbus_val;
     do
     {
         controlbus_val = controlbus_read_input();
     }
     while( !(controlbus_val & PIN_RDY) );
 
     //printf("  done\n");
 
     //printf("Latching out data block...\n");
     latch_register(mem_large_block, data_len);
                 
     for(int i=0;i<data_len;i++)
     {
         printf("At %d = 0x%02x = %c\n",i, mem_large_block[i], mem_large_block[i]);
     }
 }
 
 
 void read_page_horizontal(unsigned char* addr_cylces, unsigned char* mem_large_block,
                unsigned int data_len, 
                unsigned int prefix_cmd)
 {
    //  unsigned char mem_large_block[data_len]; /* page content */
 
 
     for(int i=0;i<data_len;i++)
     {
         mem_large_block[i] = 0x11;
     }
     // MY CHANGE
     usleep(10*REALWORLD_DELAY);
     latch_command(prefix_cmd); // prefix cmd for TLC page in cell B58R
     usleep(10*REALWORLD_DELAY);

     //printf("Latching first command byte to read a page...\n");
     latch_command(CMD_READ1[0]);
     usleep(10*REALWORLD_DELAY);

     //printf("Latching address cycles...\n");
     latch_address(addr_cylces, 6);
    usleep(10*REALWORLD_DELAY);

 
     //printf("Latching second command byte to read a page...\n");
     usleep(10*REALWORLD_DELAY);

     latch_command(CMD_READ1[1]);
    
     usleep(10*REALWORLD_DELAY);

 
     // busy-wait for high level at the busy line
     //printf("Checking for busy line...\n");
     unsigned char controlbus_val;
     do
     {
         controlbus_val = controlbus_read_input();
     }
     while( !(controlbus_val & PIN_RDY) );
 
     usleep(10*REALWORLD_DELAY);

     latch_register(mem_large_block, data_len);
                 
    int i = 0;
    
    // while loop to print information horizontally
    //     printf("Start Read:\n");
    // for (int i = 0; i < data_len; i++)
    // {
    //     printf("%02x = %c\n", mem_large_block[i], mem_large_block[i]);
    //     //printf("%c", mem_large_block[i]); // prints ascii from encoding
    // }
    // putchar('\n');
    // printf("End Read");

 }

 void read_page_back(unsigned char* addr_cylces, unsigned char* mem_large_block, unsigned int data_len, unsigned int prefix_cmd)
 {
 
     for(int i=0;i<data_len;i++)
     {
         mem_large_block[i] = 0x11;
     }
 
     // printf("  Address cycles are: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
     //     addr_cylces[0], addr_cylces[1], /* column address */
     //     addr_cylces[2], addr_cylces[3], addr_cylces[4], addr_cylces[5] ); /* row address */

     latch_command(prefix_cmd);
 
     //printf("Latching first command byte to read a page...\n");
     latch_command(CMD_READ1[0]);
 
     //printf("Latching address cycles...\n");
     latch_address(addr_cylces, 6);
 
     //printf("Latching second command byte to read a page...\n");
     latch_command(CMD_READ1[1]);
 
     // busy-wait for high level at the busy line
     //printf("Checking for busy line...\n");
     unsigned char controlbus_val;
     do
     {
         controlbus_val = controlbus_read_input();
     }
     while( !(controlbus_val & PIN_RDY) );
 
     //printf("  done\n");
 
     //printf("Latching out data block...\n");
     latch_register(mem_large_block, data_len);
                 
     for(int i=0;i<data_len;i++)
     {
         //printf("At %d = 0x%02x = %c\n",i, mem_large_block[i], mem_large_block[i]);
     }
 }
 
 
 
 /**
  * BlockErase
  *
  * "The Erase operation is done on a block basis.
  * Block address loading is accomplished in there cycles initiated by an Erase Setup command (60h).
  * Only address A18 to A29 is valid while A12 to A17 is ignored (x8).
  *
  * The Erase Confirm command (D0h) following the block address loading initiates the internal erasing process.
  * This two step sequence of setup followed by execution command ensures that memory contents are not
  * accidentally erased due to external noise conditions.
  *
  * At the rising edge of WE after the erase confirm command input,
  * the internal write controller handles erase and erase verify.
  *
  * Once the erase process starts, the Read Status Register command may be entered to read the status register.
  * The system controller can detect the completion of an erase by monitoring the R/B output,
  * or the Status bit (I/O 6) of the Status Register.
  * Only the Read Status command and Reset command are valid while erasing is in progress.
  * When the erase operation is completed, the Write Status Bit (I/O 0) may be checked."
  */
 int erase_block(unsigned char* addr_cylces)
 {
    printf("Erasing block of data\n");
     controlbus_pin_set(PIN_nWP, ON);
 
     //printf("Latching first command byte to erase a block...\n");
     latch_command(CMD_BLOCKERASE[0]); /* block erase setup command */
 
     ////printf("Erasing block of data from memory address 0x%02X\n", mem_address);
     //get_address_cycle_map_x8(mem_address, addr_cylces);
     // printf("  Address cycles are (but: will take only cycles 3..6) : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
     // 	addr_cylces[0], addr_cylces[1], /* column address */
     // 	addr_cylces[2], addr_cylces[3], addr_cylces[4], addr_cylces[5]); /* row address */
 
     //printf("Latching page(row) address (3 bytes)...\n");
     unsigned char address[] = { addr_cylces[2], addr_cylces[3], addr_cylces[4] , addr_cylces[5] };
     latch_address(address, 4);
 
     //printf("Latching second command byte to erase a block...\n");
     latch_command(CMD_BLOCKERASE[1]);
 
     /* tWB: WE High to Busy is 100 ns -> ignore it here as it takes some time for the next command to execute */
 
     // busy-wait for high level at the busy line
     //printf("Checking for busy line...\n");
     unsigned char controlbus_val;
     do
     {
         controlbus_val = controlbus_read_input();
     }
     while( !(controlbus_val & PIN_RDY) );
 
     //printf("  done\n");
 
 
     /* Read status */
     //printf("Latching command byte to read status...\n");
     latch_command(CMD_READSTATUS);
 
     unsigned char status_register;
     latch_register(&status_register, 1); /* data output operation */
 
     /* output the retrieved status register content */
     //printf("Status register content:   0x%02X\n", status_register);
 
 
     /* activate write protection again */
     controlbus_pin_set(PIN_nWP, ON);
 
 
     if(status_register & STATUSREG_IO0)
     {
         //printf(stderr, "FaiDQS to erase block.\n");
         return 1;
     }
     else
     {
         //printf("Successfully erased block.\n");
         return 0;
     }
 }
 
 
 int latch_data_out(unsigned char data[], unsigned int length)
 {
 //	//printf("\n");
     controlbus_set_direction(CONTROLBUS_OUT);
     iobus_set_direction(IOBUS_OUT);
 
     controlbus_pin_set(PIN_nCE, OFF);
     controlbus_pin_set(PIN_nWP, ON);
     controlbus_update_output();
     usleep(REALWORLD_DELAY);
     
     controlbus_pin_set(PIN_nWE, ON);
     controlbus_pin_set(PIN_nRE, ON);
     controlbus_pin_set(PIN_DQS, ON);
     controlbus_update_output();
     usleep(REALWORLD_DELAY);
 
     controlbus_pin_set(PIN_ALE, OFF);
     controlbus_pin_set(PIN_CLE, OFF);
     controlbus_pin_set(PIN_DQS, OFF);
     controlbus_update_output();
     usleep(REALWORLD_DELAY);
 
    iobus_set_direction(IOBUS_OUT);

     for(unsigned int k = 0; k < length; k++)
     {
        if (!(controlbus_read_input() & PIN_RDY)) {
    printf("NOT READY\n");
}
        //printf("data being written to NAND: 0x%02X\n", data[k]);
        //printf("DATA TO NAND: %c\n", data[k]); // print data being snet to nand
         usleep(REALWORLD_DELAY);
         // change I/O pins
         iobus_set_value(data[k]);
         iobus_update_output();
         usleep(REALWORLD_DELAY); /* TODO: assure setup i2+1 */
 
 
         // toggle nWE back high (acts as clock to latch the current address byte!)
         toggle_pin_dqs();
         controlbus_update_output();
         usleep(REALWORLD_DELAY); /* TODO: assure hold i2+1 */
     }
 
     controlbus_pin_set(PIN_CLE, ON);
     controlbus_update_output();
     usleep(REALWORLD_DELAY);
     controlbus_pin_set(PIN_nWE, OFF);
     controlbus_update_output();
     usleep(REALWORLD_DELAY);

     return 0;
 }

 /**
  * Page Program
  *
  * "The device is programmed by page.
  * The number of consecutive partial page programming operation within the same page
  * without an intervening erase operation must not exceed 8 times.
  *
  * The addressing should be done on each pages in a block.
  * A page program cycle consists of a serial data loading period in which up to 2112 bytes of data
  * may be loaded into the data register, followed by a non-volatile programming period where the loaded data
  * is programmed into the appropriate cell.
  *
  * The serial data loading period begins by inputting the Serial Data Input command (80h),
  * followed by the five cycle address inputs and then serial data.
  *
  * The bytes other than those to be programmed do not need to be loaded.
  *
  * The device supports random data input in a page.
  * The column address of next data, which will be entered, may be changed to the address which follows
  * random data input command (85h).
  * Random data input may be operated multiple times regardless of how many times it is done in a page.
  *
  * The Page Program confirm command (10h) initiates the programming process.
  * Writing 10h alone without pre-viously entering the serial data will not initiate the programming process.
  * The internal write state controller automatically executes the algorithms and timings necessary for
  * program and verify, thereby freeing the system controller for other tasks.
  * Once the program process starts, the Read Status Register command may be entered to read the status register.
  * The system controller can detect the completion of a program cycle by monitoring the R/B output,
  * or the Status bit (I/O 6) of the Status Register.
  * Only the Read Status command and Reset command are valid while programming is in progress.
  *
  * When the Page Program is complete, the Write Status Bit (I/O 0) may be checked.
  * The internal write verify detects only errors for "1"s that are not successfully programmed to "0"s.
  *
  * The command register remains in Read Status command mode until another valid command is written to the
  * command register.
  */

// Read status helper funtion for below program
uint8_t nand_read_status(void)
{
    uint8_t st;
    latch_command(0x70);
    latch_register(&st, 1);
    return st;
}
#include <stdbool.h>
#include <stdio.h>

void nand_print_status(void)
{
    uint8_t st;

    latch_command(0x70);
    latch_register(&st, 1);

    printf("NAND Status = 0x%02X\n", st);
}


 int program_page(unsigned char* addr_cylces,
                    unsigned char data[], 
                    unsigned int data_len, 
                    uint8_t prefix_cmd)
 {
 
     /* remove write protection */
     controlbus_pin_set(PIN_nWP, ON);
     
    // MY CHANGE
    usleep(10*REALWORLD_DELAY);
    latch_command(prefix_cmd); // prefix cmd for TLC page in cell
    usleep(10*REALWORLD_DELAY);
    latch_command(CMD_PAGEPROGRAM[0]); /* Serial Data Input command */
 
     //printf("Latching address cycles...\n");
    usleep(1*REALWORLD_DELAY);
    latch_address(addr_cylces, 6);
    
    usleep(1*REALWORLD_DELAY);
     //printf("Latching out the data of the page...\n");
     latch_data_out(data, data_len);
    usleep(1*REALWORLD_DELAY);
     //printf("Latching second command byte to write a page...\n");
     latch_command(CMD_PAGEPROGRAM[1]); /* Page Program confirm command command */
    usleep(1*REALWORLD_DELAY);
     // busy-wait for high level at the busy line
     //printf("Checking for busy line...\n");
     unsigned char controlbus_val;
     do
     {
         controlbus_val = controlbus_read_input();
     }
     while( !(controlbus_val & PIN_RDY) );
     
//      uint8_t st = nand_read_status();
// //     printf("PROG status = 0x%02X\n", st);

//     // Typical: bit0=FAIL (1 fail), bit6=RDY (1 ready)
//     if (st & 0x01) return -1;
    return 0;
 }

 void get_page_dummy_data(unsigned char* page_data, unsigned char data, unsigned int size)
 {
     for(unsigned int k=0; k<size; k++)
     {
         page_data[k]= data;
     }
 }
 
 int countSetBits(const unsigned char *arr, int size) {
     int totalSetBits = 0;
 
     for (int i = 0; i < size; i++) {
         unsigned char value = arr[i];
 
         // Count bits in the current byte
         while (value > 0) {
             totalSetBits += (value & 0x01);  // add 1 if LSB is set
             value >>= 1;                 // shift the bits to the right by 1
         }
     }
 
     return totalSetBits;
 }
 
 int countUnSetBits(const unsigned char *arr, int size) {
     int totalZeroBits = 0;
 
     for (int i = 0; i < size; i++) {
         unsigned char value = arr[i];
 
         // Check each of the 8 bits in the current byte
         for (int j = 0; j < 8; j++) {
             // If the least significant bit is 0, count it
             if ((value & 0x01) == 0) {
                 totalZeroBits++;
             }
             // Shift to the next bit
             value >>= 1;
         }
     }
 
     return totalZeroBits;
 }


 void uchar_to_binary(unsigned char byte, char *out) {
    for (int i = 7; i >= 0; i--) {
        out[7 - i] = (byte & (1 << i)) ? '1' : '0';
    }
    out[8] = '\0'; // null-terminate the string
}
 
 void check_status_or_exit() {
     unsigned char status;
     
     latch_command(CMD_READSTATUS);        // Issue READ STATUS command
     latch_register(&status, 1);           // Read one byte from status register
 
     if (status != EXPECTED_STATUS) {
         fprintf(stderr, "Error: NAND status = 0x%02X (expected 0x%02X). Exiting...\n", status, EXPECTED_STATUS);
         exit(EXIT_FAILURE);
     }
 }
 
 
unsigned char hex_to_byte(const char *hex) {
    unsigned char val = 0;
    for (int i = 0; i < 2; i++) {
        char c = toupper(hex[i]);
        val <<= 4;
        if (c >= '0' && c <= '9') val |= (c - '0');
        else if (c >= 'A' && c <= 'F') val |= (c - 'A' + 10);
    }
    // printf("Hex %s to Byte: 0x%02X\n", hex, val);
    return val;
    
}


void read_and_verify_csv(const char *filename, unsigned int *address, int N) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        perror("CSV open failed");
        printf("Failed to open CSV file for writing\n");
        return;
    }
    unsigned int addr[4] = {0,0,0,0};
    char line[512];
    // int current_wl = 2808; // Starting WL address
    //unsigned int addr[4] = {0,0,0,100};
    unsigned char addr_cycles[6];
    addr[0] = address[0];
    addr[1] = address[1];
    addr[2] = address[2];
    addr[3] = address[3];
    int current_wl = address[1]; // Starting WL address
    fprintf(fp, "Stage, XP,UP,LP\n");

    // while (fgets(line, sizeof(line), fp)) {
        for(int stage_loop = 0; stage_loop < log2(N); stage_loop++){
           
        // memset(LP_buffer, lp, sizeof(LP_buffer));
        // memset(UP_buffer, up, sizeof(UP_buffer));
        // memset(XP_buffer, xp, sizeof(XP_buffer));
        // memset(TP_buffer, tp, sizeof(TP_buffer));

        addr[1] = current_wl; // LP
        get_address(addr,addr_cycles);
        read_page_horizontal(addr_cycles , LP_buffer_read, 8*N+2, CMD_LP);

        // addr[1] = current_wl + 1; // UP
        get_address(addr,addr_cycles);
        read_page_horizontal(addr_cycles , UP_buffer_read, 8*N+2, CMD_UP);

        // addr[1] = current_wl + 2; // XP
        get_address(addr,addr_cycles);
        read_page_horizontal(addr_cycles , XP_buffer_read, 8*N+2, CMD_XP);


        for(int page_loop = 2; page_loop < 8*N+2; page_loop++){
        // fprintf(fp, "%d,%d,%02X,%02X,%02X,%02X\n", stage_loop, page_loop, TP_buffer[page_loop],XP_buffer[page_loop],UP_buffer[page_loop],LP_buffer[page_loop]);
        fprintf(fp, "%d,%02X,%02X,%02X\n", stage_loop, XP_buffer_read[page_loop],UP_buffer_read[page_loop],LP_buffer_read[page_loop]);
        // printf("%d,%02X,%02X,%02X,%02X\n", stage_loop+1, TP_buffer_read[page_loop],XP_buffer_read[page_loop],UP_buffer_read[page_loop],LP_buffer_read[page_loop]);
        }
        current_wl -= 0; // move to next WL row
    }

    fclose(fp);
}


void vt_distribution_for_page_PV1(
    unsigned int* addr,
    const char* csv_filename,
    unsigned char* read_offset_template,
    unsigned char* page_type,
    unsigned char* page_data,
    unsigned int page_size,
    unsigned int level)
{
    printf("  Address cycles are: %d %d %d %d\n", addr[0], addr[1], addr[2], addr[3]);
    FILE *fp = fopen(csv_filename, "w");
    if (!fp) {
        fprintf(stderr, "Failed to open %s for writing\n", csv_filename);
        return;
    }
    fprintf(fp, "Offset level,Offset Voltage(mV),#FlippedCellsTo1,#DeltaFlippedCellsTo1,#FlippedCellsTo0,#DeltaFlippedCellsTo0,Abs_DeltaFlippedCellsTo1,Abs_DeltaFlippedCellsTo0,Data\n");

    unsigned char mem_large_block[page_size];
    unsigned char mem_large_block_dummy[page_size];
    unsigned char addr_cycles[6];
    unsigned int original_addr = addr[1];
    unsigned char page_prefix_cmd;
    int set_bits_old = 0;
    int unset_bits_old = 0;

    // Reset counters for positive sweep
    set_bits_old = 0;
    unset_bits_old = 0;

    // Negative voltage sweep
    for (unsigned int loop_var = NV_VT_SWEEP_START; loop_var <= NV_VT_SWEEP_END; loop_var=loop_var+2) {
        unsigned char read_offset[3] = {0x00, 0x00, 0x00};
        printf("Level = %d, Offset Voltage =%.2f\n", level, (256-loop_var)*-10.0);
        printf("  Address cycles are: %d %d %d %d\n", addr[0], addr[1], addr[2], addr[3]);
        memcpy(read_offset, read_offset_template, 3);
        
        // Adjust address based on page type
        switch(*page_type) {
            case 0b001: // LP
                page_prefix_cmd = CMD_LP;
                // addr[1] += 0;
                if(level==1){
                read_offset[0] = (unsigned char)loop_var;
                read_offset[1] = 0x00;
                read_offset[2] = 0x00;
                } 
                if(level==5){
                read_offset[1] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[2] = 0x00;
                } 
                break;
            case 0b010: // UP
                // addr[1] += 1;
                page_prefix_cmd = CMD_UP;
                if(level==2){
                read_offset[0] = (unsigned char)loop_var;
                read_offset[1] = 0x00;
                read_offset[2] = 0x00;
                }
                if(level==4){
                read_offset[1] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[2] = 0x00;
                }
                if(level==6){
                read_offset[2] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[1] = 0x00;
                }
                break;
            case 0b100: // XP
                // addr[1] += 2;
                page_prefix_cmd = CMD_XP;
                if(level==3){
                read_offset[0] = (unsigned char)loop_var;
                read_offset[1] = 0x00;
                read_offset[2] = 0x00;
                }
                if(level==7){
                read_offset[1] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[2] = 0x00;
                }
                break;
        }
        // Set read threshold based on page type
        latch_command(CMD_READ_OFFSET_PREFIX);
        latch_address(read_offset, 3);

        get_address(addr, addr_cycles); // get bit mapping of address
        read_page_horizontal(addr_cycles, mem_large_block, page_size, page_prefix_cmd);
        usleep(10*REALWORLD_DELAY);

        // Reset read threshold
        unsigned char read_offset_reset[3] = {0x00, 0x00, 0x00};
        latch_command(CMD_READ_OFFSET_PREFIX);
        latch_address(read_offset_reset, 3);
        read_page_horizontal(addr_cycles, mem_large_block_dummy, 100, page_prefix_cmd);

        // Restore original address
        switch(*page_type) {
            case 0b001: // LP
                addr[1] -= 0;
                break;
            case 0b010: // UP
                addr[1] -= 0;
                break;
            case 0b100: // XP
                addr[1] -= 0;
                break;
        }


        int set_bits = countSetBits(mem_large_block, page_size);
        int unset_bits = countUnSetBits(mem_large_block, page_size);

        printf("Delta flip abs = %d\n", abs(set_bits - set_bits_old)); // print to terminal for quick inspection
        fprintf(fp, "%d,%.2f,%d,%d,%d,%d,%d,%d,",
            loop_var,
            (256-loop_var) * -10.0,
            set_bits,
            set_bits - set_bits_old,
            unset_bits,
            unset_bits - unset_bits_old,
            abs(set_bits - set_bits_old),
            abs(unset_bits - unset_bits_old)
        );

        // Write raw data
        for(int i = 2; i < page_size; i++) {
            fprintf(fp, "%02x", mem_large_block[i]);
        }
        fprintf(fp, "\n");

        set_bits_old = set_bits;
        unset_bits_old = unset_bits;
    }



    // Positive voltage sweep
    for (unsigned int loop_var = PV_VT_SWEEP_START; loop_var <= PV_VT_SWEEP_END; loop_var=loop_var+2) {
        unsigned char read_offset[3] = {0x00, 0x00, 0x00};
        printf("Level = %d, Offset Voltage =%.2f\n", level, loop_var*10.0);
        printf("  Address cycles are: %d %d %d %d\n", addr[0], addr[1], addr[2], addr[3]);
        memcpy(read_offset, read_offset_template, 3);

        
        // Adjust address based on page type
        switch(*page_type) {
            case 0b001: // LP
                // addr[1] += 0;
                page_prefix_cmd = CMD_LP;
                if(level==1) {
                read_offset[0] = (unsigned char)loop_var;
                read_offset[1] = 0x00;
                read_offset[2] = 0x00;
                } 
                if(level==5) {
                read_offset[1] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[2] = 0x00;
                } 
                break;
            case 0b010: // UP
                // addr[1] += 1;
                page_prefix_cmd = CMD_UP;
                if(level==2) {
                read_offset[0] = (unsigned char)loop_var;
                read_offset[1] = 0x00;
                read_offset[2] = 0x00;
                }
                if(level==4) {
                read_offset[1] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[2] = 0x00;
                }
                if(level==6) {
                read_offset[2] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[1] = 0x00;
                }
                break;
            case 0b100: // XP
                // addr[1] += 2;
                page_prefix_cmd = CMD_XP;
                if(level==3) {
                read_offset[0] = (unsigned char)loop_var;
                read_offset[1] = 0x00;
                read_offset[2] = 0x00;
                }
                if(level==7) {
                read_offset[1] = (unsigned char)loop_var;
                read_offset[0] = 0x00;
                read_offset[2] = 0x00;
                }
                break;
        }
        // Set read threshold based on page type
        latch_command(CMD_READ_OFFSET_PREFIX);
        latch_address(read_offset, 3);
        // Read page
        // printf("  Address cycles are: %d %d %d %d\n", addr[0], addr[1], addr[2], addr[3] );
        // printf("%d", *page_type);
        get_address(addr, addr_cycles);
        read_page_horizontal(addr_cycles, mem_large_block, page_size, page_prefix_cmd);
        usleep(10*REALWORLD_DELAY);

        // Reset read threshold
        unsigned char read_offset_reset[3] = {0x00, 0x00, 0x00};
        latch_command(CMD_READ_OFFSET_PREFIX);
        latch_address(read_offset_reset, 3);
        read_page_horizontal(addr_cycles, mem_large_block_dummy, page_size, page_prefix_cmd);

        // Restore original address
        switch(*page_type) {
            case 0b001: // LP
                addr[1] -= 0;
                break;
            case 0b010: // UP
                addr[1] -= 0;
                break;
            case 0b100: // XP
                addr[1] -= 0;
                break;
        }


        int set_bits = countSetBits(mem_large_block, page_size);
        int unset_bits = countUnSetBits(mem_large_block, page_size);
        printf("Delta flip abs = %d\n", abs(set_bits - set_bits_old)); // print to terminal for quick inspection
        
        fprintf(fp, "%d,%.2f,%d,%d,%d,%d,%d,%d,",
            loop_var,
            loop_var * 10.0,
            set_bits,
            set_bits - set_bits_old,
            unset_bits,
            unset_bits - unset_bits_old,
            abs(set_bits - set_bits_old),
            abs(unset_bits - unset_bits_old)
        );

        // Write raw data
        for(int i = 2; i < page_size; i++) {
            fprintf(fp, "%02x", mem_large_block[i]);
        }
        fprintf(fp, "\n");

        set_bits_old = set_bits;
        unset_bits_old = unset_bits;
    }

    fclose(fp);
}


int main(int argc, char **argv)
{
    struct ftdi_version_info version;
    unsigned char ID_register[256];
    int f;

    // show library version
    version = ftdi_get_library_version();

    // Init 1. channel for databus
    if ((nandflash_iobus = ftdi_new()) == 0)
    {
        return EXIT_FAILURE;
    }

    ftdi_set_interface(nandflash_iobus, INTERFACE_A);
    f = ftdi_usb_open(nandflash_iobus, FT2232H_VID, FT2232H_PID);
    if (f < 0 && f != -5)
    {
        ftdi_free(nandflash_iobus);
        exit(-1);
    }
    ftdi_set_bitmode(nandflash_iobus, IOBUS_BITMASK_WRITE, BITMODE_BITBANG);

    // Init 2. channel for controlbus
    if ((nandflash_controlbus = ftdi_new()) == 0)
    {
        return EXIT_FAILURE;
    }
    ftdi_set_interface(nandflash_controlbus, INTERFACE_B);
    f = ftdi_usb_open(nandflash_controlbus, FT2232H_VID, FT2232H_PID);
    if (f < 0 && f != -5)
    {
        ftdi_free(nandflash_controlbus);
        exit(-1);
    }
    ftdi_set_bitmode(nandflash_controlbus, CONTROLBUS_BITMASK, BITMODE_BITBANG);

    usleep(2 * 1000000);

    controlbus_reset_value();
    controlbus_update_output();

    iobus_set_direction(IOBUS_OUT);
    iobus_reset_value();
    iobus_update_output();

    iobus_set_direction(IOBUS_OUT);
    controlbus_set_direction(CONTROLBUS_OUT);

    // Reset NAND
    latch_command(CMD_RESET);
    usleep(1000000);

    // Check status
    latch_command(CMD_READSTATUS);
    latch_register(ID_register, 1);
    // --------------------------------------------------------------
    //          --------------- Main Logic ---------------
    // --------------------------------------------------------------

    // main function
    // single level test target
    int target_level = 6; // change this to sweep a different level (1-7)
    unsigned int target_WL = 0; // target WL to sweep on

    for (unsigned int target_WL = 10; target_WL < 929; target_WL = target_WL){ // loop over 

        // define variables for page program
        unsigned char page_data[SIZE];
        unsigned char dummy_data1[SIZE];
        unsigned char dummy_data2[SIZE];
        unsigned char dummy_data3[SIZE];
        unsigned char target_page_data1[SIZE];
        unsigned char target_page_data2[SIZE];
        unsigned char target_page_data3[SIZE];

        unsigned char address_cycles1[6]; // dummy WL 927
        unsigned char address_cycles2[6]; // dummy WL 928
        unsigned char address_cycles3[6]; // target WL 929

        unsigned int address1[4] = {0, 0, 0, 0}; // dummy
        unsigned int address2[4] = {0, 0, 0, 0}; // dummy
        unsigned int address3[4] = {0, 0, 0, 0}; // target

        unsigned char target_lp;
        unsigned char target_up;
        unsigned char target_xp;

        uint8_t levels[8][3] = {
    //  { XP, UP, LP} for each level
        {0xff, 0xff, 0xff}, // L0
        {0xff, 0xff, 0x00}, // L1
        {0xff, 0x00, 0x00}, // L2
        {0x00, 0x00, 0x00}, // L3
        {0x00, 0xff, 0x00}, // L4
        {0x00, 0xff, 0xff}, // L5
        {0x00, 0x00, 0xff}, // L6
        {0xff, 0x00, 0xff}  // L7
        };

        // set target page data based on target level
        target_lp = levels[target_level][2];
        target_up = levels[target_level][1];
        target_xp = levels[target_level][0];

        // fill dummy data pages with L6
        get_page_dummy_data(dummy_data1, 0xFF, SIZE); //LP
        get_page_dummy_data(dummy_data2, 0x00, SIZE); //UP
        get_page_dummy_data(dummy_data3, 0x00, SIZE); //XP 

        // fill target page with target level
        get_page_dummy_data(target_page_data1, target_lp, SIZE); //LP
        get_page_dummy_data(target_page_data2, target_up, SIZE); //UP
        get_page_dummy_data(target_page_data3, target_xp, SIZE); //XP 

        // Page addresses
        
        address1[1] = target_WL - 2; // dummy WL
        address2[1] = target_WL - 1; // dummy WL
        address3[1] = target_WL; // target WL

        get_address(address1, address_cycles1);
        get_address(address2, address_cycles2);
        get_address(address3, address_cycles3);

        // Erase block
        erase_block(address_cycles1);
        usleep(10000);
        check_status_or_exit();

        // Program dummy WL 927
        program_page(address_cycles1, dummy_data1, SIZE, CMD_LP);
        usleep(100000);
        program_page(address_cycles1, dummy_data2, SIZE, CMD_UP);
        usleep(100000);
        program_page(address_cycles1, dummy_data3, SIZE, CMD_XP);
        usleep(100000);
        check_status_or_exit();

        // Program dummy WL 928
        program_page(address_cycles2, dummy_data1, SIZE, CMD_LP);
        usleep(100000);
        program_page(address_cycles2, dummy_data2, SIZE, CMD_UP);
        usleep(100000);
        program_page(address_cycles2, dummy_data3, SIZE, CMD_XP);
        usleep(100000);
        check_status_or_exit();

        // Program target WL 929
        program_page(address_cycles3, target_page_data1, SIZE, CMD_LP);
        usleep(100000);
        program_page(address_cycles3, target_page_data2, SIZE, CMD_UP);
        usleep(100000);
        program_page(address_cycles3, target_page_data3, SIZE, CMD_XP);
        usleep(100000);
        check_status_or_exit();

        printf("\nPages programmed successfully\n\nResetting NAND\n\n");

        latch_command(CMD_RESET);
        usleep(100000);

        // --------------- Vth distribution sweep ---------------

        unsigned char pageTypes[] = {
            0b001,  // Level 1 - Lower Page
            0b010,  // Level 2 - Upper Page
            0b100,  // Level 3 - eXtra Page
            0b010,  // Level 4 - Upper Page
            0b001,  // Level 5 - Lower Page
            0b010,  // Level 6 - Upper Page
            0b100   // Level 7 - eXtra Page
        };

        unsigned char read_offset_template[8] = {0};
        char filename[100];
        sprintf(filename, "Page%dL%d_test_232L.csv", address3[1], target_level);
        vt_distribution_for_page_PV1(address3, filename, read_offset_template, &pageTypes[target_level - 1], page_data, 8 * 1024 + 2, target_level);

        latch_command(CMD_RESET);
        usleep(10000);
    }

    // Cleanup
    controlbus_pin_set(PIN_nCE, ON);
    usleep(2 * 1000000);

    ftdi_disable_bitbang(nandflash_iobus);
    ftdi_usb_close(nandflash_iobus);
    ftdi_free(nandflash_iobus);

    ftdi_disable_bitbang(nandflash_controlbus);
    ftdi_usb_close(nandflash_controlbus);
    ftdi_free(nandflash_controlbus);

    return 0;
}