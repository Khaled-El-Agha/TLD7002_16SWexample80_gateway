/**
 *  @file TLD7002_HAL.h
 *  @author Infineon
 *  @date 06.03.2024
 *	@brief This files is the Hardware Abstraction Layer required by the TLD7002-16 Device Drivers. 
 *        It contains mainly microcontroller specific (Arduino UNO Hardware in this case) serial write /read functions  
 *
 ***********************************************************************************************************************
 *
 * Copyright (c) 2024, Infineon Technologies AG
 * All rights reserved.
 *
 **********************************************************************************************************************/

#ifndef MCLDArduino_H_
#define MCLDArduino_H_

/*******************************************************************************
**                                  Includes                                  **
*******************************************************************************/
#include "types.h"

/*******************************************************************************
**                         Global Macro Declarations                          **
*******************************************************************************/
#define HSLI_SPEED_BPS 230400  /* acceptable down to 20kbps, even if datasheet states 200kbps, when setting very low speed 
                              interframe delay has to be extended (or all bit = 1 will be read as an interframe delay)*/
#define BYTE_TIME_uS        (((8+2)*1000000)/HSLI_SPEED_BPS)

/* PIN defines */
#define PIN_GPIN0A      8
#define PIN_GPIN0B      9
#define PIN_SOFTSER_RX  2
#define PIN_SOFTSER_TX  3

/*******************************************************************************
**                        Global Variable Declarations                        **
*******************************************************************************/


/*******************************************************************************
**                        Global Function Declarations                        **
*******************************************************************************/
#define printHAL Serial.print 

void initMCLDSerial();
void sendMCLDMessage(uint8 *txBuffer, uint32 len_write);
boolean readReceivedMCLDAnswer(uint8* read_dest, uint32 len_read);
void emptyingReceiveBuffer(void);
void generateStopSyncBreak(void);
void generateStartSyncBreak(void);
void delay_uS(uint16 uSeconds);
void delay_mS(uint32 delay_mS);


#endif /* MCLDArduino_H_ */
