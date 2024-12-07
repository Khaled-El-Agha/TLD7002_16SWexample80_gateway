/**
 *  @file TLD7002_HAL.cpp
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

/*******************************************************************************
**                                  Includes                                  **
*******************************************************************************/
#include "TLD7002_HAL.h" 
#include <SoftwareSerial.h>

 #if defined(ARDUINO_AVR_UNO)
  // Uno pin assignments
    #define SerialHSLI Serial 
#elif defined(ARDUINO_UNOR4_MINIMA)
  // Uno R4 assignments
    #define SerialHSLI Serial1
#else
  #error Unsupported board selection.
#endif

/*******************************************************************************
**                         Global Variable Definitions                        **
*******************************************************************************/

SoftwareSerial softSer=SoftwareSerial(PIN_SOFTSER_RX,  /* soft serial TX pin is used to communicate with TLD, RX is */
                                      PIN_SOFTSER_TX); /* connected to GND, while RX for TLD will be on the HW serial */

/*******************************************************************************
**                         Global Function Definitions                        **
*******************************************************************************/

/**
 * @brief initMCLDSerial: microntroller specific serial init. 
 * use HW serial as RX (read) & use SW serial as TX (write). This is a workaround needed on Arduino, 
 * becasue RX is not possible soft serial at>200kbps and the TLD7002-16 needs minimum 200kbps at the HSLI interface.
 * HW serial remains free for programming and debug printf
 */
void initMCLDSerial()
{
  softSer.begin(HSLI_SPEED_BPS);    /* Soft Serial used as TX for TLD7002 writing only */
 
  pinMode(PIN_SOFTSER_RX, INPUT);
  pinMode(PIN_SOFTSER_TX, OUTPUT);
  digitalWrite (PIN_SOFTSER_TX,1); /* bring softserial pin to high HIGH */
  
  SerialHSLI.begin(HSLI_SPEED_BPS);     /* Hardware serial used as RX for TLD7002 reading only */
}

/** 
* @brief sendMCLDMessage - TLD7002-16 Device Driver mandatory serial TX function. It uses Soft Serial on Arduino
@param* txBuffer - buffer to be transmitted
*/
void sendMCLDMessage(uint8 *txBuffer, uint32 len_write)
{
  while (SerialHSLI.available() > 0){ /* delete data in the (hw) RX buffer until available, they could be present from 
                                    previous operations. Remove RX data by reading and discarding them*/
    SerialHSLI.read();               
  }
  /* SEND SERIAL DATA with the soft serial */
  softSer.write(txBuffer,(uint8)len_write); /* write to TLD7002-16 using soft serial */

}

/** 
* @brief read received data from MCLD and set it to an array
* This functions is the microcontroller specific function to read out the data from TLD7002-16
* serial module and set them available to the TLD7002-16 device drivers.
* @param read_dest: pointer to array of data frame buffer.
* @param len_read: length of received data. [0 ... ASC1_RX_BUFFER_SIZE]
* @return copy of received MCLD answer successful [TRUE: received MCLD answer successful; FALSE: not successful]
*/
boolean  readReceivedMCLDAnswer(uint8* read_dest, uint32 len_read) { /* read on HW serial */
  uint8 ReadBuffer[100];
  boolean copy_successful = FALSE; /*< copy initially not ok */
  int size_rcv_data=0,avl,tmp,cnt;
  int len_write;

  /* READOUT SERIAL DATA (sent + received) readout data to buffer and get size of read buffer */
   cnt = 0;

   //Serial.println("***********serial read =");  // DEBUG print received data, please note htat the Arduino UNO serial buffer is only 64 bytes, and it is not sufficent for READREG or WRITEREG with DLC7, answer will be truncated!
   while (SerialHSLI.available() > 0) {
    /* read the incoming byte: */
    ReadBuffer[cnt++] = SerialHSLI.read();
    //Serial.print(ReadBuffer[cnt-1],HEX );Serial.print(" ");  // DEBUG print received data,
    size_rcv_data++;
   }
   //Serial.println(ReadBuffer[cnt],HEX )// DEBUG print last received data,

  /**
   * ReadBufferCh1 contains sent and received data. Therefore calculate length of written data
   * by subtracting the expected length to read (input parameter) from size of received data.
   * This should be only calculated, if the length of received data is bigger than the length of expected answer 
   */
  if (size_rcv_data > len_read)
     len_write= size_rcv_data - len_read;
  else
     len_write =0;
    
  if(len_write != 0) { /*< if the the length of written data is not 0 (seems to be a valid answer)*/
    for(uint32 index=len_write; index < (len_write + len_read); index++) { /* Go through the elements of the buffer */
      if(index >= len_write) {         /* the first elements include the data written to MCLD. The elements after them  
                                          include the response read from MCLD. */
        *read_dest = ReadBuffer[index];/* copy data */
        read_dest++;                   /* increase the address of the buffer array */
        copy_successful = TRUE;        /* copy data seems to be copy_successful, if this if is called minimum one time*/
      }
    }
  }

return copy_successful;
}

/** @brief This functions empties the serial receive buffer.
*/
void emptyingReceiveBuffer(void)
{
  while (SerialHSLI.available() > 0) {
    SerialHSLI.read();
  }
}

/** @brief start TLD7002 SYNC_BREAK detail implementation
* This functions prepares the AURIX outputs to start the SYNC_BREAK signal on CAN.
*/
void generateStartSyncBreak(void)
{
/* currently nothing */
}

/** @brief stop TLD7002 SYNC_BREAK detail implementation
* This functions prepares the AURIX outputs to stop the SYNC_BREAK signal on CAN.
*/
void generateStopSyncBreak(void)
{
 /* currently nothing */
}

/*
 * @brief Wait for a given amount of microseconds before returning. If the function is using register ticks,
 * make sure to account for register overflow.
 * @param uSeconds - the waiting time, expressed in microseconds maximum is 65535 
 */
void delay_uS(uint16 uSeconds)
{
    delayMicroseconds(uSeconds);
}

void delay_mS(uint32 delay_mS)
{
    delay(delay_mS);
}

 