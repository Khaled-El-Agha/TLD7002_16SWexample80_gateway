/**
 *  @file TLD7002FuncLayer.cpp
 *  @author Infineon
 *  @date 06.03.2024
 *  @version v1.05
 *	@brief Functional layer, which provide simplified access to the TLD7002-16 device drivers
 *  It combines several calls to the TLD7002-16 device Drivers in order to perform  complex functionalities
 *  without the needs for the user to understand the TLD7002-16 registers bitfields
 *  The Functional Layer simplifies the control of TLD7002-16 devices, in a light driver unit with many light functions
 *   -Simplify channel group access
 *   -Control/Diagnose Light Blocks
 *   -Simplify complex TLD7002 procedures, in example: Device Init / TLD7002-16 OTP write / DTS, VLED, NTC reading
 *   -Simplify HSLI commands (HSLI wrapper)
 *   -Combines Device Drivers TX + delay + RX functions in one API
 **********************************************************************************************************************
 *
 * Copyright (c) 2024, Infineon Technologies AG
 * All rights reserved.
 *
 *********************************************************************************************************************/

// NOTE: uncomment define depending on microcontroller UART implementation
//#define BLOCKING_UART_READ_AND_WRITE 1 // UART RX and TX are blocking: functions will hang till all the requested bytes are transmitted or received (RX timeout needed)
#define BLOCKING_UART_WRITE            1 // UART TX is blocking: function will exit only when all the requested bytes are transmitted (Example Arduino UNO Software serial are blocking when writing )
//#define NON_BLOCKING_UART            1 // UART write HW is non blocking: TX function will exit immediately, RX will exit as soon as there are no more data o be received (Example Arduino Mega 2560 SerialHSLI) not complete

/***********************************************************************************************************************
 *                                                     Includes                                                       **
 **********************************************************************************************************************/
#include "TLD7002_FuncLayer.h"

/***********************************************************************************************************************
 *                                             Private Utility functions                                              **
 **********************************************************************************************************************/
/*int32 max(int32 a, int32 b); already present in Arduino */ 
int8    findInArray_8(uint8 sample, uint8* array, uint32 arraySize);
boolean isPresentInArray_32(uint32 sample, uint32* array, uint32 arraySize);

uint16  TLD7002_getDeviceDiagFromFrameTermination(TLD7002_FRAME_TERMINATION_t frameTermination);
uint16  TLD7002_getChannelDiagFromChannelStatusByteOut(TLD7002_FRAME_CHANNEL_STATUS_BYTE_t statusByte);
char    gBuffPrint[40]; /* buffer for Debug prints */

/***********************************************************************************************************************
 *                                            Global Variable Definitions                                             **
 **********************************************************************************************************************/
/* gBuffTxRx it is a buffer used from the Device driver to store HSLI tx/rx frames, TLD7002_READ_OST_FRAME_t is the Longest
 * HSLI frame this buffer is used instead of multiple specific frame structures requested by the device drivers, (one per
 * frame type) + 5 bytes of padding 
 * Potential Issue: this was tested with HighTec compiler v4.9.4.1. Not tested in other compilers */
uint8 gBuffTxRx [sizeof (TLD7002_READ_OST_FRAME_t) + 5];

/***********************************************************************************************************************
 *                                           Global Function Definitions                                              **
 **********************************************************************************************************************/

/**
 * @brief TLD7002initDrivers initialize TLD7002-16 Device Drivers (but not the TLD7002-16 device itself)
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 */
void TLD7002_initDrivers(TLD7002_NetworkInstance_t* HSLInetwork)
{
    /* initialize serial pin and speed that will be used for TLD7002-16 control */
    initMCLDSerial();

    /* initialize Device Drivers ( not initializing the device TLD7002-16 itself, but only the Device Drivers) by
     * providing HW specific function pointers  */
    TLD7002_InitNetworkInstance(HSLInetwork);
}

/**
 * @brief TLD7002initDevice functions initialize the TLD7002-16ES device and set it to INIT mode
 * it clears also eventual error flags
 * @param TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param addr - HSLI address of the TLD7002 to be initialized
 * @return: it returns true if the TLD7002-16 device is initialized and it is not in Fail Safe mode
 */
boolean TLD7002_initDevice(TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
  uint8   rx_frames_val;
  boolean initSuccess = TRUE;

  HSLInetwork->rcv_empty_buffer(); /* Empty HSLI receiver buffer */

  /* send PM change to INIT twice, to syncronize Device driver master rolling counter and TLD7002 rolling counter
  * this restore the device to INIT and send DC to 0, ready for next command*/
  TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, addr, TLD7002_FRAME_PM_INIT_MODE);
  /* NOTE: ignore if the first HSLI command is not properly received, becasue Master rolling counters is probably wrong initially */
  delay_uS(INTERFR_DLY_MIN_ARDUINO); /* Interframe delay ok for the TLD7002-16SYS_EVAL demoboard, set on the TLD7002-16 OTP */
  TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, addr, TLD7002_FRAME_PM_INIT_MODE);  /* send cmd twice to sync MRC */

  /* NOTE: ignore if the second HSLI command is not properly received, becasue Slave RC could be not aligned yet (if previous command was discarded due to Master RC) */
  /* wait > FAILSAFE2INIT, so if the device it was in fail safe, it can move to init*/
  delay_uS(FAILSAFE2INIT_DELAY);

  /* clear all TLD7002 error flags */
  rx_frames_val = TLD7002_TRX_HWCR_ALL (HSLInetwork, gBuffTxRx,  addr);
  delay_uS(INTERFR_DLY_MIN_ARDUINO); /* interframe delay after any command sent  to avoid next command overlap*/

  /* check of rx frame content to identify, if the step was sucessful */
  if(rx_frames_val != TLD7002_FRAME_VAL_NO_ERROR) {   /* if an error at the device driver receive function was detected */
    initSuccess = FALSE;
    sprintf(gBuffPrint, "*Init Err No response frameVal=%d \r\n", rx_frames_val );
    printHAL(gBuffPrint);
  }
  else { /* if the HSLI response frame was properly received, check its content */
    if(((TLD7002_HWCR_FRAME_t*)gBuffTxRx)->r_hardware_control.frame_termination.OST.OSB_FAULT) {   /* if an internal fault occured */
      initSuccess = FALSE;
      printHAL("*Init Err-FAULT\r\n");
    }
    else if(((TLD7002_HWCR_FRAME_t*)gBuffTxRx)->r_hardware_control.frame_termination.ACK.TER) {   /* if an transmission error on TLD7002-16ES side occured */
      initSuccess = FALSE;
      printHAL("*Init Err-TER\r\n");
    }
    else if( ((TLD7002_HWCR_FRAME_t*)gBuffTxRx)->r_hardware_control.frame_termination.ACK.MODE == TLD7002_FRAME_ACK_BYTE_MODE_2 ) {   /* if the device is not in init mode, Active Mode (due to a GPIN in HIGH state for instance).*/
      initSuccess = FALSE;
      sprintf(gBuffPrint, "*Init Err-MODE= %d \r\n", ((TLD7002_HWCR_FRAME_t*)gBuffTxRx)->r_hardware_control.frame_termination.ACK.MODE );
      printHAL(gBuffPrint);
    }
  }
  return initSuccess;
}

/*
 * @brief OTP_Write: OTP write PROCEDURE, it permanently write the OTP on the TLD7002-16, and then checks if the OTP is
 * written correctly. The sequence of STEPS and all the delay between steps are as specified in the
 * Infineon-TLD7002-16ES_OTP_programming_procedure-AN-v01_10-EN application note
 * NOTE: OTP can be written only once in a TLD7002-16 device, if OTP write fails, the device is permanently ruined
 * NOTE: test OTP_hex_cfg with emulation in advance
 * PREREQUISITE: apply VS in the following range 16V < VS < 20V
 * PREREQUISITE: TLD7002-16 GPIN0 has to be connected to uC pin named PIN_GPIN0
 * @global gBuffTxRx - buffer used from the Device driver to store HSLI tx/rx frames
 * @param otpCfg - TLD7002-16 OTP configuration array, 40 bytes
 * @param TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param addr - address of the TLD7002-16 after OTP write, this has to match to the one set in the otp_cgf array
 * @param newInterfrDelay - TLD7002-16 interframe delay set on the otpCfg array, this has to match to the one set in the
 *                          otpCfg array
 * @return True if OTP write was successful, False otherwise
  */
uint16 TLD7002_OTPwrite(const uint16 *otpCfg, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr, uint16 newInterfrDelay, uint16 gpin0Pin)
{
    uint8 deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;  /* used to store transmission HSLI replies */
    uint16 errorFlags = NO_ERR;/* return value for the TLD7002_OTPwrite function */

    digitalWrite(gpin0Pin,0); /* ensure GPIN0 is LOW */

    /*  Step1: ensure the device is in INIT mode to enter into OTP mode, before the OTP mode request */
    TLD7002_TX_PM_CHANGE_FRAME(HSLInetwork, (TLD7002_PM_CHANGE_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(1005); /* delay counted starting from the last bit transmitted */

    /* Step2: send command twice to syncronize LLD library master rolling counter and TLD7002 rolling counter */
    TLD7002_TX_PM_CHANGE_FRAME(HSLInetwork, (TLD7002_PM_CHANGE_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(1005);

    /* Step3: set interframe delay to 100us(INTERFR_DLY_MIN_ARDUINO) by  writing the volatile register TLD7002_HSLI_TIMING_CFG(ADD 0x3B) with 000C */
    TLD7002_TX_WRITE_REG_DLC1_FRAME(HSLInetwork, (TLD7002_WRITE_REG_DLC1_FRAME_t*)gBuffTxRx, BROADCAST_ADDRESS, 0x3B, 0x001C);
    delay_uS(INTERFR_DLY_MIN_ARDUINO); /* delay counted starting from the last bit transmitted */

    /* Step4: set the TLD7002-16 in OTP mode */
    TLD7002_TX_PM_CHANGE_FRAME(HSLInetwork, (TLD7002_PM_CHANGE_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_OTP_MODE);
    delay_uS(1005); /* delay counted starting from the last bit transmitted */

    /* Step5: set GPIN0 HIGH to enable programming */
    digitalWrite (gpin0Pin,1); /*Set_GPIN0_to_high_value();*/
    delay_uS(50);

    /* Step6: write OTP Programming passw at address 0x81 */
    TLD7002_TX_WRITE_REG_DLC1_FRAME(HSLInetwork, (TLD7002_WRITE_REG_DLC1_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, 0x81, OTP_WRITE_PASSW);
    delay_uS(INTERFR_DLY_MIN_ARDUINO); /* delay counted starting from the last bit transmitted */

    /* Step7: write first 32 words */
    TLD7002_TX_WRITE_REG_DLC7_FRAME(HSLInetwork, (TLD7002_WRITE_REG_DLC7_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, OTP_START_ADDR, (uint16 *)otpCfg);
    delay_mS(18); /* wait 17,5ms, OTP write is slow, delay counted starting from the last bit transmitted */

    /* Step8: Write last 8 words */
    TLD7002_TX_WRITE_REG_DLC4_FRAME(HSLInetwork, (TLD7002_WRITE_REG_DLC4_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, 0xA3, (uint16 *)&otpCfg[DLC7_LENGTH]);
    delay_mS(5); /* wait 5ms, OTP write is slow */

    /* Step9: The GPIN0 voltage of the device under programming can be set to low after programming*/
    digitalWrite (gpin0Pin, 0); /* Set GPIN0 to low_value() according to OTP app note procedure; */
    delay_uS(INTERFR_DLY_MIN_ARDUINO);

    /* Step 10: The device is moved to INIT mode.If the OTP writing procedure ends successfully then the device is
     configured by the new OTP array.*/
    TLD7002_TX_PM_CHANGE_FRAME(HSLInetwork, (TLD7002_PM_CHANGE_FRAME_t*)gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(max(newInterfrDelay, 205)); /* delay counted starting from the last bit transmitted, delay shall be:
                                                   delay > 200us or bigger than the just configured interframe delay */

    /* Step 11: Send HWCR frame to delete the FAULT produced at startup by the unwritten device*/
    TLD7002_TRX_HWCR_ALL (HSLInetwork, gBuffTxRx,  addr); /* First TRX command with device addr!= broadcast, result
                                         is probably bad becasue roll counter is not sync with Device drivers library */
    delay_uS(newInterfrDelay); /* wait > new programmed interframe delay */

    /* Step 12  read OTP_STATUS register */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_OTP_STATUS);
    delay_uS(newInterfrDelay);       /* wait > new programmed interframe delay */

    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)  /* if received frame is valid, check the status byte */
    {
        if(((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] == 0x03) /* OTP_STATUS as expected for correct OTP write */
        {
            printHAL("OTP wr:OTP_STAT OK=0x03\r\n");
        }
        else
        {
            /*printf("OTP wr:err OTP_STAT=0x%04X\r\n" ,((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0]);*/
            errorFlags |= OPER_ERR;/* OTP write failed, status is not as exrequired by the OTP write app note */
        }
    }
    else
    {
        /*printf("OTP wr:Err frame1 res=%d\r\n", deviceResponse); */ 
        errorFlags |= COMM_DRIVER_ERR; /* OTP write possibly failed becasue thedevice is not responding correctly */
    }

    /* Step13: set the TLD7002-16 in OTP mode */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx,addr, TLD7002_FRAME_PM_OTP_MODE);
    if (deviceResponse != TLD7002_FRAME_VAL_NO_ERROR)   /* received frame is valied */
    {
        /*printf("OTP wr:Err frame2 res=%d\r\n", deviceResponse);*/
        errorFlags |= COMM_DRIVER_ERR; /* OTP write possibly failed becasue thedevice is not responding correctly */
    }

    /* the interframe shall be counted starting from the last bit transmitted from the TLD7002-16 (slave response) */
    delay_uS(newInterfrDelay);

    /* STEP14 and Step 15: read back the entire OTP, simplified with CMD read 5 times 8 word from the OTP */
    for (uint8 n = 0; n < (OTP_USER_SIZE/DLC4_LENGTH) ; n++)
    {
        deviceResponse = TLD7002_TRX_READ_REG_DLC4(HSLInetwork, gBuffTxRx, addr, OTP_START_ADDR + n * DLC4_LENGTH);
        delay_uS (newInterfrDelay); /* wait interframe delay after TLD7002-16 answer */

        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            /* lenght of DLC4 is (TLD7002_LEN_WRITE_REG_DLC4_WRITE-TLD7002_LEN_WRITE_REG_OVHD)/2 = 8 16-bit words*/
            for (uint8 i = 0; i < DLC4_LENGTH; i++)
            {
                if (((TLD7002_READ_REG_DLC4_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i]!= otpCfg[i + n * DLC4_LENGTH]) /* compare readed OTP */
                {                                                                                                  /* values with the ones written OTP cfg */
                    /* printHAL("OTP wr:Err readback Mismatch\r\n");*/
                    errorFlags |= OPER_ERR;/* OTP write failed, status is not as exrequired by the OTP write app note */
                    /* Exit loop */
                    break;
                }
            }
        }
        else
        {
            /*printf("OTP wr:Err readBack frame err res=%d\r\n", deviceResponse);*/
            errorFlags |= COMM_DRIVER_ERR; /* OTP write possibly failed becasue thedevice is not responding correctly */
            /* Exit loop */
            break;
        }
    }

    /* STEP 16 check output status fault BIT */
    delay_uS(2 * PWM_PERIOD_uS); /* wait for 2 PWM periods from the last HWCR command, CRC is checked 1 per PWM
                              period, this is the worst case time needed to highlight CRC errors on the OUTPUT STATUS */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_OTP_STATUS); /* dummy read, just to have the
                                                                                        OUTPUT STATUS byte reply back */
    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        if (((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.frame_termination.OST.OSB_FAULT != 0)
        { /* check if the OUTPUT STATUS byte FAULT bit is 0 */
            errorFlags |= DEVICE_FAULT_ERR; /* OTP write possibly failed becasue thedevice is reports internal Fault bit */
        }
    }
    else
    {
        errorFlags |= COMM_DRIVER_ERR; /* OTP write possibly failed becasue thedevice is not responding correctly */
    }

    return errorFlags;
}

/**
 * OTP read PROCEDURE
 * @brief TLD7002_OTPread - TLD7002-16 OTP read PROCEDURE, The sequence and all the delay between steps are as specified in the
 * Infineon-TLD7002-16ES_OTP_programming_procedure-AN-v01_10-EN application note
 * @param otpCfg[in/out] - pointer to TLD7002-16 OTP configuration array size 44 ( user defined 40 bytes + 4 read only bytes). Will be filled with OTP config from device
 * @param gBuffTxRx - buffer used from the Device driver to store HSLI tx/rx frames
 * @param TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @return True if OTP emulation was successful, False otherwise
 */
uint16 TLD7002_OTPread(uint16 *otpCfg, TLD7002_NetworkInstance_t* HSLInetwork, uint8 deVaddr)
{
    uint8 deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;  /* used to store transmission HSLI replies */
    uint16 errorFlags = NO_ERR; /* return value for the TLD7002_OTPread function */

    /*  Step1: ensure the device is in INIT mode to enter into OTP mode, before the OTP mode request */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, deVaddr, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(INTERFR_DLY_MAX_TLD7002); /* delay counted starting from the last bit transmitted */

    /* case Step2: Sent twice the INIT mode request to sync MRC and RC of the Device Driver and TLD7002-16 device */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, deVaddr, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(7000); /* If the device/devices under programming never reaches/reach the failsafe state the delay*/
                           /* can be reduced to 1005us, delay counted starting from the last bit transmitted          */

/* Step2bis(optional, needed for Arduino UNO which has slow soft serial): set interframe delay to 100us by  writing the volatile register TLD7002_HSLI_TIMING_CFG(ADD 0x3B) with 000C*/
    TLD7002_TRX_WRITE_REG_DLC1(HSLInetwork, gBuffTxRx,BROADCAST_ADDRESS, 0x3B, 0x001C);
    delay_uS(INTERFR_DLY_MIN_ARDUINO); /* delay counted starting from the last bit transmitted */

    /* Step3: set the TLD7002-16 in OTP mode */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx,deVaddr, TLD7002_FRAME_PM_OTP_MODE);
    delay_uS(1005); /* delay counted starting from the last bit transmitted */


    /* Step4 (part 1): Read back the entire user OTP array */
    /* entire CFC configurationis 40 words, read 5 times 8 word from the OTP (arduino has 62 bytes limited serial buffer, can not read the enire DLC7 frame) */
    for (uint8 n = 0; n < (OTP_USER_SIZE/DLC4_LENGTH); n++)
    {
        deviceResponse = TLD7002_TRX_READ_REG_DLC4(HSLInetwork, gBuffTxRx, deVaddr, OTP_START_ADDR + n * DLC4_LENGTH);
        /*wait interfframe delay after TLD7002-16 answer*/
        delay_uS (INTERFR_DLY_MIN_ARDUINO);

        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            for (uint8 i = 0; i < DLC4_LENGTH; i++)
            {
                otpCfg[i + n * DLC4_LENGTH] = ((TLD7002_READ_REG_DLC4_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i]; /* write 8 bytes of the OTP at the time in to the parameter otpCfg */
            }
        }
        else
        {
            //printHAL("OTP read ERR devRes= %d\r\n",deviceResponse);
            errorFlags |= COMM_DRIVER_ERR; /* OTP read failed becasue thedevice is not responding correctly */
            /* breaks for loop ( performing OTP reading), if frame errors are present during readback*/
            break;
        }
    }

 
    /* Step4 (part 2): Read back the four read only OTP registers */
    deviceResponse = TLD7002_TRX_READ_REG_DLC3(HSLInetwork, gBuffTxRx, deVaddr, OTP_READ_ONLY_START_ADDR);
    /*wait interframe delay after TLD7002-16 answer*/
    delay_uS (INTERFR_DLY_MIN_ARDUINO);

    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        for (uint8 i = 0; i < OTP_READ_ONLY_SIZE; i++)
        {
            otpCfg[OTP_USER_SIZE+i] = ((TLD7002_READ_REG_DLC3_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i]; /* write 8 bytes of the OTP at the time in to the parameter otpCfg */
        }
    }
    else
    {
        sprintf(gBuffPrint, "**OTP read DLC3 ERR devRes=0x%04X \n", deviceResponse );
        printHAL(gBuffPrint);
        errorFlags |= COMM_DRIVER_ERR; /* OTP read failed becasue thedevice is not responding correctly */
    }

    /* Step 6: go back to INIT mode */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, deVaddr, TLD7002_FRAME_PM_INIT_MODE);
    delay_mS(21); /* 20,27ms is the longest wait time for this step, used as fixed value, without having to calculate the
                   maximum between tINIT (250µs + 2(1/fPWM) and the new interframe delay */

    /* return true if OTP write was successfully performed */
    return errorFlags;
}

/**
 * OTP EMULATION PROCEDURE: new checked 24/01/23
 * @brief TLD7002_OTPemulate - TLD7002-16 OTP emulation PROCEDURE, it emulates the OTP on the TLD7002-16, and then checks if
 * the OTP is emulated correctly. The sequence and all the delay between steps are as specified in the
 * Infineon-TLD7002-16ES_OTP_programming_procedure-AN-v01_10-EN application note
 * NOTE: TLD7002-16 emulation is only possible if GPIN0 was not already configured as analog pin in a previous OTP
 * emulation or programming
 * PREREQUISITE: TLD7002-16 GPIN0 has to be connected to uC pin named PIN_GPIN0
 * @param otpCfg - TLD7002-16 OTP configuration array, 40 bytes
 * @param gBuffTxRx - buffer used from the Device driver to store HSLI tx/rx frames
 * @param TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param addrAfterEmu - device addr after emulation (has to match the address passed on the OTP configuration file,
          which is emulated)
 * @param newInterfrDelay - TLD7002 interframe delay set on the otpCfg array, this has to match to the one set in the
          otpCfg array
 * @return True if OTP emulation was successful, False otherwise
 */
uint16 TLD7002_OTPemulate(const uint16 *otpCfg, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addrAfterEmu, uint16 newInterfrDelay, uint16 gpin0Pin)
{
    uint8 deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;  /* used to store transmission HSLI replies */
    uint16 errorFlags = NO_ERR;/* return value for the TLD7002_OTPread function */

    digitalWrite (gpin0Pin,0); /* bring GPIN0 is LOW */
    delay_uS(1005);

    /*  Step1: ensure the device is in INIT mode to enter into OTP mode, before the OTP mode request */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(1005); /* delay counted starting from the last bit transmitted */

    /* case Step2: Sent twice the INIT mode request to sync MRC and RC of the Device Driver and TLD7002-16 device */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(7000); /* If the device/devices under programming never reaches/reach the failsafe state the delay*/
                           /* can be reduced to 1005us, delay counted starting from the last bit transmitted          */

    /* Step3: set interframe delay to 100us by  writing the volatile register TLD7002_HSLI_TIMING_CFG(ADD 0x3B) with 000C*/
    TLD7002_TRX_WRITE_REG_DLC1(HSLInetwork, gBuffTxRx,BROADCAST_ADDRESS, 0x3B, 0x001C);
    delay_uS(INTERFR_DLY_MIN_ARDUINO); /* delay counted starting from the last bit transmitted */

    /* Step4: set the TLD7002-16 in OTP mode */
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx,BROADCAST_ADDRESS, TLD7002_FRAME_PM_OTP_MODE);
    delay_uS(1005); /* delay counted starting from the last bit transmitted */

    /* Step5: set GPIN0 HIGH to enable programming */
    digitalWrite (gpin0Pin, 1);
    delay_uS(INTERFR_DLY_MIN_ARDUINO);

    /* Step6 : write OTP mode emulation password OTP emu*/
    TLD7002_TRX_WRITE_REG_DLC1(HSLInetwork, gBuffTxRx, (uint8)BROADCAST_ADDRESS, TLD7002_OTP_EMULATION, EMULATION_PASSW);
    delay_uS(INTERFR_DLY_MIN_ARDUINO); /* interframe delay */

    /*  step 7 : write the first 32 words of the OTP */
    TLD7002_TRX_WRITE_REG_DLC7(HSLInetwork, gBuffTxRx, BROADCAST_ADDRESS , OTP_START_ADDR, (uint16 *)otpCfg);
    delay_uS(INTERFR_DLY_MIN_ARDUINO); /* delay counted starting from the last bit transmitted */

    /*  step 8 : write the last 8 words of the OTP so the new slave address is already sent to the device */
    TLD7002_TRX_WRITE_REG_DLC4(HSLInetwork, gBuffTxRx, BROADCAST_ADDRESS , 0xA3, (uint16 *)&otpCfg[DLC7_LENGTH]);
    delay_uS(newInterfrDelay);  /* wait for interframe delay */

    /* Step9: The GPIN0 voltage of the device under programming can be set to low after programming*/
    digitalWrite (gpin0Pin,0); /*  Set GPIN0 to a logic low */
    delay_uS(INTERFR_DLY_MIN_ARDUINO);

    /* Step10: clear all TLD7002 error especially the FAULT due to VIRGIN OTP */
    TLD7002_TRX_HWCR_ALL (HSLInetwork, gBuffTxRx,  addrAfterEmu); /* first command with new device address(not broadcast, will syncronize also Dev Driv slave rolling counter) */
    delay_uS(newInterfrDelay);

    /* Step11 and STEP 12 combined: Read back and compare the entire OTP array */
    /* entire CFC configurationis 40 words, read 5 times 8 word from the OTP (arduino has 62 bytes limited serial buffer, can not read the enire DLC7 frame) */
    for (uint8 n = 0; n < OTP_USER_SIZE/DLC4_LENGTH; n++)
    {
        deviceResponse = TLD7002_TRX_READ_REG_DLC4(HSLInetwork, gBuffTxRx, addrAfterEmu, OTP_START_ADDR + n * DLC4_LENGTH);
        delay_uS (newInterfrDelay);/*wait interfframe delay after TLD7002-16 answer*/

        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            /* lenght of DLC4 is (TLD7002_LEN_WRITE_REG_DLC4_WRITE-TLD7002_LEN_WRITE_REG_OVHD)/2 = 8 16-bit words */
            for (uint8 i = 0; i < DLC4_LENGTH; i++)
            {
                if (((TLD7002_READ_REG_DLC4_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i]!= otpCfg[i + n * DLC4_LENGTH])
                {
                    /*printf("**OTP Mismatch err ** \r\n");*/
                    /*printf("Rec: %04X, exp: %04X\r\n", ((TLD7002_READ_REG_DLC4_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i], otpCfg[i + n * DLC4_LENGTH]);*/
                    errorFlags |= OPER_ERR; /* emulation operation was not successfull */
                    /* Exit loop */
                    break;
                }
            }
        }
        else
        {
            /* printf("read ERR res= %d\r\n",deviceResponse); */
            errorFlags |= COMM_DRIVER_ERR; /* emulation operation was probably not successfull becasue the device did not reply, so success is unknown */
            /* Exit loop */
            break;
        }
    }

    /* Step 13: The device is moved to INIT mode.If the OTP writing procedure ends successfully then the device is
    * configured by the new OTP array.*/
    TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, addrAfterEmu, TLD7002_FRAME_PM_INIT_MODE);
    delay_mS(21); /* 20,27ms is the longest wait time for this step, used as fixed value, without having to calculate the
                     maximum between tINIT (250Âµs + 2(1/fPWM) and the new interframe delay */

    /* Step 13: send again INIT mode just to retrieve the INTERNAL FAULT byte */
    deviceResponse = TLD7002_TRX_PM_CHANGE(HSLInetwork, gBuffTxRx, addrAfterEmu, TLD7002_FRAME_PM_INIT_MODE);
    delay_uS(newInterfrDelay);  /* wait for interframe delay JUST IN CASE AFTER THE PROCEDURE A COMMAND IS RECEIVED IN THE hsli */

    /* if the device replied correctly and there are no internal faults, or previous OTPsuccess issues (E.G. readback mismatch) then the emulation is successful */
    if ( deviceResponse == TLD7002_FRAME_VAL_NO_ERROR )
    {
        if (((TLD7002_PM_CHANGE_FRAME_t*)gBuffTxRx)->r_power_mode_change.frame_termination.OST.OSB_FAULT != 0 )
        {
            errorFlags |= DEVICE_FAULT_ERR; /* report device internal fault */
            /*printHAL( "int fault\r\n"); */ /* probably due to OTP CRC error, see TLD7002-16 datasheet for all the possible sources */
        }
    }
    else
    {
        errorFlags |= COMM_DRIVER_ERR; /* report device internal fault */
    }

    return errorFlags;
}

/**
 * @brief TLD7002_readNTC_GPIN1: reads TLD7002-16 external temperature on NTC connected to GPIO1
 * this is valid for a 10k NTC part number: Mitsubishi TD11-3H103F 0603 10K 3370K TD11-3H103F 0603 10K Ohm 3370K
 * NTC is placed as low side resistor divider with Upper resistor 10k to the 5V supply (see TLD7002-16SHIELD schematic)
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *tempNTC - pointer to the variable that will contain the temperature reading [°C]
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_readNTC_GPIN1(float *tempNTC, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16  ADCvalue = 0;
    uint32  GPINmV = 0;
    float   GPINv = 0;
    uint8   deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;/*used to store transmission HSLI replies*/
    uint16  errorFlags = NO_ERR;

    /* read GPIO1 voltage, becasue the NTC is applied to GPI=1 with a voltage divider  */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_ADC_VGPIO1); /* send read register
                                                                                       request to read VS ADC voltage */
    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* check the VALID (ready )bitfield */
        if (((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VGPIO1_VAL_F_MSK)
        {
            ADCvalue = ((TLD7002_READ_REG_DLC1_FRAME_t*) gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VGPIO1_VGPIO1_OUT_MSK;
            GPINmV = ((uint32) ADCvalue * ADC_GPIN_FULL_SCALE_mV)/ADC_RESOLUTION_LSB;   /* calculate ADC value in mV */
            GPINv = (float) GPINmV / 1000; /* From mV to V */
            *tempNTC = (1.8928 * GPINv * GPINv * GPINv * GPINv) - (23.013 * GPINv * GPINv * GPINv)+
                     (95.833 * GPINv * GPINv)- 181.98 * (GPINv) + 169.94 ; /* fourth order aproximation formula for temp calc
                                                            = 1.8928x4 - 23.013x3 + 95.833x2 - 181.98x + 169.94 */
        }
        else
        {
            errorFlags = ADC_ERR; /* ADC not valid or device in INIT  mode */
        }
    }
    else
    {
        errorFlags = COMM_DRIVER_ERR;
    }

    return errorFlags;
}

/**
 * @brief TLD7002readGPIO0: reads TLD7002-16 ADC on GPIO0
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *GPINmV - pointer to the variable that will contain the GPIN0 voltage [mV]
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_readGPIN0(uint16 *GPINmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16  ADCvalue;
    uint8   deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;/*used to store transmission HSLI replies*/
    uint16  errorFlags = NO_ERR;

    /* read GPIN0 voltage, becasue the NTC is applied to GPI=1 with a voltage divider  */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_ADC_VGPIO0); /* send read register
                                                                                       request to read VS ADC voltage */
    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* check the VALID (ready )bitfield */
        if (((TLD7002_READ_REG_DLC1_FRAME_t*) gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VGPIO0_VAL_F_MSK)
        {
            ADCvalue = ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VGPIO0_VGPIO0_OUT_MSK;
            *GPINmV = ((uint32) ADCvalue * ADC_GPIN_FULL_SCALE_mV) / ADC_RESOLUTION_LSB;   /* calculate ADC value in mV */
        }
        else
        {
            errorFlags = ADC_ERR; /* ADC not valid or device in INIT  mode */
        }
    }
    else
    {
        /* Serial.print("Frame Error Res=");Serial.println(res);*/
        errorFlags = COMM_DRIVER_ERR;
    }
  return errorFlags;
}

/**
 * @brief TLD7002readGPIO1: reads TLD7002-16 ADC on GPIO1
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *GPINmV - pointer to the variable that will contain the GPIN0 voltage [mV]
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_readGPIN1(uint16 *GPINmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16  ADCvalue;
    uint8   deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;/*used to store transmission HSLI replies*/
    uint16  errorFlags = NO_ERR;

    /* read GPIN0 voltage, becasue the NTC is applied to GPI=1 with a voltage divider  */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_ADC_VGPIO1); /* send read register
                                                                                       request to read VS ADC voltage */
    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* check the VALID (ready )bitfield */
        if (((TLD7002_READ_REG_DLC1_FRAME_t*) gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VGPIO1_VAL_F_MSK)
        {
            ADCvalue = ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VGPIO1_VGPIO1_OUT_MSK;
            *GPINmV = ((uint32) ADCvalue * ADC_GPIN_FULL_SCALE_mV) / ADC_RESOLUTION_LSB;   /* calculate ADC value in mV */
        }
        else
        {
            errorFlags = ADC_ERR; /* ADC not valid or device in INIT  mode */
        }
    }
    else
    {
        /* Serial.print("Frame Error Res=");Serial.println(res);*/
        errorFlags = COMM_DRIVER_ERR;
    }

  return errorFlags;
}

/**
 * @brief TLD7002readDTS: reads TLD7002-16 internal temp (DTS Die Temp Sensor) 
 * and print it in the serial monitor in °C. It reads also if the TLD7002-16 thermal derating is active
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *DTStemp - pointer to the variable that will contain the Die temperature readings [°C]
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode 
 */
uint16 TLD7002_readDTS( uint16 *DTStemp,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16 DTSvalue = 0;
    uint8  deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;/* used to store transmission HSLI replies */
    uint16 errorFlags = NO_ERR;

    /* send read register request to read Die Temp Sensor(internal Temp) */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_DTS_STAT);

    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* check the VALID (ready )bitfield */
        if (((TLD7002_READ_REG_DLC1_FRAME_t*) gBuffTxRx)->r_read_reg.Data[0] & TLD7002_DTS_STAT_DTS_TEMP_READY_MSK)
        {
            DTSvalue = (((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_DTS_STAT_DTS_TEMP_MSK)
                                                                                       >>TLD7002_DTS_STAT_DTS_TEMP_POS;
            *DTStemp = (((uint32)DTSvalue*1000) / 1139 ) - 273; /* DTS_TEMP formula on TLD7002-16 user manual */
        }
        else
        {
            /* ADC not valid or device in INIT mode */
            errorFlags |= ADC_ERR;
        }
    }
    else
    {
        /* Frame Error */
        errorFlags |= COMM_DRIVER_ERR;
    }

    return errorFlags;
}

/**
 * @brief TLD7002readVFWD: reads TLD7002-16 VFWD on output "ch" in mV and print it's value on the serial monitor
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *VFWDmV - pointer to the variable that will contain the VFWD voltage in [mV]
 * @param[in] addr - address of the TLD7002 device
 * @param[in] channel - channel where the VFWD voltage will be measured
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_readVFWD(uint16 *VFWDmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr, uint8 channel)
{
    uint16   ADCvalue = 0;
    uint32   ADCmV = 0;
    uint8    deviceResponse = TLD7002_FRAME_VAL_NO_ERROR; /*used to store transmission HSLI replies */
    uint16   errorFlags = NO_ERR;
    uint8    startAdd = 0;

    if (channel < TLD7002_CHANNELS)
    {
        startAdd = TLD7002_LD_ADC_VFWD0 + channel;

        /*** READ VLED ***/
        /* send read register request to read VS ADC voltage */
        deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, startAdd);

        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            /* check the TLD7002_LD_ADC_VFWDi_VFWD_MSK VALID bitfield */
            if (((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VFWDi_VAL_F_MSK)
            {
                ADCvalue = ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VFWDi_VFWD_MSK;
                ADCmV = ((uint32)ADCvalue * ADC_VOUT_FULL_SCALE_mV)/ADC_RESOLUTION_LSB; /* VS ADC to mV  conversion formula */
                *VFWDmV = ADCmV;
            }
            else
            {
                /*ADC not valid or device in INIT mode */
                errorFlags |= ADC_ERR;
                *VFWDmV = 0 ; /* clear anyway VLED */
            }
        }
        else
        {
            /* Frame Error */
            errorFlags = COMM_DRIVER_ERR;
            *VFWDmV = 0 ; /* clear anyway VLED */
        }
    }
    else
    {
        /* not valid channel provided */
        errorFlags |= OPER_ERR;
    }

    return errorFlags;
}
  
/**
 * @brief TLD7002readVLED: reads TLD7002-16 VLED voltage in mV and print it's value on the serial monitor
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *VLEDmV - pointer to the variable that will contain the VLED voltage in [mV]
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_readVLED(uint16 *VLEDmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16  ADCvalue = 0;
    uint32  ADCmV = 0;
    uint8   deviceResponse = TLD7002_FRAME_VAL_NO_ERROR; /*used to store transmission HSLI replies */
    uint16  errorFlags = NO_ERR;

    /* send read register to read VLED ADC voltage */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_ADC_VLED);

    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* check the VALID bitfield */
        if ( ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VLED_VAL_F_MSK)
        {
            ADCvalue = ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VS_VLED_MSK;
            ADCmV = ((uint32)ADCvalue * ADC_VLED_FULL_SCALE_mV)/ADC_RESOLUTION_LSB; /* VS ADC to mV  conversion formula */
            *VLEDmV = ADCmV ;
        }
        else
        {
            /*ADC not valid or device in INIT mode */
            errorFlags |= ADC_ERR;
            *VLEDmV = 0 ; /* clear anyway VLED */
        }
    }
    else
    {
        /* Frame Error */
        errorFlags = COMM_DRIVER_ERR;
        *VLEDmV = 0 ; /* clear anyway VLED */
    }

    return errorFlags;
}

/**
 * @brief TLD7002readVLED: reads TLD7002-16 VS voltage in mV and print it's value on the serial monitor
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in-out] *VSmV - pointer to the variable that will contain the VS voltage reading [mV]
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_readVS(uint16 *VSmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16  ADCvalue = 0;
    uint32  ADCmV = 0;
    uint8   deviceResponse = TLD7002_FRAME_VAL_NO_ERROR; /*used to store transmission HSLI replies */
    uint16  errorFlags = NO_ERR;

    /*** READ VLED ***/
    /* send read register request to read VS ADC voltage */
    deviceResponse = TLD7002_TRX_READ_REG_DLC1(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_ADC_VS);

    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* check the VALID bitfield */
        if ( ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VS_VAL_F_MSK)
        {
            ADCvalue = ((TLD7002_READ_REG_DLC1_FRAME_t*)gBuffTxRx)->r_read_reg.Data[0] & TLD7002_LD_ADC_VS_VLED_MSK;
            ADCmV = ((uint32)ADCvalue * ADC_VS_FULL_SCALE_mV)/ADC_RESOLUTION_LSB; /* VS ADC to mV  conversion formula */
            *VSmV = ADCmV ;
        }
        else
        {
            /*ADC not valid or device in INIT mode */
            errorFlags |= ADC_ERR;
            *VSmV = 0 ; /* clear anyway VLED */
        }
    }
    else
    {
        /* Frame Error */
        errorFlags = COMM_DRIVER_ERR;
        *VSmV = 0 ; /* clear anyway VLED */
    }
    return errorFlags;
}

/**
 * @brief TLD7002_setCurrAll: set the output current to the requested value in all channels. NOTE: CUR_WRN threshold is set in the OTP , 
 * therefore spurious CRU_WRN will arise by changing the output current with ths funciton
 * @param[in] currUA - current in micro Amp [uA]
 * @param[in] TLD7002_NetworkInstance_t* - Reference to TLD7002 network instance
 * @param[in] addr - address of the TLD7002 device
 * @return: error on the conversion or in the device operating mode
 */
uint16 TLD7002_setCurrAll(uint32 currUA,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr)
{
    uint16  DACval = 1; /* DEBUG: use formula form DS current configuration register (  LD_PWM_DAC_CFG.DAC_CONFIGi) value*/
    uint8   deviceResponse = TLD7002_FRAME_VAL_NO_ERROR; /*used to store transmission HSLI replies */
    uint16  errorFlags = NO_ERR;

    // printf("curr req is %d\n",currUA);

    if ( currUA< TLD7002MIN_CUR_UA ) /* check if requested current is below the minimum TLD7002-16 output curr. value*/
      currUA =  TLD7002MIN_CUR_UA;
    else if (currUA >  TLD7002MAX_CUR_UA)/* check if requested current is above the max TLD7002-16 output curr. value*/
            currUA = TLD7002MAX_CUR_UA;

    DACval = (currUA - TLD7002MIN_CUR_UA)/TLD7002CUR_STEP_UA;  /* formula from TLD7002-16 user Manual: DAC_CONFIGi = (IOUT - 5,625mA)/1,25mA  */
    
    /*** READ LD_PWM_DAC_CFGi registerw (it contains the actual current plus other bitfields) ***/
    deviceResponse = TLD7002_TRX_READ_REG_DLC6(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_PWM_DAC_CFG0);/* DLC 6 is 32 bytes => 16 words*/
  
    if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
    {
        /* lenght of DLC4 is (TLD7002_LEN_WRITE_REG_DLC4_WRITE-TLD7002_LEN_WRITE_REG_OVHD)/2 = 8 16-bit words */
        for (uint8 i = 0; i < DLC6_LENGTH; i++)
        {
        /* delete LD_PWM_DAC_CFGi field  inside the TLD7002_LD_PWM_DAC_CFGi register*/
        (((TLD7002_READ_REG_DLC6_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i]) &= (uint16)(~TLD7002_LD_PWM_DAC_CFGi_DAC_CONFIG_MSK);
        /* update LD_PWM_DAC_CFGi field  inside the TLD7002_LD_PWM_DAC_CFGi register*/
        (((TLD7002_READ_REG_DLC6_FRAME_t*)gBuffTxRx)->r_read_reg.Data[i]) |= (uint16)(((uint16)DACval << TLD7002_LD_PWM_DAC_CFGi_DAC_CONFIG_POS) & TLD7002_LD_PWM_DAC_CFGi_DAC_CONFIG_MSK);
        }
        deviceResponse = TLD7002_TRX_WRITE_REG_DLC6(HSLInetwork, gBuffTxRx, addr, TLD7002_LD_PWM_DAC_CFG0,((TLD7002_READ_REG_DLC6_FRAME_t*)gBuffTxRx)->r_read_reg.Data);/* DLC 6 is 32 bytes => 16 words*/
        if (deviceResponse != TLD7002_FRAME_VAL_NO_ERROR)        
          errorFlags = COMM_DRIVER_ERR;
    }
     else
    {
        /* Frame Error */
        errorFlags = COMM_DRIVER_ERR;
     }
 
    return errorFlags;
}

/*-------------------------------------- Device Driver Wrappers TX,RX combined ----------------------------------------*
 * Wrapper functions which combines TX of the TLD7002 frame and RX of the TLD7002 acknowledge answer the wrappers func
 * are casting one single gBuffTxRx[] in to each possible frame type needed by the device driver TX or RX function
 * NOTE:
 * UART TX could be implemented as a blocking function, therefore the TRX functions are delay-waiting for the TLD7002 answer,
 * before calling the RX device driver function. In case of HW UART implementation, where TX is not blocking code execution while
 * transmitting on the UART, the delay has to count also for transmitted bytes.
 */

/* TX-RX wrapper, see TLD7002_TX_DC_UPDATE_8BIT_FRAME + TLD7002_RX_DC_UPDATE_8BIT_FRAME description */
uint8 TLD7002_TRX_DC_UPDATE_8BIT(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint16* dcVal)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_DC_UPDATE_8BIT_FRAME(mcldNet, (TLD7002_DC_UPDATE_8BIT_FRAME_t*)buffTxRx, addr, dcVal); /* update shadow registers
                                                          with new duty cycles (output still not change until DC_SYNC) */
  #ifdef BLOCKING_UART_READ
  delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
  delay_uS(TLD7002_LEN_DC_UPDATE_8BIT_READ * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_DC_UPDATE_8BIT_WRITE + TLD7002_LEN_DC_UPDATE_8BIT_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
    rx_frames_val = TLD7002_RX_DC_UPDATE_8BIT_FRAME(mcldNet, (TLD7002_DC_UPDATE_8BIT_FRAME_t*)buffTxRx);/* receive TLD7002 answer */
  }
  return rx_frames_val;
}
 
/* TX-RX wrapper, see TLD7002_TX_DC_UPDATE_14BIT_FRAME + TLD7002_RX_DC_UPDATE_14BIT_FRAME description */
uint8 TLD7002_TRX_DC_UPDATE_14BIT(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint16* dcVal)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_DC_UPDATE_14BIT_FRAME(mcldNet, (TLD7002_DC_UPDATE_14BIT_FRAME_t*)buffTxRx, addr, dcVal);/* send a DC_UPDATE
                                      to move to active mode,  providing also new duty cycle array to shadow registers */
  // debug test
  #ifdef BLOCKING_UART_READ_AND_WRITE
    delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the requested bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
    delay_uS((TLD7002_LEN_DC_UPDATE_14BIT_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#  elif NON_BLOCKING_UART
       delay_uS((TLD7002_LEN_DC_UPDATE_14BIT_WRITE+TLD7002_LEN_DC_UPDATE_14BIT_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#endif
  
  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_DC_UPDATE_14BIT_FRAME(mcldNet, (TLD7002_DC_UPDATE_14BIT_FRAME_t*)buffTxRx);/* receive TLD7002 answer */
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_READ_REG_DLC1 + TLD7002_RX_READ_REG_DLC1_FRAME description */
uint8 TLD7002_TRX_READ_REG_DLC1(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_READ_REG_DLC1_FRAME(mcldNet, (TLD7002_READ_REG_DLC1_FRAME_t*)buffTxRx, devAddr, startAdd);/*TX a read DLC1 frame*/
	#ifdef BLOCKING_UART_READ
	delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
	# elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
	delay_uS((TLD7002_LEN_READ_REG_DLC1_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_READ_REG_WRITE +TLD7002_LEN_READ_REG_DLC1_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_READ_REG_DLC1_FRAME(mcldNet, (TLD7002_READ_REG_DLC1_FRAME_t*)buffTxRx);/*RX a read DLC1 frame*/ 
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_READ_REG_DLC3 + TLD7002_RX_READ_REG_DLC3_FRAME description */
uint8 TLD7002_TRX_READ_REG_DLC3(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_READ_REG_DLC3_FRAME(mcldNet, (TLD7002_READ_REG_DLC3_FRAME_t*)buffTxRx, devAddr, startAdd);/*TX a read DLC3 frame*/
	#ifdef BLOCKING_UART_READ
	delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
	# elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
	delay_uS((TLD7002_LEN_READ_REG_DLC3_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_READ_REG_WRITE +TLD7002_LEN_READ_REG_DLC3_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_READ_REG_DLC3_FRAME(mcldNet, (TLD7002_READ_REG_DLC3_FRAME_t*)buffTxRx);/*RX a read DLC3 frame*/ 
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_READ_REG_DLC4 + TLD7002_RX_READ_REG_DLC4_FRAME description (DLC4 = 8 words) */
uint8 TLD7002_TRX_READ_REG_DLC4(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_READ_REG_DLC4_FRAME(mcldNet, (TLD7002_READ_REG_DLC4_FRAME_t*)buffTxRx, devAddr, startAdd);/*TX a read DLC4 frame*/

  #ifdef BLOCKING_UART_READ
  delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
  delay_uS((TLD7002_LEN_READ_REG_DLC4_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_READ_REG_WRITE +TLD7002_LEN_READ_REG_DLC4_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */  
  #endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_READ_REG_DLC4_FRAME(mcldNet, (TLD7002_READ_REG_DLC4_FRAME_t*)buffTxRx);/*RX a read DLC4 frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_READ_REG_DLC6 + TLD7002_RX_READ_REG_DLC6_FRAME description */
uint8 TLD7002_TRX_READ_REG_DLC6(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_READ_REG_DLC6_FRAME(mcldNet, (TLD7002_READ_REG_DLC6_FRAME_t*)buffTxRx, devAddr, startAdd);/*TX a read DLC6 frame*/

  #ifdef BLOCKING_UART_READ
  delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
  delay_uS((TLD7002_LEN_READ_REG_DLC6_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_READ_REG_WRITE +TLD7002_LEN_READ_REG_DLC6_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */  
  #endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_READ_REG_DLC6_FRAME(mcldNet, (TLD7002_READ_REG_DLC6_FRAME_t*)buffTxRx);/*RX a read DLC6 frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_READ_REG_DLC0 + TLD7002_RX_READ_REG_DLC0_FRAME description */
uint8 TLD7002_TRX_DC_UPDATE_DLC0(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_DC_UPDATE_8BIT_DLC0_FRAME(mcldNet, (TLD7002_DC_UPDATE_8BIT_FRAME_t*)buffTxRx, addr);/* send a DC_UPDATE DLC0
                                                                                               to move to active mode*/
	#ifdef BLOCKING_UART_READ
	delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
	# elif BLOCKING_UART_WRITE /* only the UART write is blocking untill all bytes are transmitted, but then wait for the reply to be entirely received  */
	  delay_uS((TLD7002_LEN_DC_UPDATE_8BIT_DLC0_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_DC_UPDATE_8BIT_DLC0_WRITE+TLD7002_LEN_DC_UPDATE_8BIT_DLC0_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */  
	#endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_DC_UPDATE_8BIT_DLC0_FRAME(mcldNet, (TLD7002_DC_UPDATE_8BIT_FRAME_t*)buffTxRx);/*RX a read DLC0 frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_HWCR_ALL_FRAME + TLD7002_RX_HWCR_FRAME description */
uint8 TLD7002_TRX_HWCR_ALL (TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_HWCR_ALL_FRAME(mcldNet, (TLD7002_HWCR_FRAME_t*)buffTxRx,addr);

	#ifdef BLOCKING_UART_READ
	delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
	# elif BLOCKING_UART_WRITE
	delay_uS((TLD7002_LEN_HWCR_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_HWCR_WRITE+TLD7002_LEN_HWCR_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_HWCR_FRAME(mcldNet, (TLD7002_HWCR_FRAME_t*)buffTxRx);/*RX a read HWCR response frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_PM_CHANGE_FRAME + TLD7002_RX_PM_CHANGE_FRAME description */
uint8 TLD7002_TRX_PM_CHANGE(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, TLD7002_FRAME_POWER_MODE_CHANGE_t mode)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_PM_CHANGE_FRAME(mcldNet,(TLD7002_PM_CHANGE_FRAME_t*)buffTxRx, addr, mode);

	#ifdef BLOCKING_UART_READ
	delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
	# elif BLOCKING_UART_WRITE
	delay_uS((TLD7002_LEN_PM_CHANGE_READ) * BYTE_TIME_uS ); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_PM_CHANGE_WRITE+TLD7002_LEN_PM_CHANGE_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#endif
  
  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_PM_CHANGE_FRAME(mcldNet, (TLD7002_PM_CHANGE_FRAME_t*) buffTxRx); /*RX a read PM_CHANGE response frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_WRITE_REG_DLC7 + TLD7002_RX_WRITE_REG_DLC7_FRAME description */
uint8 TLD7002_TRX_WRITE_REG_DLC7(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd, uint16* data)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_WRITE_REG_DLC7_FRAME(mcldNet, (TLD7002_WRITE_REG_DLC7_FRAME_t*)buffTxRx, devAddr, startAdd, data);
	#ifdef BLOCKING_UART_READ
	delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
	# elif BLOCKING_UART_WRITE
	delay_uS((TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_WRITE_REG_DLC7_WRITE+TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#endif
  
  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_WRITE_REG_DLC7_FRAME(mcldNet, (TLD7002_WRITE_REG_DLC7_FRAME_t*)buffTxRx); /*RX a write DLC7 response frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_WRITE_REG_DLC6 + TLD7002_RX_WRITE_REG_DLC6_FRAME description */
uint8 TLD7002_TRX_WRITE_REG_DLC6(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd, uint16* data)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_WRITE_REG_DLC6_FRAME(mcldNet, (TLD7002_WRITE_REG_DLC6_FRAME_t*)buffTxRx, devAddr, startAdd, data);

  #ifdef BLOCKING_UART_READ
  delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE
  delay_uS((TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_WRITE_REG_DLC6_WRITE+TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #endif
  
  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_WRITE_REG_DLC6_FRAME(mcldNet,(TLD7002_WRITE_REG_DLC6_FRAME_t*)buffTxRx); /*RX a write DLC4 response frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_WRITE_REG_DLC4 + TLD7002_RX_WRITE_REG_DLC4_FRAME description */
uint8 TLD7002_TRX_WRITE_REG_DLC4(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd, uint16* data)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_WRITE_REG_DLC4_FRAME(mcldNet, (TLD7002_WRITE_REG_DLC4_FRAME_t*)buffTxRx, devAddr, startAdd, data);

  #ifdef BLOCKING_UART_READ_AND_WRITE
  delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE
  delay_uS((TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_WRITE_REG_DLC4_WRITE+TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #endif
  
  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_WRITE_REG_DLC4_FRAME(mcldNet,(TLD7002_WRITE_REG_DLC4_FRAME_t*)buffTxRx); /*RX a write DLC4 response frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_WRITE_REG_DLC1 + TLD7002_RX_WRITE_REG_DLC1_FRAME description */
uint8 TLD7002_TRX_WRITE_REG_DLC1(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 devAddr, uint8 startAdd, uint16 data)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_WRITE_REG_DLC1_FRAME(mcldNet, (TLD7002_WRITE_REG_DLC1_FRAME_t*)buffTxRx, devAddr, startAdd, data);
  #ifdef BLOCKING_UART_READ_AND_WRITE
  delay_uS(BYTE_TIME_uS ); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE
  delay_uS((TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #  elif NON_BLOCKING_UART
     delay_uS((TLD7002_LEN_WRITE_REG_WRITE+TLD7002_LEN_WRITE_REG_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_WRITE_REG_DLC1_FRAME(mcldNet, (TLD7002_WRITE_REG_DLC1_FRAME_t*)buffTxRx); /*RX a write DLC1 response frame*/
  }
  return rx_frames_val;
}

/* TX-RX wrapper, see TLD7002_TX_READ_OST_FRAME + TLD7002_RX_READ_OST_FRAME description */
uint8 TLD7002_TRX_READ_OST(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr)
{
  boolean tx_frame_successful; /* stores the HSLI transmission result */
  uint8   rx_frames_val = TLD7002_FRAME_VAL_UNDEFINED_ERROR;/* stores the HSLI receive result, define as UNDEF_ERR in case TX is failing*/
  
  tx_frame_successful = TLD7002_TX_READ_OST_FRAME(mcldNet, (TLD7002_READ_OST_FRAME_t*)buffTxRx,  addr);

  #ifdef BLOCKING_UART_READ_AND_WRITE
    delay_uS(BYTE_TIME_uS); /* wait just for 1 byte to be received, then an HW blocking function will hang till all the requested bytes are received (RX timeout needed)*/
  # elif BLOCKING_UART_WRITE
  delay_uS((TLD7002_LEN_READ_OST_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
	#  elif NON_BLOCKING_UART
      delay_uS((TLD7002_LEN_READ_OST_WRITE+TLD7002_LEN_READ_OST_READ) * BYTE_TIME_uS); /* wait for TLD7002 complete reply */
  #endif

  if(tx_frame_successful) {    /* if frame was transmitted successful */
     rx_frames_val = TLD7002_RX_READ_OST_FRAME(mcldNet, (TLD7002_READ_OST_FRAME_t*)buffTxRx); /*RX a read OST response frame*/
  }
  return rx_frames_val;
}

/**
 * @brief TLD7002_TX_BROADCAST_DC_SYNC: activate duty cycles which were stored in the shadow registers with TLD7002_TRX_DC_UPDATE
 * command. It sends a DC_SYNC_FRAME on the HSLI bus. Since the DC_SYNC it is a broadcast command, there will be no reply on it
 * from the TLD7002-16 (because if multiple TLD7002-16 are connected to the bus, they would all reply)
 */
void TLD7002_TX_BROADCAST_DC_SYNC(TLD7002_NetworkInstance_t* HSLInetwork, uint8* buffTxRx)
{
  TLD7002_TX_BRDC_DC_SYNC_FRAME(HSLInetwork, (TLD7002_BRDC_DC_SYNC_FRAME_t*)buffTxRx ); /* send DC sync to bring  */
                                                        /* duty cycles stored in the shadow register at the output */
  return;
}

/*----------------------------------------- lightSystem & LightFunc API ------------------------------------------*/

/*
 * @brief TLD7002_initLightSystem: this function initialize a lightSystem struct. 
 * it also initializes all the TLD7002-16 devices in the system
 * @param system - pointer to the light system to initialize
 * @return initSuccess - it returns true if all the TLD7002-16 devices on the system has not properly initialized
 */
boolean TLD7002_initLightSystem(lightSystem_s *system)
{
    boolean initSuccess = true;

    for (uint8 dev = 0; dev < system->addressCount; dev++)
    {
        system->deviceDiag[dev] = NO_ERR;
        initSuccess &=TLD7002_initDevice(system->mcldNet, system->addressList[dev]);

        /* Initialize system matrix with 0% duty and NO_ERR diag */
        for (uint8 ch = 0; ch < TLD7002_CHANNELS; ch++)
        {
            system->pixelMatrix[dev][ch].diag = NO_ERR;
            system->pixelMatrix[dev][ch].duty = 0x00;
        }
    }

    return initSuccess;
}

/*
 * @brief TLD7002_setLocalDutyLightFunc: this function set all the pixels of a light function to the same duty cycle.
 * @param func - pointer to the LightFunc instance
 * @param duty - value of duty cycle between 0 and 10000
 * @return TRUE if the duty was set, FALSE otherwise
 */
boolean TLD7002_setLocalDutyLightFunc(lightFunc_s *func, uint16 duty)
{
    uint8   dev = 0;
    uint8   ch  = 0;
    int8    devIndex = 0;
    boolean opSuccess = TRUE;

    for (uint8 pixel = 0; pixel < func->numOfPixels; pixel++)
    {
        dev = func->pixelList[pixel].device;
        ch  = func->pixelList[pixel].channel;

        /* Given the device address dev, find its index in the addressList of the lightSystem */
        devIndex = findInArray_8(dev, (uint8 *) func->system->addressList, func->system->addressCount);

        if (devIndex >= 0)
        {
            func->system->pixelMatrix[devIndex][ch].duty = duty;
        }
        else
        {
            /* Device address was not found in the lightSystem's addressList */
            opSuccess = FALSE;
        }
    }

    return opSuccess;
}


/*
 * @brief TLD7002_setLocalDutyPixel: this function set one specific pixel of a light function to a given duty cycle.
 * @param func - pointer to the LightFunc instance
 * @param index - the index of the pixel in the light function pixel list
 * @param duty - value of duty cycle between 0 and 10000
 * @return TRUE if the duty was set correctly in the light system matrix, returns FALSE if the index is not valid
 */
boolean TLD7002_setLocalDutyPixel(lightFunc_s *func, uint8 index, uint16 duty)
{
    uint8 dev = 0;
    uint8 ch  = 0;
    boolean opSuccess = TRUE;

    /* If index is invalid, opSuccess is FALSE */
    if (index >= func->numOfPixels)
    {
        opSuccess = FALSE;
    }
    else
    {
        dev = func->pixelList[index].device;
        ch  = func->pixelList[index].channel;

        /* Given the device address dev, find its index in the addressList of the lightSystem */
        int8 devIndex = findInArray_8(dev, (uint8 *) func->system->addressList, func->system->addressCount);

        if (devIndex >= 0)
        {
            func->system->pixelMatrix[devIndex][ch].duty = duty;
        }
        else
        {
            /* Device address was not found in the lightSystem's addressList */
            opSuccess = FALSE;
        }
    }

    return opSuccess;
}


/*
 * @brief TLD7002_getLocalDiagPixel: this function get the cumulative diagnostic of every pixel of a light function.
 * @param func - pointer to the LightFunc instance
 * @return a 16-bit diagnostic value
 */
uint16 TLD7002_getLocalDiagLightFunc(lightFunc_s *func)
{

    uint16 errorFlags = NO_ERR;
    uint8 dev = 0;
    uint8 ch  = 0;

    for (uint8 pixel = 0; pixel < func->numOfPixels; pixel++)
    {
        dev = func->pixelList[pixel].device;
        ch  = func->pixelList[pixel].channel;

        /* Given the device address dev, find its index in the addressList of the lightSystem */
        int8 devIndex = findInArray_8(dev, (uint8 *) func->system->addressList, func->system->addressCount);

        if (devIndex >= 0)
        {
            errorFlags |= func->system->pixelMatrix[devIndex][ch].diag;
            errorFlags |= func->system->deviceDiag[devIndex];
        }
        else
        {
            /* Device address was not found in the lightSystem's addressList */
            errorFlags |= OPER_ERR;
        }
    }

    return errorFlags;
}


/*
 * @brief TLD7002_getLocalDiagPixel: this function get the diagnostic of one specific pixel of a light function.
 * @param func - pointer to the LightFunc instance
 * @param index - the index of the pixel in the light function pixel list
 * @return a 16-bit diagnostic value
 */
uint16 TLD7002_getLocalDiagPixel(lightFunc_s *func, uint8 index)
{

    uint16 errorFlags = NO_ERR;
    uint8 dev = 0;
    uint8 ch  = 0;

    /* if index is valid */
    if (index < func->numOfPixels)
    {
        dev = func->pixelList[index].device;
        ch  = func->pixelList[index].channel;

        /* Given the device address dev, find its index in the addressList of the lightSystem */
        int8 devIndex = findInArray_8(dev,(uint8 *) func->system->addressList, func->system->addressCount);

        if (devIndex >= 0)
        {
            errorFlags = func->system->pixelMatrix[devIndex][ch].diag;
            errorFlags |= func->system->deviceDiag[devIndex];
        }
        else
        {
            /* Device address was not found in the lightSystem's addressList */
            errorFlags = OPER_ERR;
        }
    }
    else
    {
        /* Invalid pixel index */
        errorFlags = OPER_ERR;
    }

    return errorFlags;
}


/*
 * @brief TLD7002_getLocalDeviceDiag: this function get the diagnostic of one specific device in the light system.
 * @param system - pointer to the lightSystem instance
 * @param addr - the address of the device
 * @return a 16-bit diagnostic value of the status of the device with address addr
 */
uint16 TLD7002_getLocalDeviceDiag(lightSystem_s *system, uint8 addr)
{

    uint16 errorFlags = NO_ERR;

    /* Given the device address, find its index in the addressList of the lightSystem */
    int8 devIndex = findInArray_8(addr,(uint8 *) system->addressList, system->addressCount);

    if (devIndex >= 0)
    {
        errorFlags = system->deviceDiag[devIndex];

    }
    else
    {
        /* Device address was not found in the lightSystem's addressList */
        errorFlags |= OPER_ERR;
    }

    return errorFlags;
}


/*
 * @brief TLD7002_txDutyLightSystem: this function sends the DC update of all the pixels of a given lightSystem to the TLD7002 devices.
 * Then it broadcast the DC sync.
 * @param system - pointer to the lightSystem instance
 * @param buffTxRx - buffer used from the Device driver to store HSLI tx/rx frames
 */
void TLD7002_txDutyLightSystem(lightSystem_s *system, uint8 *buffTxRx)
{

    uint16 dutyArray[TLD7002_CHANNELS]; /* Array containing the duty cicles that will be sent to the devices */
    uint8 deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;  /* used to store transmission HSLI replies */

    /* For each device address in the light system */
    for (uint8 dev = 0; dev < system->addressCount; dev++)
    {

        /* Fill duty array: every row of the pixelMatrix contains the channels of a TLD7002 device */
        for (uint8 ch = 0; ch < TLD7002_CHANNELS; ch++)
        {
            dutyArray[ch] = system->pixelMatrix[dev][ch].duty;
        }

        /* Provides new duty cycle array to shadow registers */
        deviceResponse = TLD7002_TRX_DC_UPDATE_14BIT(system->mcldNet,  buffTxRx, system->addressList[dev], dutyArray);

        /* wait for interframe delay before next HSLI BUS communication to TLD7002-16 */
        delay_uS(INTERFR_DLY_MIN_ARDUINO);

        /* Get device diagnostic from frame termination */
        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            system->deviceDiag[dev] = TLD7002_getDeviceDiagFromFrameTermination(((TLD7002_DC_UPDATE_14BIT_FRAME_t *) buffTxRx)->r_pwm_dc_update.frame_termination);
        }
        else
        {
            system->deviceDiag[dev] = COMM_DRIVER_ERR;
        }

        /* Clean duty array */
        for (uint8 ch = 0; ch < TLD7002_CHANNELS; ch++)
        {
            dutyArray[ch] = 0;
        }
    }

    /* Perform Broadcast DC Sync */
    TLD7002_TX_BRDC_DC_SYNC_FRAME(system->mcldNet, (TLD7002_BRDC_DC_SYNC_FRAME_t*) buffTxRx);

}

/*
 * @brief TLD7002_rxDiagLightSystem: this function sends the READ OST command for every TLD7002 device of a given lightSystem.
 * Then interprets the device and channel diagnostics and save it in the light system structure (on both pixelMatrix[dev][ch].diag and deviceDiag)
 * @param system - pointer to the lightSystem instance
 * @param buffTxRx - buffer used from the Device driver to store HSLI tx/rx frames
 */
void TLD7002_rxDiagLightSystem(lightSystem_s *system, uint8 *buffTxRx)
{
    uint8 deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;  /* used to store transmission HSLI replies */

    /* For each device address in the light system */
    for (uint8 dev = 0; dev < system->addressCount; dev++)
    {

        deviceResponse = TLD7002_TRX_READ_OST (system->mcldNet, buffTxRx, system->addressList[dev]);

        delay_uS(INTERFR_DLY_MIN_ARDUINO);             /* wait for interframe delay before next HSLI BUS communication to TLD7002-16    */

        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            /* Get device diag from frame termination if FRAME_VAL_NO_ERR */
            system->deviceDiag[dev] = TLD7002_getDeviceDiagFromFrameTermination(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.frame_termination);

            for (uint8 ch = 0; ch < TLD7002_CHANNELS; ch++)
            {

                switch (ch)
                {
                    case 0:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT0);
                        break;
                    case 1:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT1);
                        break;
                    case 2:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT2);
                        break;
                    case 3:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT3);
                        break;
                    case 4:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT4);
                        break;
                    case 5:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT5);
                        break;
                    case 6:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT6);
                        break;
                    case 7:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT7);
                        break;
                    case 8:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT8);
                        break;
                    case 9:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT9);
                        break;
                    case 10:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT10);
                        break;
                    case 11:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT11);
                        break;
                    case 12:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT12);
                        break;
                    case 13:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT13);
                        break;
                    case 14:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT14);
                        break;
                    case 15:
                        system->pixelMatrix[dev][ch].diag =  TLD7002_getChannelDiagFromChannelStatusByteOut(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.CSB_OUT15);
                        break;
                    default:
                        system->pixelMatrix[dev][ch].diag =  NO_ERR;
                        break;
                }
            }
        }

        else
        {
            system->deviceDiag[dev] = COMM_DRIVER_ERR;
        }
    }
}


/*
 * @brief TLD7002_rxDiagLightSystem: this function sends the HWCR commadn to clear all the light system TLD7002-16 devices warning flags 
 * in case the devices are not responding update the LighSystem deviceDiag
 * @param system - pointer to the lightSystem instance
 * @param buffTxRx - buffer used from the Device driver to store HSLI tx/rx frames
 */
void TLD7002_txClrDiagLightSystem(lightSystem_s *system, uint8 *buffTxRx)
{
    uint8 deviceResponse = TLD7002_FRAME_VAL_NO_ERROR;  /* used to store transmission HSLI replies */

    /* For each device address in the light system */
    for (uint8 dev = 0; dev < system->addressCount; dev++)
    {

        /* clear all TLD7002 error flags */
        deviceResponse = TLD7002_TRX_HWCR_ALL (system->mcldNet, buffTxRx, system->addressList[dev] );

        /* wait for interframe delay before next HSLI BUS communication to TLD7002-16 */
        delay_uS(INTERFR_DLY_MIN_ARDUINO); /* do not remove this delay, it is inside the for loop for each device */

        if (deviceResponse == TLD7002_FRAME_VAL_NO_ERROR)
        {
            /* Get device diag from frame termination if FRAME_VAL_NO_ERR */
            system->deviceDiag[dev] = TLD7002_getDeviceDiagFromFrameTermination(((TLD7002_READ_OST_FRAME_t*)buffTxRx)->r_read_output_status.frame_termination);
        }
        else
        {
            system->deviceDiag[dev] = COMM_DRIVER_ERR;
        }
    }
}

/*
 * @brief TLD7002_printAddressDiag: this function prints a specific address device diagnostic + channel cumulative diag
 * prerequisite: TLD7002_rxDiagLightSystem has to be called in advance to fill the light system diag matrix
 * @param statusByte - the output status byte
 */
void TLD7002_printAddressDiag(lightSystem_s *system, uint8 *buffTxRx, uint8 addr)
{
   uint16 diag = NO_ERR;
   int8 devIndex = 0;

   /* Given the device address, find its index in the addressList of the lightSystem */
  devIndex = findInArray_8(addr,(uint8 *) system->addressList, system->addressCount);

  if (devIndex >= 0)
  {
      printHAL("* Device Diag(ADDR: 0x)");
      printHAL(addr);
      printHAL("\n");

      /* Get device diag */
      diag = system->deviceDiag[devIndex];

      /* Get cumulative channel diag */
      for (uint8 ch = 0; ch < TLD7002_CHANNELS; ch++)
      {
          diag |= system->pixelMatrix[devIndex][ch].diag;
      }

      TLD7002_printDiag(diag);
  }
}

/*
 * @brief TLD7002_printDiag: this function prints a verbose version of the "Error masks define"
 * Example usage:
 * turnDiag = TLD7002_getLocalDiagLightFunc(&gTurnFunc); 
 * TLD7002_printDiag(turnDiag);  
 * @param diag - the 16-bit diagnostic
 */
void TLD7002_printDiag(uint16 diag)
{
  if ((diag & OPER_ERR) != NO_ERR)
  {
      printHAL("OPERATION ERR \n"); /*some operation was not successful*/
  }

  /* Print device diag */
  if ((diag & (DEVICE_ERR | COMM_ERR)) != NO_ERR) /* check if device diagnostic is clean */
  {
    printHAL("DEVICE ERR= ");
    if (diag & DEVICE_UV_ERR)
    {
        printHAL("UNDERV- ");
    }
    if (diag & DEVICE_FAULT_ERR)
    {
        printHAL("INTER FAULT- ");
    }
    if (diag & DEVICE_MODE_ERR)
    {
        printHAL("MODE ERR- ");
    }
    if ((diag & COMM_T_ERR) | (diag & COMM_DRIVER_ERR))
    {
        printHAL("COM ERR- ");
    }
    printHAL("\n");
  }

  if ((diag & CHANNEL_ERR) != NO_ERR) /* check if channel diagnostic is not clean*/
  {
    printHAL("CH ERR= ");
    if (diag & CH_VFWD_WRN)
    {
        printHAL("VFWD- ");
    }
    if (diag & CH_CUR_WRN)
    {
        printHAL("OUT CURR WRN- ");
    }
    if (diag & CH_DC_WRN)
    {
        printHAL("DC WRN- ");
    }
    if (diag & CH_OUT_SHORT_WRN)
    {
        printHAL("OUT SHORT WRN- ");
    }
    if (diag & CH_OVLD_ERR)
    {
        printHAL("OVERLOAD- ");
    }
    if (diag & CH_SLS_ERR)
    {
        printHAL("SLS- ");
    }
    if (diag & CH_OL_ERR)
    {
        printHAL("OL- ");
    }
  }
  printHAL("\n");
}

/***********************************************************************************************************************
 *                                             Private Utility functions                                              **
 **********************************************************************************************************************/

/*
 * @brief isPresentInArray_8: This function verify if a given uint8 sample is present in a uint8 array and returns the
 * position of its first occurrence.
 * @param sample - the sample
 * @param array - the array that will be searched
 * @param arraySize - the size of the array
 * @return The index of the first occurrence of the sample in the array, if sample is present in the array, -1 otherwise
 */
int8 findInArray_8(uint8 sample, uint8* array, uint32 arraySize)
{
  int8 returnIndex = -1;

  for (uint32 index = 0; index < arraySize; index++)
  {
      if (sample == array[index])
      {
          returnIndex = index;
          /* Exit loop */
          break;
      }
  }
    return returnIndex;
}

/*
 * @brief isPresentInArray_32: This function verify if a given uint32 sample is present in a uint8 array.
 * @param sample - the sample
 * @param array - the array that will be searched
 * @param arraySize - the size of the array
 * @return TRUE if sample is present in the array, FALSE otherwise
 */
boolean isPresentInArray_32(uint32 sample, uint32* array, uint32 arraySize)
{
    boolean found = FALSE;

    for (uint32 index = 0; index < arraySize; index++)
    {
        if (sample == array[index])
        {
            found = TRUE;
        }
    }

    return found;
}

/*
 * @brief TLD7002_getDeviceDiagFromFrameTermination: this function interprets a frame termination and returns a
 * a 16-bit diagnostic value
 * @param frameTermination - the frame termination
 * @returns a 16-bit diagnostic value
 */
uint16 TLD7002_getDeviceDiagFromFrameTermination(TLD7002_FRAME_TERMINATION_t frameTermination)
{
    uint16 errorFlags = NO_ERR;

    /* Check for internal fault */
    if (frameTermination.OST.OSB_FAULT)
    {
        errorFlags |= DEVICE_FAULT_ERR;
    }

    /* Check if the device is in ACTIVE Mode or EMULATION mode (not acceptable to be in INIT)*/
    if (frameTermination.ACK.MODE == TLD7002_FRAME_ACK_BYTE_MODE_1 || frameTermination.ACK.MODE == TLD7002_FRAME_ACK_BYTE_MODE_3 )
    {
        if (frameTermination.OST.VLED_VS_UV)
        {
            errorFlags |= DEVICE_UV_ERR;
        }
        if (frameTermination.ACK.TER == TRUE)
        {
            errorFlags |= COMM_T_ERR;
        }
    }
    else
    {
        /*  device not in active or Emulation, it not makes sense to interpret the outputs diagnostic bytes */
        errorFlags |= DEVICE_MODE_ERR;
    }

    return errorFlags;
}

/*
 * @brief TLD7002_getChannelDiagFromChannelStatusByteOut: this function interprets a frame output status byte and returns
 * a 16-bit diagnostic value
 * @param statusByte - the output status byte
 * @returns a 16-bit diagnostic value
 */
uint16 TLD7002_getChannelDiagFromChannelStatusByteOut(TLD7002_FRAME_CHANNEL_STATUS_BYTE_t statusByte)
{
    uint16 errorFlags = NO_ERR;

    if (statusByte.OUT_STAT)
        errorFlags |= CH_OUT_STAT;

    if (statusByte.VFWD_WRN)
        errorFlags |= CH_VFWD_WRN;

    if (statusByte.CUR_WRN)
        errorFlags |= CH_CUR_WRN;

    if (statusByte.DC_WRN)
        errorFlags |= CH_DC_WRN;

    if (statusByte.OUT_SHORT_WRN)
        errorFlags |= CH_OUT_SHORT_WRN;

    if (statusByte.OVLD)
        errorFlags |= CH_OVLD_ERR;

    if (statusByte.SLS)
        errorFlags |= CH_SLS_ERR;

    if (statusByte.OL)
        errorFlags |= CH_OL_ERR;

    return errorFlags;
}

/*
 * @brief max: Returns the max between two signed 32-bit integers
 *  already present in Arduino
int32 max(int32 a, int32 b)
{
    return a > b ? a : b;
} */
