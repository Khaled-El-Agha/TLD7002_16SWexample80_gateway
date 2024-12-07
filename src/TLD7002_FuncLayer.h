/**
 *  @file TLD7002FuncLayer.h
 *  @author Infineon
 *  @date 06.03.2024
 *  @version v1.05
 *  @brief TLD7002-16ES functional layer from Infineon Technologies AG.
 *
 ***********************************************************************************************************************
 *
 * Copyright (c) 2024, Infineon Technologies AG
 * All rights reserved.
 *
 **********************************************************************************************************************/

#ifndef TLD7002FUNC_LAYER_H_
#define TLD7002FUNC_LAYER_H_

/***********************************************************************************************************************
 *                                                     Includes                                                       **
 **********************************************************************************************************************/
#include "TLD7002_ServiceLayer.h"
#include "TLD7002_HAL.h"

/***********************************************************************************************************************
 *                                             Global Macro Declarations                                              **
 **********************************************************************************************************************/
#define BROADCAST_ADDRESS (0)     /* Broadcast message address */
#define INTERFR_DLY_MIN_ARDUINO (100)   /*[us] Interframe delay minimum allowed for Arduino UNO R3 softserial*/
#define INTERFR_DLY_MAX_TLD7002 (2500)  /*[us] highest possible interframe delay for TLD7002-16*/

#define INIT2ACTIVE_DELAY   (250)   /* [us]Time needed from the TLD7002 to "wake up" from INIT mode */
#define FAILSAFE2INIT_DELAY (5000)  /* [us]Time needed from the TLD7002 from fail safe to INIT mode 250us + 2(1/fPWM)@400Hz*/
#define PWM_PERIOD_uS       (1120)  /* [us]Output LED PWM period, default is 300Hz =>3.3ms, setup is 899Hz =>1.1ms*/
#define EMULATION_PASSW     (0x3BD2) /* Password needed in order to emulate the TLD7002-16 OTP. See TLD7002-16 OTP app note */
#define OTP_WRITE_PASSW     (0xA47B) /* Password needed in order to emulate the TLD7002-16 OTP. See TLD7002-16 OTP app note */

#define TLD7002_CHANNELS    (16)     /* Total output channels in the TLD7002-16 */
#define TLD7002MIN_CUR_UA   5625     /* [uA] Minimum output current set for TLD7002-16 (5,625mA) */     
#define TLD7002MAX_CUR_UA   76500    /* [uA] Minimum output current set for TLD7002-16 (76,5mA) */ 
#define TLD7002CUR_STEP_UA  1125     /* [uA] Output current set for TLD7002-16 (1,125mA) */     

/* lenght of DLC4 is (TLD7002_LEN_WRITE_REG_DLC4_WRITE-TLD7002_LEN_WRITE_REG_OVHD)/2 = 8 16-bit words*/
#define DLC3_LENGTH         ((TLD7002_LEN_WRITE_REG_DLC3_WRITE - TLD7002_LEN_WRITE_REG_OVHD)/2U) /*payload lenght for DLC4_WRITE in 16 bits words*/
#define DLC4_LENGTH         ((TLD7002_LEN_WRITE_REG_DLC4_WRITE - TLD7002_LEN_WRITE_REG_OVHD)/2U) /*payload lenght for DLC4_WRITE in 16 bits words*/
#define DLC6_LENGTH         ((TLD7002_LEN_WRITE_REG_DLC6_WRITE - TLD7002_LEN_WRITE_REG_OVHD)/2U) /*payload lenght for DLC6_WRITE in 16 bits words*/
#define DLC7_LENGTH         ((TLD7002_LEN_WRITE_REG_DLC7_WRITE - TLD7002_LEN_WRITE_REG_OVHD)/2U) /*payload lenght for DLC7_WRITE in 16 bits words*/
#define OTP_USER_SIZE       (40)    /* Number of 16-bit OTP words modifiable by the user for configuration */
#define OTP_READ_ONLY_SIZE  (4)     /* Number of 16-bit OTP read only words OTP_LOG_WORD0 to OTP_LOG_WORD3 */
#define OTP_START_ADDR      (0x83)
#define OTP_READ_ONLY_START_ADDR (OTP_START_ADDR + OTP_USER_SIZE )

#define ADC_GPIN_FULL_SCALE_mV  (5496U)     /* GPINx has 5496mV full scale 1023 value       */
#define ADC_VOUT_FULL_SCALE_mV  (20067U)    /* VOUT has 20067mv full scale 1023 value       */
#define ADC_VLED_FULL_SCALE_mV  (20067U)    /* VLED has 20067mv full scale 1023 value       */
#define ADC_VS_FULL_SCALE_mV    (20067U)    /* VS has 20067mv full scale 1023 value         */
#define ADC_RESOLUTION_LSB      (1023U)     /* ADC resolution in LSB (quantization levels)  */

/* Error masks define */
#define NO_ERR            (0x0000) /* No error flag detected */
#define CH_OUT_STAT       (0x0001) /* OUT STAT bit is present signaling that the output is OFF (duty cycle is 0) */
#define CH_VFWD_WRN       (0x0002) /* Forward Voltage Warning */
#define CH_CUR_WRN        (0x0004) /* Current warning */
#define CH_DC_WRN         (0x0008) /* Duty cycle warining */
#define CH_OUT_SHORT_WRN  (0x0010) /* Output short to adjacent pin warning is present */
#define CH_OVLD_ERR       (0x0020) /* Channel Overload error (overtemperature on the channel) */
#define CH_SLS_ERR        (0x0040) /* Single LED short error */
#define CH_OL_ERR         (0x0080) /* Open load error */
#define DEVICE_UV_ERR     (0x0100) /* VS or VLED undervoltage is present: therefore other diagnostic will ont report any fault */
#define DEVICE_FAULT_ERR  (0x0200) /* Device Fault bit in the output status byte is 1 */
#define DEVICE_MODE_ERR   (0x0400) /* Device mode is not Emulation or Active, therefore channels cannot be turned on or ADC can not work properly */
#define COMM_T_ERR        (0x0800) /* Frame termination flag sent by the device is =1 : the TLD7002-16 has not accepted thee last HSLI command */
#define COMM_DRIVER_ERR   (0x1000) /* Device driver has not been able to decode the TLD7002-16 reply / or missing reply from the device */
#define ADC_ERR           (0x2000) /* ADC read not valid  */
#define OPER_ERR          (0x4000) /* Operation Error - the operation requested was not completed successfully  */

/* Cumulative error masks */
#define CHANNEL_ERR       (CH_OUT_STAT | CH_VFWD_WRN | CH_CUR_WRN | CH_DC_WRN | CH_OUT_SHORT_WRN \
                           | CH_OVLD_ERR | CH_SLS_ERR | CH_OL_ERR) /* filters all errors related to TLD7002-16 channels */

#define DEVICE_ERR        (DEVICE_MODE_ERR | DEVICE_FAULT_ERR | DEVICE_UV_ERR) /* filters all errors related to TLD7002-16 device */
#define COMM_ERR          (COMM_T_ERR | COMM_DRIVER_ERR )/* filters all errors related to communications with the TLD7002-16 device */

/***********************************************************************************************************************
 *                                             MUST BE CONFIGURED BY USER                                             **
 **********************************************************************************************************************/
#define NUM_OF_DEVICES      (2)  /* Number of TLD7002 devices (needed for lightSystem and LightFunc functionalities)*/

/***********************************************************************************************************************
 *                                        Multi-instance support definitions                                          **
 **********************************************************************************************************************/

/*
 * A pixel correspond to a single channel of a TLD7002-16 device
 * A pixel can have a 10 bit duty cycle and a diagnostic output status byte
 * The pixel struct is for internal use in the implementation
 */
typedef struct
{
    uint16      duty;   /* Duty cycle of the pixel                      */
    uint16      diag;   /* Diagnostic output status byte of the pixel   */

} pixel_s;


/*
 * Struct used to identify a pixel inside a lightFunc_s
 */
typedef struct
{
    const uint8     device;   /* HSLI device address of the pixel     */
    const uint8     channel;  /* Channel number of the pixel (0-15)   */

} funcPixel_s;


/*
 * A light system groups all the light functions associated to the same HSLI bus.
 * Each light system is associated with one and only one HSLI network.
 * A light system contains internally a matrix of numOfDevice x TLD7002_CHANNELS pixels.
 * Every row represents a TLD7002-16 device in the light system; every column represent a channel in the TLD7002-16 device.
 * Every cell in the matrix correspond to a specific device-channel couple and indicates its duty cycle and diagnostic osb.
 * A lightSystem_s instance shall be initialized calling TLD7002_initLightSystem before performing any other lightSystem operation
 */
typedef struct
{
    TLD7002_NetworkInstance_t   *mcldNet;                      /* TLD7002 as gateway physical network instance           */
    const uint8                 addressList[NUM_OF_DEVICES];    /* List of unique devices addresses            */
    uint8                       addressCount;                   /* Number of actual unique addresses (devices) within the light system*/
    uint16                      deviceDiag [NUM_OF_DEVICES];    /* Array of device diagnostics                            */
    pixel_s                     pixelMatrix[NUM_OF_DEVICES][TLD7002_CHANNELS];  /* Matrix of pixels */

} lightSystem_s;


/*
 * A light function is a grouping of different pixels (LED connected to a TLD7002-16 channel), on the same HSLI network.
 * the pixels can be distributed on one or more TLD7002-16 devices. The pixels of a light function can be addressed individually
 * or all at once
 */
typedef struct
{
    lightSystem_s       *system;                /* Pointer to the light system                      */
    uint8               numOfPixels;            /* Number of pixels in the light function           */
    const funcPixel_s   pixelList[];            /* List of pixels in the light function             */

} lightFunc_s;

/***********************************************************************************************************************
 *                                           Global Function Declarations                                             **
 **********************************************************************************************************************/
void    TLD7002_initDrivers(TLD7002_NetworkInstance_t* HSLInetwork);
bool    TLD7002_initDevice(TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16  TLD7002_OTPwrite(const uint16 *otpCfg, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr, uint16 newInterfrDelay, uint16 gpin0Pin);
uint16  TLD7002_OTPemulate(const uint16 *otpCfg, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addrAfterEmu, uint16 newInterfrDelay, uint16 gpin0Pin);
uint16  TLD7002_OTPread(uint16 *otpCfg, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);

uint16  TLD7002_readNTC_GPIN1(float *tempNTC, TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16  TLD7002_readGPIN0(uint16 *GPINmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16  TLD7002_readGPIN1(uint16 *GPINmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16  TLD7002_readDTS( uint16 *DTStemp,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16  TLD7002_readVFWD(uint16 *VFWDmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr, uint8 channel);
uint16  TLD7002_readVLED(uint16 *VLEDmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16  TLD7002_readVS(uint16 *VSmV,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);
uint16 TLD7002_setCurrAll(uint32 currUA,TLD7002_NetworkInstance_t* HSLInetwork, uint8 addr);

/*------------------------------------------- lightSystem & LightFunc API --------------------------------------------*/
boolean  TLD7002_initLightSystem(lightSystem_s *system);

uint16  TLD7002_getLocalDiagPixel(lightFunc_s *func, uint8 index);
boolean TLD7002_setLocalDutyPixel(lightFunc_s *func, uint8 index, uint16 duty);

uint16  TLD7002_getLocalDiagLightFunc(lightFunc_s *func);
boolean TLD7002_setLocalDutyLightFunc(lightFunc_s *func, uint16 duty);

void    TLD7002_txDutyLightSystem(lightSystem_s *system, uint8 *buffTxRx);
void    TLD7002_rxDiagLightSystem(lightSystem_s *system, uint8 *buffTxRx);
void    TLD7002_txClrDiagLightSystem(lightSystem_s *system, uint8 *buffTxRx);

void    TLD7002_printDiag(uint16 diag);
void    TLD7002_printAddressDiag(lightSystem_s *system, uint8 *buffTxRx, uint8 addr);

/*-------------------------------------- Device Driver Wrappers TX,RX combined ---------------------------------------*/
uint8 TLD7002_TRX_READ_REG_DLC1(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr);
uint8 TLD7002_TRX_DC_UPDATE_14BIT(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint16* dcVal);
uint8 TLD7002_TRX_DC_UPDATE_8BIT(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint16* dcVal);
uint8 TLD7002_TRX_DC_UPDATE_DLC0(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr);
uint8 TLD7002_TRX_HWCR_ALL(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr);
uint8 TLD7002_TRX_PM_CHANGE(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, TLD7002_FRAME_POWER_MODE_CHANGE_t mode);
uint8 TLD7002_TRX_WRITE_REG_DLC7(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr, uint16* data);
uint8 TLD7002_TRX_WRITE_REG_DLC6(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAdd, uint16* data);
uint8 TLD7002_TRX_WRITE_REG_DLC4(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr, uint16* data);
uint8 TLD7002_TRX_WRITE_REG_DLC1(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr, uint16 data);
uint8 TLD7002_TRX_READ_REG_DLC3(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr);
uint8 TLD7002_TRX_READ_REG_DLC4(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr);
uint8 TLD7002_TRX_READ_REG_DLC6(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr, uint8 startAddr);
uint8 TLD7002_TRX_READ_OST(TLD7002_NetworkInstance_t *mcldNet, uint8* buffTxRx, uint8 addr) ;
void  TLD7002_TX_BROADCAST_DC_SYNC(TLD7002_NetworkInstance_t* HSLInetwork,uint8* buffTxRx);


#endif /* TLD7002FUNC_LAYER_H_ */
