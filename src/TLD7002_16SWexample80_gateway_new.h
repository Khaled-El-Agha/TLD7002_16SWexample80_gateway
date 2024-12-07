/**
 *  @file TLD7002_ControlLayer.h
 *  @author Infineon
 *  @date 26.08.2024
 *	@brief TLD7002 specific Device Driver implementation from Infineon Technologies AG.
 *	@note  This file includes the definitions for the TLD7002 control layer
 *
 ***********************************************************************************************************************
 *
 * Copyright (c) 2024, Infineon Technologies AG
 * All rights reserved.
 *
 **********************************************************************************************************************
 */

/**
 * @addtogroup TLD7002_16SWEXAMPLE50_GATEWAY_NEW
 * @{
 */

#ifndef TLD7002_16SWEXAMPLE50_GATEWAY_NEW
#define TLD7002_16SWEXAMPLE50_GATEWAY_NEW

/*******************************************************************************
 *                               Defines
 ******************************************************************************/
/********** hard-coded HW dependent , example for TLD7002-16GWAY_DEMO board, adapt to your hardware if needed */
#define DEVICE_ADDR_4  4 /* address of device in the light system HSLI network */
#define DEVICE_ADDR_5  5 /* direct LED on turn indicator for GWAY_DEMO board*/    

/* Number of pixels in each LightFunc */
#define LB_PX_COUNT     1
#define HB_PX_COUNT     2 /* one for PWM one for diag */
#define TURN_PX_COUNT   16
#define DRL_PX_COUNT    1
#define BASIC2_PX_COUNT 4 /*  TLD2331 */
#define BASIC1_PX_COUNT 1 /*  TLD1173 */
#define VIN_SW_PX_COUNT 1 /* the MOSFET that enables VIN can be seen as a light function 100% duty => ON */

/********** hard-coded user configurable */
#define DUTY_TAIL    600  /* %oo Tail function duty cycle 1/tenThousand 10000=100% example 1000 = 10%*/
#define DUTY_STOP    9000 /* %oo Stop function duty cycle 10000=100% */
#define DUTY_TURN    2000 /* %oo Turn function duty cycle 10000=100% */
#define DUTY_LB      5000 /* %oo LB function duty cycle 10000=100% */
#define DUTY_DRL_POS 9000 /* %oo DRL dimmed to POS function duty cycle 10000=100% */
#define DUTY_FULL_ON 10000 /* %oo DRL dimmed to POS function duty cycle 10000=100% */

/* diagnostic mask, use it to consider only relevant warning flags (customize the selection for your specific application) */
#define DIAG_CARE_MASK_DIRECT (COMM_DRIVER_ERR |COMM_T_ERR | DEVICE_ERR | CH_OL_ERR | CH_VFWD_WRN | CH_SLS_ERR |CH_OUT_SHORT_WRN) /* diagnostic masks on on "direct LED functions" connected to the LTD70002-16 */
#define DIAG_CARE_MASK_GWAY  (COMM_DRIVER_ERR |COMM_T_ERR | DEVICE_ERR | CH_VFWD_WRN )/* On the gateway channels where PWMI and ERR is combined with glue logic, an error on the "external LED driver" is producing a VFWD_WRN (if the threshold is set to 1,25V). Disregard other eventual errors */
#define DIAG_CARE_MASK_HB    (COMM_DRIVER_ERR | CH_OL_ERR)/* On the gateway channel for High Beam ,the an error on the "external LED driver" is producing an open load. Disregard other eventual errors */

/******************************************************************************/
/*	Includes																  */
/******************************************************************************/
#include "types.h"

/* variables used to collect and analyze (masking) the light function diagnostics */
typedef struct
{
    uint16_t HBdiag;
    uint16_t LBdiag; 
    uint16_t turnDiag; 
    uint16_t DRLdiag;
    uint16_t BASIC1diag;
    uint16_t BASIC2diag;
} TLD7002_FunctionsDiag_t;

/* variables used to prepare the duty cycle of the light Functions before sending to the devices */
typedef struct
{
  uint16_t HBduty;
  uint16_t LBduty; 
  uint16_t DRLduty;
  uint16_t BASIC1duty;
  uint16_t BASIC2duty;
} TLD7002_FunctionsDc_t;

/* storing sensors values */
typedef struct
{
  uint16_t VLEDmV;
  uint16_t DTStemp;
} TLD7002_SensorValues_t;






#endif /* TLD7002_16SWEXAMPLE50_GATEWAY_NEW */
/**@}*/