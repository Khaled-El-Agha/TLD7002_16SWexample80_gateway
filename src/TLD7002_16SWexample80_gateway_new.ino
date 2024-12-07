 /*********************************************************************************************************************
 * @file   TLD7002_16SWexample70_gateway.ino
 * @author Infineon
 * @date   04.10.2024
 * @version v1.48
 * @brief Example of usage of the TLD7002-16ES Functional Layer (version 1.01) to build a gateway "light system" 
 * This example code runs on a TLD7002-16GWAY_DEMO P02 connected to an Arduino UNO rev3 board, as shown in the 
 * TLD7002-16GWAY_DEMO manual. If the proper LED loads are connected to the TLD7002-16GWAY_DEMO P02 the sketch will  
 * enable in sequence all the loads: DRL on TLD6098, LB and then HB on TLD5191, Tail light on the	TLD2331-3ET, Side Marker on 
 * a TLD1173-1ET and a wiping indicator on the second TLD7002-16ES device.
 * Serial monitor is available for debug printf, it has to be set to 230400 baud  in Arduino UNO R3
**********************************************************************************************************************
 *
 * Copyright (c) 2024, Infineon Technologies AG
 * All rights reserved.
 *
 *********************************************************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "TLD7002_FuncLayer.h"
#include "TLD7002_16SWexample80_gateway_new.h"

/*******************************************************************************
**                      Global Variable Definitions                           **
*******************************************************************************/

/* HSLI network instance, needed by the device drivers to know to which physical bus are the HSLI messages are for.*/
TLD7002_NetworkInstance_t gHSLInetwork =
{
  0,                      /* reset master rolling counter */
  { 0 },                  /* reset the slave rolling counters */
  sendMCLDMessage,        /* microcontroller specific writing UART over CAN function */
  readReceivedMCLDAnswer, /* microcontroller specific reading UART over CAN function */
  emptyingReceiveBuffer,  /* microcontroller specific emptying the UART receive buffer function */
  generateStartSyncBreak, /* microcontroller specific function to start sync break generation */
  generateStopSyncBreak,  /* microcontroller specific function to stop sync break generation */
};

extern char gBuffPrint[]; /* buffer for Debug prints */

/* HSLI Buffer needed by the TLD7002-16 device driver to transmit UART HSLI frames, or by the application to retrieve HSLI data */
extern uint8 gBuffTxRx [sizeof (TLD7002_READ_OST_FRAME_t)+5];

static TLD7002_FunctionsDiag_t functionsDiag = { 0u };    /* variables used to collect and analyze (masking) the light function diagnostics */
static TLD7002_FunctionsDc_t functionsDC = { 0u };        /* variables used to prepare the duty cycle of the light Functions before sending to the devices */
static TLD7002_SensorValues_t sensorsValues = { 0u };     /* storing sensors values */
static uint16_t state = 0;                                /* state for the light scenario changing */
static bool initSucces = FALSE;
static uint16_t err=0;
static uint16_t wipeIndex = 0;

/*******************************************************************************
//            light system / light function definitions
*******************************************************************************/
/* lightSystem instance : light system groups all the light functions associated to the same HSLI bus */
lightSystem_s gSys = {
  .mcldNet        = &gHSLInetwork,                 /* Pointer to the HSLI network structure instance */
  .addressList    = {DEVICE_ADDR_4, DEVICE_ADDR_5},/* List of addresses of the devices connected to the HSLI network */
  .addressCount   = NUM_OF_DEVICES                 /* NOTE: NUM_OF_DEVICES define in TLD7002_FuncLayer.h has to be changed in case of system with different number of devices */
};

/* lightFunctions, are a grouping of different pixels on the same HSLI network.
 * define here each pixel of the light function. PIX = (device address, channel) */

/* Low Beam light function PWM and diagnostic are in the same output*/
lightFunc_s gLBfunc = {
   .system         = &gSys,              /* pointer to the lightSystem where the light function belongs */
   .numOfPixels    = LB_PX_COUNT,        /* number of pixel on this funciton */
   .pixelList      = {{DEVICE_ADDR_4 , 7}}/* PIX0 : device address, channel. this pixel is used to control LB gateway with TLD5191, for both PWM and diagnosis*/
};

/* High Beam light function PWM and diagnostic are in 2 different pixels*/
#define HB_DIAG_PIX 1 /* NB pixel start counting from PIX0 , the PIX 1 is the second in the array */
lightFunc_s gHBfunc = {
   .system         = &gSys,                 /* pointer to the lightSystem where the light function belongs */
   .numOfPixels    = HB_PX_COUNT,           /* number of pixel on this funciton */
   .pixelList      = {{DEVICE_ADDR_4 , 9},  /* PIX0 HIgh Beam enable, keep to 100% , duty is same of LB*/
                      {DEVICE_ADDR_4 , 10}} /* PIX1 HIgh Beam enable, keep to 100% , duty is same of LB*/
};

/* DRL lightFunction  define here each pixel of the light function (device address, channel) */
lightFunc_s gDRLfunc = {
  .system         = &gSys,                  /* pointer to the lightSystem where the light function belongs */
  .numOfPixels    = DRL_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_4 , 12}} /* PIX0 : (device address, channel). this pixel is used to control DRL gateway with TLD6098 on the TLD7002-16GWAY_DEMO_P02S02, for both PWM and diagnosis */
};

/* TURN lightFunction  define here each pixel of the light function (device address, channel) */
lightFunc_s gTurnFunc = {
  .system         = &gSys,                /* pointer to the lightSystem where the light function belongs */
  .numOfPixels    = TURN_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_5 , 0},/* PIX0 : device address, channel) */
                      {DEVICE_ADDR_5 , 1},/* PIX1 : device address, channel) */
                      {DEVICE_ADDR_5 , 2},/* PIX2 : device address, channel) */
                      {DEVICE_ADDR_5 , 3},
                      {DEVICE_ADDR_5 , 4},
                      {DEVICE_ADDR_5 , 5},
                      {DEVICE_ADDR_5 , 6},
                      {DEVICE_ADDR_5 , 7},
                      {DEVICE_ADDR_5 , 8},
                      {DEVICE_ADDR_5 , 9},
                      {DEVICE_ADDR_5 , 10},
                      {DEVICE_ADDR_5 , 11},
                      {DEVICE_ADDR_5 , 12},
                      {DEVICE_ADDR_5 , 13},
                      {DEVICE_ADDR_5 , 14},
                      {DEVICE_ADDR_5 , 15}}
};

/* BASIC+ lightFunction  define here each pixel of the light function (device address, channel) */
#define BASIC_DIAG_PIX 3 /* NB pixel start counting from PIX0 , the PIX3 is the fourth */
lightFunc_s gBASIC2func = {
  .system         = &gSys,                  /* pointer to the lightSystem where the light function belongs */
  .numOfPixels    = BASIC2_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_4 , 1}, /* PIX0 : device address, channel :only PWM is present, no Diagnostic*/
                      {DEVICE_ADDR_4 , 2}, /* PIX1 : device address, channel :only PWM is present, no Diagnostic*/
                      {DEVICE_ADDR_4 , 3}, /* PIX2 : device address, channel :only PWM is present, no Diagnostic*/
                      {DEVICE_ADDR_4 , 4}} /* PIX3:  here is connected the diagnostic ERRN , producing an OL in case a fault is happening on the Basic + */
};


/* BASIC+ lightFunction  define here each pixel of the light function (device address, channel) */
lightFunc_s gBASIC1func = {
  .system         = &gSys,                  /* pointer to the lightSystem where the light function belongs */
  .numOfPixels    = BASIC1_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_4 , 0}}/* PIX0 : device address, channel */
};


/* VinSwitch Function, that is used to enable the power supply ot external LED drivers on the gateway app.
   define here each pixel of the light function (device address, channel) */
#define O11A_SW_VIN 11
lightFunc_s gVINswitchFunc = {
  .system         = &gSys,                // pointer to the lightSystem where the light function belongs
  .numOfPixels    = VIN_SW_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_4 , O11A_SW_VIN}/* PIX0 : device address, channel */
                     }
};
#define O5A_SET1_H 5
#define O6A_SET2_H 6
/* H Bridge DCDC set current Function, on TLD7002-16GWAY_DEMO_S02_P02 if out5 & out6 are set to 0% the output current is 1A
   define here each pixel of the light function (device address, channel) */
lightFunc_s gHsetFunc = {
  .system         = &gSys,                        // pointer to the lightSystem where the light function belongs
  .numOfPixels    = VIN_SW_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_4 , O5A_SET1_H},/* PIX0 :Out5@100%=>0,375mA if Pix2 is 0% */
                      {DEVICE_ADDR_4 , O6A_SET2_H} /* PIX1 :Out6@100%=>0,72A if pix1 is 0%  */
                      }
};
/* Boost TLD6098 DCDC set current Function, on TLD7002-16GWAY_DEMO_S02_P02 if out14 & out15 are set to 0% the output current is 1A 
   define here each pixel of the light function (device address, channel) */
#define O14A_DRL_SET 14
#define O15A_DRL_SET 15
lightFunc_s gBsetFunc = {
  .system         = &gSys,                        // pointer to the lightSystem where the light function belongs
  .numOfPixels    = VIN_SW_PX_COUNT,        
  .pixelList      = { {DEVICE_ADDR_4 , O14A_DRL_SET},/* PIX0 : Out14@100%=>0,26A if pix2 is 0%  */
                      {DEVICE_ADDR_4 , O15A_DRL_SET} /* PIX1 : Out15@100%=>0,66A if pix1 is 0% */
                      }
};

/*******************************************************************************
**                      Function Prototypes                                 **
*******************************************************************************/
void printDiag(const char* diagText,uint16 diag );
boolean trueEvery1s();
boolean trueEvery10ms();
boolean trueEvery50ms();

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/

/**
 * @brief Arduino setup function, called only once at startup, typically hardware modules init are performed here
 */
void setup(void) 
 {
   Serial.begin(9600);
   while (!Serial && (millis() < 5000)); // wait up to 5 seconds to have the debug serial ready (important for Arduino R4)
  
   /* pin definition is in "TLD7002_HAL.h" file, the Hardware abstraction layer for the TLD7002-16 device drivers */
   /* set Arduino pins directions */
   pinMode(PIN_GPIN0A,   OUTPUT);    
   pinMode(PIN_GPIN0B,   OUTPUT); 
}

/**
 * @brief Arduino main loop function called repeatedly  
 */
void loop(void)
{

  /************   initializations: drivers and devices     ****************/
  /* initialize TLD7002-16 Device Drivers (but not the TLD7002-16 device itself) */
  TLD7002_initDrivers(&gHSLInetwork);
  printHAL("-Gateway Examples V1.47-\r\n");

  /* Initialize the lightSystem structs and the TLD7002-16 devices in the light system */
  initSucces = TLD7002_initLightSystem(&gSys);
  if (initSucces == true)
    printf("init OK\n");
  else
    printHAL("** init ERR\n");

  /* Gateway Hardware configuration: enable VIN switch mosfet to power "external LED drivers" and prepare SET analog dimming on DCDC*/
  TLD7002_setLocalDutyLightFunc(&gVINswitchFunc, DUTY_FULL_ON);/* enable VIN switch moseft */
  TLD7002_setLocalDutyPixel(&gHsetFunc, 0, DUTY_FULL_ON);      /* low HBridge current, see TLD7002-16GWAY schematic*/
  TLD7002_setLocalDutyPixel(&gBsetFunc, 0, DUTY_FULL_ON);      /* low TLD6098 boost DCDC current, see TLD7002-16GWAY schematic*/
  TLD7002_txDutyLightSystem(&gSys, gBuffTxRx); /* Transmit the duty cycles to all the TLD7002-16 channels in the lightSystem  */

  /* loop */
  while (1)
  {
    /********** periodic light system update , every 10ms  ****************/
    if (trueEvery10ms() == TRUE)
    {
      /* HSLI bus interactions: send duty cycles and retrieve diagnostic on the HSLI bus to all devices in the system  */
      TLD7002_txDutyLightSystem(&gSys, gBuffTxRx); /* Transmit the duty cycles to all the TLD7002-16 channels in the lightSystem  */
      delay_uS(INTERFR_DLY_MIN_ARDUINO );                    /* wait for interframe delay before HSLI command */
      TLD7002_rxDiagLightSystem(&gSys, gBuffTxRx); /* Receive all the lightSystem channels diagnostic and fill local lightSystem matrix with the RX values  */

      /* Local variables interactions: analyze and print LightFuncs cumulative (all pixels of that funciton) diagnosis flags  */
      functionsDiag.LBdiag    = TLD7002_getLocalDiagLightFunc(&gLBfunc);  /* retrieve diag from local matrix, filled earlier by TLD7002_rxDiagLightSys */
      functionsDiag.turnDiag  = TLD7002_getLocalDiagLightFunc(&gTurnFunc);
      functionsDiag.DRLdiag   = TLD7002_getLocalDiagLightFunc(&gDRLfunc); 
      functionsDiag.BASIC1diag = TLD7002_getLocalDiagLightFunc(&gBASIC1func);
      functionsDiag.BASIC2diag = TLD7002_getLocalDiagPixel(&gBASIC2func, BASIC_DIAG_PIX); /* diagnostic on Basic + it is only on pixel 4, the other piels are only used to provide PWM */      
      if  ( functionsDC.HBduty != 0 ) // only if HB is ON
        functionsDiag.HBdiag = TLD7002_getLocalDiagPixel(&gHBfunc, HB_DIAG_PIX);    /* diagnostic on HB on one pix, the other pixel is used to provide PWM */      
    } /* if every 10 msec */

    /********** On application demand (in example every 1 second): change light scenario in the local array, and read ADCs *************/
    if (trueEvery1s() == TRUE)
    {
      /*  update stop and tail duty cycle temp variables */
      switch(state){
        case 0:
            functionsDC.LBduty = 0;  
            functionsDC.HBduty = 0;  
            functionsDC.DRLduty = 0; 
            functionsDC.BASIC1duty = 0;
            functionsDC.BASIC2duty = DUTY_TAIL;
            state = 1;
            break;
        case 1:
            functionsDC.LBduty = DUTY_LB;
            functionsDC.HBduty = 0;  
            functionsDC.DRLduty = DUTY_DRL_POS/10;
            functionsDC.BASIC1duty = DUTY_TAIL;
            functionsDC.BASIC2duty = 0;
            state = 2;
            break;
        case 2:
            functionsDC.LBduty = DUTY_LB;
            functionsDC.HBduty = DUTY_FULL_ON;  
            functionsDC.DRLduty = DUTY_DRL_POS/10;;
            functionsDC.BASIC1duty = DUTY_TAIL;
            functionsDC.BASIC2duty = DUTY_STOP;
            state = 3;
            break;

        case 3:
            functionsDC.LBduty = 0;  
            functionsDC.HBduty = 0;  
            functionsDC.DRLduty = DUTY_DRL_POS;
            functionsDC.BASIC1duty = DUTY_TAIL;
            functionsDC.BASIC2duty = DUTY_STOP;
            state = 0;
            break;
      }

      /* update local duty cycles: work on light system duty cyle LOCAL matrix, before sending all toghether to the TLD7002 devices */
      TLD7002_setLocalDutyLightFunc(&gLBfunc, functionsDC.LBduty);
      TLD7002_setLocalDutyLightFunc(&gHBfunc, functionsDC.HBduty);
      TLD7002_setLocalDutyLightFunc(&gDRLfunc, functionsDC.DRLduty);
      TLD7002_setLocalDutyLightFunc(&gBASIC1func, functionsDC.BASIC1duty);
      TLD7002_setLocalDutyLightFunc(&gBASIC2func, functionsDC.BASIC2duty);
      /* turn duty cycle is updated on an the 50ms task that performs wiping */
     
      /* diagnostic prints & clear TLD7002-16 warning flags */
      printDiag("LBdiag=",functionsDiag.LBdiag & DIAG_CARE_MASK_GWAY );/* filter lightFunc diag. with DIAG_CARE_MASK and print only when errors are present */
      printDiag("turnDiag=", functionsDiag.turnDiag & DIAG_CARE_MASK_DIRECT);
      printDiag("DRLdiag=",functionsDiag.DRLdiag & DIAG_CARE_MASK_GWAY); 
      printDiag("BASIC1diag=",functionsDiag.BASIC1diag & DIAG_CARE_MASK_GWAY);
      printDiag("BASIC2diag=",functionsDiag.BASIC2diag & DIAG_CARE_MASK_GWAY);
      printDiag("HB=",functionsDiag.HBdiag & DIAG_CARE_MASK_HB);
      TLD7002_txClrDiagLightSystem(&gSys, gBuffTxRx);    /* send HWCLR to the TLD7002-16 in the LightSystem to clear them*/

      /* OPTIONAL ADC readings: retrieve them from the TLD7002-16 and print them */
      err = TLD7002_readVLED( &sensorsValues.VLEDmV , &gHSLInetwork, DEVICE_ADDR_5);/* read VLED voltage */
      delay_uS(INTERFR_DLY_MIN_ARDUINO ); 
      err |= TLD7002_readDTS( &sensorsValues.DTStemp , &gHSLInetwork, DEVICE_ADDR_5);/* read Die Temp Sensor  */
      delay_uS(INTERFR_DLY_MIN_ARDUINO ); 
      if (err == NO_ERR){
        sprintf(gBuffPrint, "VLED pin= %dmV, DTS= %d°C\r\n", sensorsValues.VLEDmV, sensorsValues.DTStemp ); 
        printHAL(gBuffPrint);
      }else{ 
        sprintf(gBuffPrint, "read ERR=0x%X\r\n", err);
        printHAL(gBuffPrint);
      }
    } /* if every 1sec */

    /********** wiping indicator , wipe local duty cycles on Turn ligh function ********/
    if (trueEvery50ms() == TRUE)
    {
      if (wipeIndex < TURN_PX_COUNT)
      {
        TLD7002_setLocalDutyPixel(&gTurnFunc, wipeIndex++, DUTY_TURN);
      }else{
         wipeIndex=0;
         TLD7002_setLocalDutyLightFunc(&gTurnFunc, 0);
      }      
    } /* if every 100ms */
  }/* while(1)  */
}

/* functions that return true only once every 1 seconds */
boolean trueEvery1s()
{
  static uint32 lastUpdate = 0;

  if (millis() - lastUpdate >= 1000)
  {
    lastUpdate = millis();
    return true;
  }
  else
      return false;
}

/* functions that return true only once every 10 milli seconds */
boolean trueEvery10ms()
{
  static uint32 lastUpdate = 0;

  if (millis() - lastUpdate >= 10)
  {
    lastUpdate = millis();
    return true;
  }
  else
      return false;
}

/* functions that return true only once every 10 milli seconds */
boolean trueEvery50ms()
{
  static uint32 lastUpdate = 0;

  if (millis() - lastUpdate >= 50)
  {
    lastUpdate = millis();
    return true;
  }
  else
      return false;
}

/* functions that prints diagnostic if it is different from NO_ERR  */
void printDiag(const char* diagText,uint16 diag )
{
  if (diag != NO_ERR){
      printHAL(diagText);
      TLD7002_printDiag(diag);
  }
}

/* [] END OF FILE */
