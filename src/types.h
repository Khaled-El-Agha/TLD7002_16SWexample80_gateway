/**
 *  @file types.h
 *  @author Infineon
 *  @date 03.08.2022
 *	@brief this file provide to the compiler the data types names adopted by the device driver. In the TLD7002-16 device
 *      driver the AUTOSAR application data types name are used. In the file TLD7002.h a corresponding include of types.h is 
 *      provided with an option to turn off the types.h  include with #define TLD7002_INCLUDE_EXT_TYPEDEF 0, in case AUTOSAR 
 *      type defines are already present in the compiler (not the case for Arduino ide)
 *
 ***********************************************************************************************************************
 *
 * Copyright (c) 2022, Infineon Technologies AG
 * All rights reserved.
 *
 **********************************************************************************************************************/

 
#ifndef type_H_
#define type_H_

/*******************************************************************************
**                                  Includes                                  **
*******************************************************************************/
#include <Arduino.h>

/*******************************************************************************
**                         Global Macro Declarations                          **
*******************************************************************************/
#define TRUE true
#define FALSE false

/*******************************************************************************
**         Typedefs TODO: adjust according to your compiler                    **
*******************************************************************************/

typedef int8_t                 int8;
typedef int32_t                int32;
typedef uint8_t                uint8;
typedef uint16_t               uint16;
typedef uint32_t               uint32;
typedef uint64_t               uint64;
typedef bool                   boolean;

#endif /* type_H_ */
