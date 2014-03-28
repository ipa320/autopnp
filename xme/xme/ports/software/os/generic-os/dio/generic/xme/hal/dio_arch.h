/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dio_arch.h 3458 2013-05-23 09:37:04Z ruiz $
 */

/**
 * \file
 *         Digital I/O abstraction (platform specific part: generic OS based
 *         implementation).
 */

#ifndef XME_HAL_DIO_ARCH_H
#define XME_HAL_DIO_ARCH_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/dio.h"

#include "xme/core/mdl.h"

#include <stdint.h>

/******************************************************************************/
/***   Model constraints                                                    ***/
/******************************************************************************/
XME_MDL_RESOURCE_PROVIDE("dio", "port", "A", "Virtual Port A (8 bit)")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A0", "Virtual Pin A.0")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A1", "Virtual Pin A.1")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A2", "Virtual Pin A.2")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A3", "Virtual Pin A.3")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A4", "Virtual Pin A.4")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A5", "Virtual Pin A.5")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A6", "Virtual Pin A.6")
XME_MDL_RESOURCE_PROVIDE("dio", "portpin", "A7", "Virtual Pin A.7")

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define XME_HAL_DIO_NUM_PORTS 1 ///< the number of ports. 
#define XME_HAL_DIO_PORT_A 0    ///< the value of port A.
#define XME_HAL_DIO_PIN_A0 0    ///< the value of port A0.
#define XME_HAL_DIO_PIN_A1 1    ///< the value of port A1.
#define XME_HAL_DIO_PIN_A2 2    ///< the value of port A2.
#define XME_HAL_DIO_PIN_A3 3    ///< the value of port A3.
#define XME_HAL_DIO_PIN_A4 4    ///< the value of port A4.
#define XME_HAL_DIO_PIN_A5 5    ///< the value of port A5.
#define XME_HAL_DIO_PIN_A6 6    ///< the value of port A6.
#define XME_HAL_DIO_PIN_A7 7    ///< the value of port A7.

#endif // #ifndef XME_HAL_DIO_ARCH_H
