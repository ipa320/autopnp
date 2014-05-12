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
 * $Id: console.h 2837 2013-04-03 11:52:26Z tum.martin.perzl $
 */

/** 
 * \file 
 * \brief Console abstraction.
 */

#ifndef XME_HAL_CONSOLE_H
#define XME_HAL_CONSOLE_H

/**
 * \defgroup hal_console Console Component
 * @{
 *
 * \brief  Universal console abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdint.h>
#include "xme/defines.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/** 
 * \enum xme_hal_console_base_t
 * \brief Display type for number.
 * \see   xme_hal_console_number()
 */
typedef enum
{
	XME_HAL_CONSOLE_BASE_INVALID = 0, ///< the display is invalid.
	XME_HAL_CONSOLE_BASE2 = 2, ///< Display the console as binary.
	XME_HAL_CONSOLE_BASE10 = 10, ///< Display as decimal.
	XME_HAL_CONSOLE_BASE16 = 16 ///< Display as hexadecimal.
}
xme_hal_console_base_t;

/** 
 * \enum xme_hal_console_num_t
 * \brief Type of number to show on the console.
 * \see   xme_hal_console_number()
 */
typedef enum
{
	XME_HAL_CONSOLE_NUM_INVALID = 0, ///< Invalid type of number.
	XME_HAL_CONSOLE_NUM_UINT8 = 1,   ///< Type of number to use unsigned int 8 bits.
	XME_HAL_CONSOLE_NUM_INT8 = 2,    ///< Type of number to use signed int 8 bits.
	XME_HAL_CONSOLE_NUM_UINT16 = 3,  ///< Type of number to use unsigned int 16 bits.
	XME_HAL_CONSOLE_NUM_INT16 = 4,   ///< Type of number to use signed int 16 bits.
	XME_HAL_CONSOLE_NUM_UINT32 = 5,  ///< Type of number to use unsigned int 32 bits.
	XME_HAL_CONSOLE_NUM_INT32 = 6    ///< Type of number to use signed int 32 bits.
}
xme_hal_console_num_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the console.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the console has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the console initialization failed.
 */
xme_status_t
xme_hal_console_init(void);

/**
 * \brief  Frees resources occupied by the console abstraction.
 */
void
xme_hal_console_fini(void);

/**
 * \brief Reset console (clear screen).
 */
void
xme_hal_console_reset(void);

/**
 * \brief Writes a string to console.
 *
 * \param s String to write. Must be null-terminated.
 */
void
xme_hal_console_string(char *s);

/**
 * \brief Writes a string to console with maximum size.
 *
 * \param s String to write.
 * \param size Maximum size of string.
 */
void
xme_hal_console_stringn(char *s, uint16_t size);

/**
 * \brief Writes a single character to console.
 *
 * \param c Character to write.
 */
void
xme_hal_console_char(char c);

/**
 * \brief Writes a number to the console.
 *
 * \param nr Pointer to number.
 * \param nt The type of the number in memory.
 * \param bt Display type: binary, decimal or hexadecimal.
 */
void
xme_hal_console_number(
		void *nr,
		xme_hal_console_num_t nt,
		xme_hal_console_base_t bt
);

/**
 * \brief Set cursor position.
 *
 * \param x X position.
 * \param y Y position.
 */
void xme_hal_console_setPos(
		uint8_t x,
		uint8_t y
);

/** 
 * \def xme_hal_console_uint8_b( number )
 * \brief A macro that writes an unsigned int using 8 bits representation using binary base.
 */
#define xme_hal_console_uint8_b( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT8, XME_HAL_CONSOLE_BASE2 )

/** 
 * \def xme_hal_console_uint8_d( number )
 * \brief A macro that writes an unsigned int using 8 bits representation using decimal base.
 */
#define xme_hal_console_uint8_d( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT8, XME_HAL_CONSOLE_BASE10 )

/** 
 * \def xme_hal_console_uint8_x( number )
 * \brief A macro that writes an unsigned int using 8 bits representation using hexadecimal base.
 */
#define xme_hal_console_uint8_x( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT8, XME_HAL_CONSOLE_BASE16 )

/** 
 * \def xme_hal_console_int8_b( number )
 * \brief A macro that writes a signed int using 8 bits representation using binary base.
 */
#define xme_hal_console_int8_b( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT8, XME_HAL_CONSOLE_BASE2 )

/** 
 * \def xme_hal_console_int8_d( number )
 * \brief A macro that writes a signed int using 8 bits representation using decimal base.
 */
#define xme_hal_console_int8_d( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT8, XME_HAL_CONSOLE_BASE10 )

/** 
 * \def xme_hal_console_int8_x( number )
 * \brief A macro that writes a signed int using 8 bits representation using hexadecimal base.
 */
#define xme_hal_console_int8_x( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT8, XME_HAL_CONSOLE_BASE16 )


/** 
 * \def xme_hal_console_uint16_b( number )
 * \brief A macro that writes an unsigned int using 16 bits representation using binary base.
 */
#define xme_hal_console_uint16_b( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT16, XME_HAL_CONSOLE_BASE2 )

/** 
 * \def xme_hal_console_uint16_d( number )
 * \brief A macro that writes an unsigned int using 16 bits representation using decimal base.
 */
#define xme_hal_console_uint16_d( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT16, XME_HAL_CONSOLE_BASE10 )

/** 
 * \def xme_hal_console_uint16_x( number )
 * \brief A macro that writes an unsigned int using 16 bits representation using hexadecimal base.
 */
#define xme_hal_console_uint16_x( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT16, XME_HAL_CONSOLE_BASE16 )

/** 
 * \def xme_hal_console_int16_b( number )
 * \brief A macro that writes a signed int using 16 bits representation using binary base.
 */
#define xme_hal_console_int16_b( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT16, XME_HAL_CONSOLE_BASE2 )

/** 
 * \def xme_hal_console_int16_d( number )
 * \brief A macro that writes a signed int using 16 bits representation using decimal base.
 */
#define xme_hal_console_int16_d( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT16, XME_HAL_CONSOLE_BASE10 )

/** 
 * \def xme_hal_console_int16_x( number )
 * \brief A macro that writes a signed int using 16 bits representation using hexadecimal base.
 */
#define xme_hal_console_int16_x( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT16, XME_HAL_CONSOLE_BASE16 )


/** 
 * \def xme_hal_console_uint32_b( number )
 * \brief A macro that writes an unsigned int using 32 bits representation using binary base.
 */
#define xme_hal_console_uint32_b( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT32, XME_HAL_CONSOLE_BASE2 )

/** 
 * \def xme_hal_console_uint32_d( number )
 * \brief A macro that writes an unsigned int using 32 bits representation using decimal base.
 */
#define xme_hal_console_uint32_d( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT32, XME_HAL_CONSOLE_BASE10 )

/** 
 * \def xme_hal_console_uint32_x( number )
 * \brief A macro that writes an unsigned int using 32 bits representation using hexadecimal base.
 */
#define xme_hal_console_uint32_x( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_UINT32, XME_HAL_CONSOLE_BASE16 )

/** 
 * \def xme_hal_console_int32_b( number )
 * \brief A macro that writes a signed int using 32 bits representation using binary base.
 */
#define xme_hal_console_int32_b( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT32, XME_HAL_CONSOLE_BASE2 )

/** 
 * \def xme_hal_console_int32_d( number )
 * \brief A macro that writes a signed int using 32 bits representation using decimal base.
 */
#define xme_hal_console_int32_d( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT32, XME_HAL_CONSOLE_BASE10 )

/** 
 * \def xme_hal_console_int32_x( number )
 * \brief A macro that writes a signed int in 32 bits representation using hexadecimal base.
 */
#define xme_hal_console_int32_x( number ) xme_hal_console_number( (void *)&number, XME_HAL_CONSOLE_NUM_INT32, XME_HAL_CONSOLE_BASE16 )

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_CONSOLE_H
