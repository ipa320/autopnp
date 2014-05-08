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
 * $Id: display.h 2693 2013-03-17 01:51:56Z camek $
 */

/**
 * \file
 * \brief Generic header for display abstraction.
 */

#ifndef XME_HAL_DISPLAY_H
#define XME_HAL_DISPLAY_H

/**
 * \defgroup hal_display Display
 * @{
 *
 * \brief  Universal display abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \enum xme_hal_display_bitmap_t
 * \brief Bitmap type.
 */
typedef enum
{
	XME_HAL_DISPLAY_BITMAP_INVALID = 0,         ///< invalid display bitmap.
	XME_HAL_DISPLAY_BITMAP_GREYSCALE_8BIT = 1,  ///< bitmap display to 8-bits greyscale.
	XME_HAL_DISPLAY_BITMAP_UNDEFINED = 255      ///< undefined display bitmap.
}
xme_hal_display_bitmap_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initialize connection between uC and display. This function has to be
 *        called once before any other display function can be used.
 */
void
xme_hal_display_init(void);

/**
 * \brief Switch on the display.
 */
void
xme_hal_display_on(void);

/**
 * \brief Switch off the display.
 */
void
xme_hal_display_off(void);

/**
 * \brief Clears the display.
 */
void
xme_hal_display_clear(void);

/**
 * \brief Display a single char at a specific position on the display.
 *
 * \param c     character to display.
 * \param pos_x x-position of character in pixels.
 * \param pos_y y-position of character in pixels.
 */
void
xme_hal_display_char(
	char c,
	uint16_t pos_x,
	uint16_t pos_y
);

/**
 * \brief Display a bitmap at a specific position on the display.
 *
 * \param d      Pointer to raw data of bitmap. Depends on type.
 * \param pos_x  x-position of bitmap in pixels.
 * \param pos_y  y-position of bitmap in pixels.
 * \param size_x Size in x dimension of bitmap.
 * \param size_y Size in y dimension of bitmap.
 * \param t      Type of bitmap.
 */
void
xme_hal_display_bitmap
(
	const char *d,
	uint16_t pos_x,
	uint16_t pos_y,
	uint16_t size_x,
	uint16_t size_y,
	xme_hal_display_bitmap_t t
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_DISPLAY_H
