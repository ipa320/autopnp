/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: console_arch.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Console abstraction (architecture specific part: implementation for
 *         embedded uC boards).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/console.h"

#include "xme/hal/include/display.h"

/******************************************************************************/
/***   Prototypes                                                     		***/
/******************************************************************************/

// Clears the buffer. All characters in the buffer will become ' ' (space).
void xme_hal_console_clear_buffer();

// Writes the whole buffer onto the screen.
void xme_hal_console_write_buffer();

// Logic to calculate the position of the next character.
void xme_hal_console_next_char();

// Logic for newline command.
void xme_hal_console_newline();

// Draws a single char at a specific position.
void xme_hal_console_draw_char_at_pos(unsigned char c, uint16_t x, uint16_t y);

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

// Storage for characters
unsigned char xme_hal_console_buffer[XME_HAL_CONSOLE_CHAR_COUNT_Y][XME_HAL_CONSOLE_CHAR_COUNT_X];
uint16_t xme_hal_console_pos_x;
uint16_t xme_hal_console_pos_y;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief  Characters to use for formatting numbers.
 */
static const char xme_hal_console_digits[] = "0123456789ABCDEF";

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_hal_console_clear_buffer()
{
	uint16_t x,y;

	for (y=0; y<XME_HAL_CONSOLE_CHAR_COUNT_Y; y++)
	{
		for (x=0; x<XME_HAL_CONSOLE_CHAR_COUNT_X; x++)
			xme_hal_console_buffer[y][x] = ' ';
	}
}

void
xme_hal_console_write_buffer()
{
	uint16_t x,y;

	for (y=0; y<XME_HAL_CONSOLE_CHAR_COUNT_Y; y++)
	{
		for (x=0; x<XME_HAL_CONSOLE_CHAR_COUNT_X; x++)
			xme_hal_console_draw_char_at_pos( xme_hal_console_buffer[y][x], x, y );
	}
}

void
xme_hal_console_draw_char_at_pos(unsigned char c, uint16_t x, uint16_t y)
{
	xme_hal_console_buffer[y][x] = c;

	xme_hal_display_char( c,
			XME_HAL_CONSOLE_CHAR_OFFSET_X + XME_HAL_CHAR_SIZE_X * x,
			XME_HAL_CONSOLE_CHAR_OFFSET_Y + XME_HAL_CHAR_SIZE_Y * y);
}

void
xme_hal_console_newline()
{
	uint16_t i;

	for (i=xme_hal_console_pos_x; i<XME_HAL_CONSOLE_CHAR_COUNT_X; i++)
	{
		xme_hal_console_draw_char_at_pos(' ', i, xme_hal_console_pos_y);
	}

	xme_hal_console_pos_x = 0;
	xme_hal_console_pos_y++;

	if ( xme_hal_console_pos_y >= XME_HAL_CONSOLE_CHAR_COUNT_Y )
	{
		uint16_t x,y;

		xme_hal_console_pos_y = XME_HAL_CONSOLE_CHAR_COUNT_Y - 1;

		for (y=0; y<XME_HAL_CONSOLE_CHAR_COUNT_Y-1; y++)
		{
			for (x=0; x<XME_HAL_CONSOLE_CHAR_COUNT_X; x++)
			{
				xme_hal_console_buffer[y][x] = xme_hal_console_buffer[y+1][x];
			}
		}

		for (x=0; x<XME_HAL_CONSOLE_CHAR_COUNT_X; x++)
		{
			xme_hal_console_buffer[XME_HAL_CONSOLE_CHAR_COUNT_Y-1][x] = ' ';
		}

		xme_hal_console_write_buffer();
	}
}

void
xme_hal_console_next_char()
{
	xme_hal_console_pos_x++;

	if ( xme_hal_console_pos_x >= XME_HAL_CONSOLE_CHAR_COUNT_X )
	{
		xme_hal_console_newline();
	}
}

xme_status_t
xme_hal_console_init(void)
{
	xme_hal_display_init();
	xme_hal_display_on();
	uprintf_init(xme_hal_console_char);

	return XME_STATUS_SUCCESS;
}

void
xme_hal_console_fini(void)
{
	// Nothing to do
}

void
xme_hal_console_reset(void)
{
	xme_hal_console_pos_x = 0;
	xme_hal_console_pos_y = 0;
	xme_hal_console_clear_buffer();
	xme_hal_console_write_buffer();
}

void
xme_hal_console_string(char *s)
{
	while (*s)
		xme_hal_console_char( *s++ );
}

void
xme_hal_console_stringn(char *s, uint16_t size)
{
	while (size-- && *s)
		xme_hal_console_char( *s++ );
}

void
xme_hal_console_char(char c)
{
	switch (c)
	{
	case '\n':
		xme_hal_console_newline();
		break;
	default:
		xme_hal_console_draw_char_at_pos(c, xme_hal_console_pos_x, xme_hal_console_pos_y);
		xme_hal_console_next_char();
		break;
	}
}

void xme_hal_console_setPos(uint8_t x, uint8_t y)
{
	xme_hal_console_pos_x = x;
	xme_hal_console_pos_y = y;
}

void
xme_hal_console_number(
		void *nr,
		xme_hal_console_num_t nt,
		xme_hal_console_base_t bt
)
{
	unsigned char buf[33];
	uint8_t position = 0;
	uint8_t minus_sign = 0;

	uint32_t unumber;
	int32_t number;

	switch (nt)
	{
	case XME_HAL_CONSOLE_NUM_UINT8:
		unumber = *((uint8_t *)nr);
		break;
	case XME_HAL_CONSOLE_NUM_UINT16:
		unumber = *((uint16_t *)nr);
		break;
	case XME_HAL_CONSOLE_NUM_UINT32:
		unumber = *((uint32_t *)nr);
		break;
	case XME_HAL_CONSOLE_NUM_INT8:
		number = *((int8_t *)nr);
		if ( number < 0 )
		{
			minus_sign = 1;
			unumber = -number;
		} else {
			unumber = number;
		}
		break;
	case XME_HAL_CONSOLE_NUM_INT16:
		number = *((int16_t *)nr);
		if ( number < 0 )
		{
			minus_sign = 1;
			unumber = -number;
		} else {
			unumber = number;
		}
		break;
	case XME_HAL_CONSOLE_NUM_INT32:
		number = *((int32_t *)nr);
		if ( number < 0 )
		{
			minus_sign = 1;
			unumber = -number;
		} else {
			unumber = number;
		}
		break;
	default:
		break;
	}

	if (!unumber)
	{
		buf[position++] = '0';
	}

	while (unumber)
	{
		buf[position++] = xme_hal_console_digits[(unumber % bt)];
		unumber /= bt;
	}

	if (minus_sign)
	{
		buf[position++] = '-';
	}

	while (position)
	{
		xme_hal_console_char(
				buf[--position]
		);
	}
}
