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
 * $Id: can.h 2693 2013-03-17 01:51:56Z camek $
 */

/** 
 * \file
 * \brief CAN transceiver abstraction.
 */

#ifndef XME_HAL_CAN_H
#define XME_HAL_CAN_H

/**
 * \defgroup hal_can CAN transceiver.
 * @{
 * 
 * \brief CAN transceiver abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/can_arch.h"

#include <stdint.h>
/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/** 
 * \enum xme_hal_can_port_t
 * \brief References different CAN ports on a device.
 */
typedef enum
{
	XME_HAL_CAN_PORT_0 = 0, ///< Refers to CAN port 0.
	XME_HAL_CAN_PORT_1 = 1, ///< Refers to CAN port 1.
	XME_HAL_CAN_PORT_2 = 2, ///< Refers to CAN port 2.
	XME_HAL_CAN_PORT_3 = 3, ///< Refers to CAN port 3.
	XME_HAL_CAN_PORT_4 = 4  ///< Refers to CAN port 4.
} xme_hal_can_port_t;

/** 
 * \enum xme_hal_can_speed_t
 * \brief Describes different CAN speed configurations.
 */
 typedef enum
{
	XME_HAL_CAN_SPEED_125k, ///< Refers to CAN speed of 125kbps.
	XME_HAL_CAN_SPEED_250k, ///< Refers to CAN speed of 250kbps.
	XME_HAL_CAN_SPEED_500k, ///< Refers to CAN speed of 500kbps. 
	XME_HAL_CAN_SPEED_1M, ///< Refers to CAN speed of 1Mbps.
} xme_hal_can_speed_t;

/** 
 * \enum xme_hal_can_event_t
 * \brief Describes CAN event used to describe the cause of a callback.
 */
typedef enum
{
	XME_HAL_CAN_EVENT_MESSAGE_RECEIVED, ///< New message available.
	XME_HAL_CAN_EVENT_MESSAGE_TRANSMITTED, ///< Space for transmitting messages available.
	XME_HAL_CAN_EVENT_ERROR ///< Error occured.
} xme_hal_can_event_t;

/** 
 * \enum xme_hal_can_error_t
 * \brief Describes different error states of CAN transceiver.
 */
typedef enum
{
	XME_HAL_CAN_ERROR_NONE, ///< No error.
	XME_HAL_CAN_ERROR_GENERIC, ///< Generic error.
	XME_HAL_CAN_ERROR_BUS_TERMINATION, ///< Problem with bus termination.
	XME_HAL_CAN_ERROR_MESSAGE_LOST, ///< No ACK received for sent message.
	XME_HAL_CAN_ERROR_RX_BUFFER_OVERRUN ///< Could not store a CAN message because RX buffer was full.
} xme_hal_can_error_t;

/** 
 * \enum xme_hal_can_identifier_length
 * \brief Length of ID field.
 */
typedef enum
{
	XME_HAL_CAN_IDENTIFIER_LENGTH11, ///< ID field length of 11bit / CAN 2.0A.
	XME_HAL_CAN_IDENTIFIER_LENGTH29 ///< ID field length of 29bit / CAN 2.0B.
} xme_hal_can_identifier_length;

/** 
 * \enum xme_hal_can_state_rx_t
 * \brief Describes state of RX.
 */
typedef enum
{
	XME_HAL_CAN_STATE_RX_MESSAGE_AVAILABLE, ///< New message in buffer.
	XME_HAL_CAN_STATE_RX_IDLE ///< No message received.
} xme_hal_can_state_rx_t;

/** 
 * \enum xme_hal_can_state_tx_t
 * \brief Describes state of TX.
 */
typedef enum
{
	XME_HAL_CAN_STATE_TX_IDLE, ///< New message can be sent.
	XME_HAL_CAN_STATE_TX_BUSY ///< Buffer is full.
} xme_hal_can_state_tx_t;

/** 
 * \struct xme_hal_can_message_t
 * \brief Describes one message received or to be transmitted via CAN.
 */
typedef struct
{
	uint32_t msgID;  ///< Identifier of message. Might be 11 or 29 bit.
	uint8_t data[8]; ///< Payload.
	uint8_t msgLen;  ///< Number of valid bytes in payload.
} xme_hal_can_message_t;

/** 
 * \typedef *xme_hal_can_callback_t
 * \brief a pointer to a function having two parameters: port and event. 
 */
typedef void (*xme_hal_can_callback_t) (xme_hal_can_port_t port, xme_hal_can_event_t event);

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes a CAN transceiver.
 *
 * \param port Identification of CAN transceiver.
 * \param speed Selection of CAN speed.
 * \param callback Function to be called in case of state changes.
 *
 * \retval XME_CORE_STATUS_SUCCESS if CAN component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if CAN component cannot be initialized.
 */
xme_status_t
xme_hal_can_init(xme_hal_can_port_t port, xme_hal_can_speed_t speed, xme_hal_can_callback_t callback);

/**
 * \brief De-Initializes a CAN transceiver.
 *
 * \param port Identification of CAN transceiver.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation has been successfully de-initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if cannot de-initialize CAN port.
 */
xme_status_t
xme_hal_can_fini(xme_hal_can_port_t port);

/**
 * \brief Transfers one CAN message to the buffer.
 *
 * This function adds one message object to the transmit buffer.
 * If this function succeeds, it does not mean that the object
 * was actually successfully sent.
 *
 * \param port Identification of CAN transceiver.
 * \param message Message to send.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the message is sent using CAN port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if cannot send the message to CAN port.
 */
xme_status_t
xme_hal_can_sendMessage(xme_hal_can_port_t port, xme_hal_can_message_t message);

/**
 * \brief Gets one CAN message from the buffer.
 *
 * This functions gets one message from the receive buffer. There
 * might be several messages available in the receive buffer.
 *
 * \param port Identification of CAN transceiver.
 * \param message Pointer to message object to store information.
 *
 * \retval XME_CORE_STATUS_SUCCESS if a message was successfully received from CAN port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if cannot receive the message from CAN port.
 */
xme_status_t
xme_hal_can_getMessage(xme_hal_can_port_t port, xme_hal_can_message_t *message);

/**
 * \brief Returns the last error that happened.
 *
 * If several errors occur, only the newest one is buffered.
 *
 * \param port Identification of CAN transceiver.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the last error is received from CAN port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if cannot receive the last error from CAN port.
 */
xme_status_t
xme_hal_can_getLastError(xme_hal_can_port_t port);

/**
 * \brief Clears the last error that happened.
 *
 * \param port Identification of CAN transceiver.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the last error is cleared in CAN port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if cannot clear last error in CAN port.
 */
xme_status_t
xme_hal_can_clearLastError(xme_hal_can_port_t port);

/**
 * \brief Returns the state of the receiver.
 *
 * This function can be called to determine if a new message is available.
 * Several message objects might be buffered.
 *
 * \param port Identification of CAN transceiver.
 *
 * \retval XME_CORE_STATUS_SUCCESS the RX line status has ben obtained from CAN port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR cannot get RX line status from CAN port.
 */
xme_hal_can_state_rx_t
xme_hal_can_getStateRX(xme_hal_can_port_t port);

/**
 * \brief Returns the state of the transceiver.
 *
 * It can be determined with this function if a new message
 * object might be added to the transmit buffer.
 *
 * \param port Identification of CAN transceiver.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the TX line state has ben obtained from CAN port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if cannot get TX line state from CAN port.
 */
xme_hal_can_state_tx_t
xme_hal_can_getStateTX(xme_hal_can_port_t port);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_CAN_H
