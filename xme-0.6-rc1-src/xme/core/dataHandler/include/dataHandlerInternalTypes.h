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
 * $Id: dataHandlerInternalTypes.h 5059 2013-09-12 14:29:40Z camek $
 */

/**
 * \file
 *         Data Handler Internal Types.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLERINTERNALTYPES_H
#define XME_CORE_DATAHANDLER_DATAHANDLERINTERNALTYPES_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/defines.h"
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include <stdbool.h>
#include <stdint.h>

//******************************************************************************//
//***   Defines                                                              ***//
//******************************************************************************//
//#define USE_QUEUE

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

/**
 * \enum dataHandler_states_t
 *
 * \brief  Enumerates the data handler states.
 */
typedef enum xme_core_dataHandler_states {
    DATAHANDLER_STATE_INVALID = 0, ///< invalid state. 

    DATAHANDLER_STATE_READ_AUDITELEMENT, ///< read audit element state. 
    DATAHANDLER_STATE_WRITE_AUDITELEMENT, ///< write audit element state. 
    DATAHANDLER_STATE_READ_SECURITYATTRIBUTE, ///< read security attribute state. 
    DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE, ///< write security attribute state. 
    DATAHANDLER_STATE_READ_SAFETYATTRIBUTE, ///< read safety attribute state. 
    DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE, ///< write safety attribute state. 

    DATAHANDLER_STATE_READ_SHADOWELEMENT, ///< read shadow element state. 
    DATAHANDLER_STATE_WRITE_SHADOWELEMENT, ///< write shadow element state. 
    DATAHANDLER_STATE_READ_SHADOWATTRIBUTE, ///< read shadow attribute state. 
    DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE, ///< write shadow attribute state. 
    DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET, ///< read shadow element state.
    DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET, ///< write shadow element state.
    DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET, ///< read shadow attribute state.
    DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET, ///< write shadow attribute state.

    DATAHANDLER_STATE_READ_TOPICELEMENT, ///< read topic element state. 
    DATAHANDLER_STATE_WRITE_TOPICELEMENT, ///< write topic element state. 
    DATAHANDLER_STATE_READ_TOPICATTRIBUTE, ///< read topic attribute state. 
    DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE ///< write topic attritube state.
} xme_core_dataHandler_states_t;

typedef enum xme_core_dataHandler_safetyClassification {
    DATAHANDLER_SIL_INVALID = 0, ///< invalid safty level.
    DATAHANDLER_SIL_NONE, ///< QM
    DATAHANDLER_SIL_LOW, ///< matches ASIL A and SIL 1
    DATAHANDLER_SIL_MEDIUM, ///< matches ASIL B and SIL 2
    DATAHANDLER_SIL_HIGH,///< matches ASIL C and SIL 3
    DATAHANDLER_SIL_EXTRA,///< matches ASIL C and SIL 4
} xme_core_dataHandler_safetyClassification_t;

/**
 * \struct dataHandler_safetyAttributes_t
 *
 * \brief  Defines the structure for safezy attributes.
 */
typedef struct xme_core_dataHandle_safetyAttributes {
        xme_core_dataHandler_safetyClassification_t classifier; ///< safety classifier
} xme_core_dataHandler_safetyAttributes_t;

/**
 * \struct dataHandler_securityClassification_t
 *
 * \brief  Enumerates the security classification.
 */
typedef enum xme_core_dataHandler_securityClassification {
    DATAHANDLER_SA_INVALID = 0, ///< invalid security level.
    DATAHANDLER_SA_SYSTEM, ///< security at system level. 
    DATAHANDLER_SA_HIGH, ///< high security level. 
    DATAHANDLER_SA_MEDIUM, ///< medium security level. 
    DATAHANDLER_SA_LOW ///< low security level. 
} xme_core_dataHandler_securityClassification_t;

/**
 * \struct dataHandler_securityAttributes_t
 *
 * \brief  Structure for security attributes.
 */
typedef struct xme_core_dataHandle_securityAttributes {
        xme_core_dataHandler_securityClassification_t classifier; ///< security classifier
} xme_core_dataHandler_securityAttributes_t;

/**
 * \struct xme_core_dataHandler_dataHandle_t
 *
 * \brief  Defines the type for the database structure.
 */
typedef struct xme_core_dataHandler_dataElement {
    uint32_t checksum; ///< contains the checksum value of this data element
    size_t size; ///< the size of data element. 
    void * data; ///< the data element. 
} xme_core_dataHandler_dataElement_t;

/**
 * \struct xme_core_dataHandler_dataHandle_t
 *
 * \brief  Defines the type for the database structure.
 */
typedef struct xme_core_dataHandler_attributeElement {
    uint32_t checksum; ///< contains the checksum value of this attribute element
    xme_core_attribute_key_t key; ///< the attribute key. 
    size_t size; ///< the size of the attribute element.
    void * data; ///< the content of the attribute element. 
} xme_core_dataHandler_attributeElement_t;

/**
 * \struct dataHandler_dataPacket_t
 * \brief Structure for storing data packet information. 
 */
typedef struct xme_core_dataHandler_dataPacket {
    xme_core_dataManager_dataPacketId_t port; ///< data packet id. 
    xme_core_component_portType_t type; ///< data packet type. 
    xme_core_topic_t topic; ///< topic associated to the data packet. 

    uint16_t attributeElements; ///< number of attribute elements. 
    xme_core_dataHandler_dataElement_t dataElement; ///< the data element of the data packet.
    xme_core_dataHandler_attributeElement_t * attributeElement; ///< the attribute elements of the data packet.
} xme_core_dataHandler_dataPacket_t;

#if defined(USE_QUEUE)
/**
 *
 */
// ringbuffer with start <--> end
    // current -> current position to read from or write to
    // next -> next element to read from
    // last -> last of queue
typedef struct xme_core_dataHandler_dataQueue
{
    /* FIXME: added this one only for information part, but queue == data structure and element == infos,
     * so this should be done in a joint way.
     */
    xme_core_dataManager_dataPacketId_t port;
    xme_core_component_portType_t type;
    xme_core_topic_t topic;

    // organization of our queue
    void* current;
    void* last;
    void* start;
    void* end;

    // data elements
    uint16_t elements;
    xme_core_dataHandler_dataPacket_t * dataElement; // TOOD: rename it!
}xme_core_dataHandler_dataQueue_t;
#endif

/**
 * \struct dataHandler_shadowStructure_t
 * \brief contains the control information to allow a test system to read and write data of
 *        the original datahandle elements.
 */
typedef struct xme_core_dataHandler_shadowStructure {
    bool lockedByTestSystem;   ///< True, if data was written to the shadow-port from external (only used in SHADOW database). However, the entry maybe not be activated yet.
    bool useShadowEntry;    ///< True, if shadow-port is activated (only used in INTERNAL database).
} xme_core_dataHandler_shadowStructure_t;

/**
 * \struct dataHandler_infoStructure_t
 * \brief structure for storing the header information. 
 */
typedef struct xme_core_dataHandler_headerStructure {
    bool writeLock;   //!< True, if completeWrite was triggered and no new data shall be written to this element
    bool persistent;          //!< True, if data element should not be cleared after a complete read
    bool overwrite;        //!< True, if port data should be overwritten when new values arrive.
} xme_core_dataHandler_infoStructure_t;

/**
 * \struct dataHandler_dataStructure_t
 *
 * \brief   Defines the internal data structure of one database element.
 */
typedef struct xme_core_dataHandler_dataStructure {
        xme_core_dataHandler_infoStructure_t info; ///< the information of data structure.
        xme_core_dataHandler_shadowStructure_t shadowEntry; ///< the shadow entry.
    // maybe an own data structure to queue all elements
#if defined(USE_QUEUE)
        xme_core_dataHandler_dataQueue_t dataEntry;
#else
        xme_core_dataHandler_dataPacket_t dataEntry; ///< the data entry.
#endif
        xme_core_dataHandler_safetyAttributes_t  severityLevel;
        xme_core_dataHandler_securityAttributes_t clearance;
} xme_core_dataHandler_dataStructure_t;

/**
 * \struct dataHandler_buddySystem_t
 * \brief  structure for the buddy system. 
 */
typedef struct xme_core_dataHandler_buddySystem
{
  //int minimalBits;
  //int numSize;
  size_t amountOfMemory; ///< the amount of memory.
  size_t freeBytes; ///< the free bytes.
  //char* bitmap;
  //uint16_t inuse;
  size_t offset; ///< the offset.
  //void * freeList;
  void * baseAdress; ///< the base address. 
  void * maxAdress; ///< the maximum address.
} xme_core_dataHandler_buddySystem_t;

/**
 * \struct internals_t
 *
 * \brief   Defines the internal settings of the database.
 *
 * \details Description of flags:
 *  - useShadowEntries: True, if the whole shadow database is active
 *  - initFromFileStorage:
 *  - correctInitialized:
 *  - alreadyInitalized:
 *  - selftestDone: True, if self-test during startup has been finished, system is now in normal operation
 */
typedef struct xme_core_dataHandler_internal
{
    bool initFromFileStorage; ///< boolean variable stating that initialization is made from file storage. 
    bool initialized; ///< flag indicating that the data handler is correctly initialized.
    xme_core_component_t selfID; ///< data handlers component id. 
    xme_core_dataManager_dataPacketId_t dataPacketId; ///< data packet id, which qualifies the port number
    uintptr_t dataHandlerPorts[XME_CORE_DATAMANAGER_DATAPACKETID_MAX];
    uintptr_t shadowHandlerPorts[XME_CORE_DATAMANAGER_DATAPACKETID_MAX];
    xme_core_dataHandler_buddySystem_t memory; ///< memory associated to the system.
    xme_core_dataHandler_buddySystem_t memoryOfShadow; ///< memory associated to the system.
} xme_core_dataHandler_internals_t;

/**
 * \brief  Pointer to the database image file, which was written in a former run of our database.
 */
typedef void * xme_core_dataHandler_imageDataStructure_t;

/**
 * \brief  Type for phases.
 */
typedef xme_core_dataHandler_dataStructure_t xme_core_dataHandler_shadow_dataStructure_t;

/**
 * \struct dataHandlerSetup_t
 * \brief the data handler setup structure. 
 */
typedef struct xme_core_dataHandler_setup
{
    xme_core_component_t componentID; ///< the component id. 
    xme_core_component_portType_t type; ///< the port type. 
    xme_core_topic_t topic; ///< the topic. 
    uint32_t bufferSize; ///< the buffer size.
    xme_core_attribute_descriptor_list_t metadata; ///< a list with associated metadata. 
    uint32_t queueSize; ///< the queue size.
    bool overwrite; ///< flag indicating if data can be overwritten. 
    bool persistent; ///< flag indicating the persistence of data. 
    uint32_t historyDepth; ///< nubmer of stored historical data.
} xme_core_dataHandler_setup_t;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief describes internal states of the data handler.
 */
extern xme_core_dataHandler_internals_t xme_core_dataHandler_dataHandlerInternals;

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DATAHANDLER_DATAHANDLERINTERNALTYPES_H
