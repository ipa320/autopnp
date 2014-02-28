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
 * $Id: deMarshalerDataHandlerMock.c 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         A mock of xme_core_dataHandler, to be used with the marshaler
 *         and demarshaler waypoint tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "deMarshalerTestTopic.h"
#include "deMarshalerTestTopicData.h"

#include "xme/hal/include/mem.h"

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include "xme/defines.h"

#include <stdbool.h>

XME_EXTERN_C_BEGIN

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
size_t
xme_wp_dataHandlerMock_getDemarshaledDataSize(void);

size_t
xme_wp_dataHandlerMock_getMarshaledDataSize(void);

size_t
xme_wp_dataHandlerMock_getMarshaledAttributeDataSize
(
    xme_core_attribute_key_t key
);

size_t
xme_wp_dataHandlerMock_getDemarshaledAttributeDataSize
(
    xme_core_attribute_key_t key
);

/**
 * \brief  Used instead of the original dataHandler function.
 *
 * \param port For value 1 xme_wp_dataHandlerMock_demarshaledData is read, else
 *        xme_wp_dataHandlerMock_marshaledData is read.
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
);

/**
 * \brief  Used instead of the original dataHandler function.
 *
 * \param port For 1 data is written to xme_wp_dataHandlerMock_marshaledData, else to
 *        xme_wp_dataHandlerMock_demarshaledData
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
);

/**
 * \brief  Used instead of the original dataHandler function.
 *
 * \detail Port value | key | location from which data is read
 *         0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute0
 *         0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute1
 *         1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_marshaledAttribute_attribute0
 *         1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_marshaledAttribute_attribute1
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
);

/**
 * \brief  Used instead of the original dataHandler function.
 *
 * \detail Port value | key | location to which data is writen
 *         0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_marshaledAttribute_attribute0
 *         0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_marshaledAttribute_attribute1
 *         1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute0
 *         1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute1
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void const * const buffer,
    unsigned int bufferSize
);

xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t port
);

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
);

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
/**
 * \brief  Test instance of demarshaled topic data. 
 *         Used for comparing/storing the results of the marshaler/demarshaler.
 */
xme_wp_deMarshalerTest_topic_test_t 
xme_wp_dataHandlerMock_demarshaledData =
{
    SFINIT(flag, true),
    SFINIT(uint8, 0xAB),
    SFINIT(uint16, 0xABCD),
    SFINIT(uint32, 0xABCD1234),
    SFINIT(uint64, 0xAABBCCDDEEFF0011),
    SFINIT(int8, (int8_t)0xAB),
    SFINIT(int16, (int16_t)0xABCD),
    SFINIT(int32, (int32_t)0xABCD1234),
    SFINIT(int64, (int64_t)0xAABBCCDDEEFF0011),
    SFINIT(f, 1.2f),
    SFINIT(d, 123.456),
    SFINIT(c, 'a'),
    SFINIT(e, 0x7FFFFFFF),
    SFINIT(array0, {0xABCD, 0x1234, 0x0F0F}),
    SFINIT(array1, {{0xAABB, 0xCCDD, 0xEEFF}, {0xABCD, 0x1234, 0x0F0F}}),
    SFINIT(subStruct, {
        SFINIT(uint16, 0xABCD),
        SFINIT(a, {0xABCD, 0x1234, 0x0F0F}),
        SFINIT(subSubStruct, {
            SFINIT(flag0, true),
            SFINIT(flag1, false)
        })
    }),
    SFINIT(array2,
    {
        {
            SFINIT(flag0, true),
            SFINIT(flag1, false)
        },
        {
            SFINIT(flag0, true),
            SFINIT(flag1, false)
        }
    }),
    SFINIT(array3,
    {
        {
            SFINIT(x, {0xAB, 0xCD}),
            SFINIT(y, 
            { 
                { 
                    SFINIT(c, {0x01, 0x02}) 
                },
                { 
                    SFINIT(c, {0x01, 0x02}) 
                }
            }),
            SFINIT(z, 
            { 
                { 
                    SFINIT(c, {0x01, 0x02}) 
                },
                { 
                    SFINIT(c, {0x01, 0x02}) 
                }
            })
        }
    })
};

/**
 * \brief  Test instance of marshaled topic data (network byte order!). 
 *         Used for comparing/storing the results of the demarshaler/marshaler.
 */
uint8_t 
xme_wp_dataHandlerMock_marshaledData[] =
{
    0x01, // flag
    0xAB, // uint8
    0xAB, 0xCD, // uint16
    0xAB, 0xCD, 0x12, 0x34, // uint32
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, // uint64
    0xAB, // int8
    0xAB, 0xCD, // int16
    0xAB, 0xCD, 0x12, 0x34, // int32
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, // int64
    0x3F, 0x99, 0x99, 0x9A, // f
    0x40, 0x5E, 0xDD, 0x2F, 0x1A, 0x9F, 0xBE, 0x77, // d
    0x61, // c
    0x7F, 0xFF, 0xFF, 0xFF, // e
    0xAB, 0xCD, // array0[0] 
    0x12, 0x34, // array0[1]
    0x0F, 0x0F, // array0[2]
    0xAA, 0xBB, // array1[0][0] 
    0xCC, 0xDD, // array1[0][1]
    0xEE, 0xFF, // array1[0][2]
    0xAB, 0xCD, // array1[1][0]
    0x12, 0x34, // array1[1][1]
    0x0F, 0x0F, // array1[1][2]
    0xAB, 0xCD, // subStruct.int16
    0xAB, 0xCD, 0x12, 0x34, 0x0F, 0x0F, // subStruct.a
    0x01, // subStruct.subSubStruct.flag0
    0x00, // subStruct.subSubStruct.flag1
    0x01, // array2[0].flag0
    0x00, // array2[0].flag1
    0x01, // array2[1].flag0
    0x00, // array2[1].flag1
    0xAB, // array3[0].x[0]
    0xCD, // array3[0].x[1]
    0x01, // array3[0].y[0].c[0]
    0x02, // array3[0].y[0].c[1]
    0x01, // array3[0].y[1].c[0]
    0x02, // array3[0].y[1].c[1]
    0x01, // array3[0].z[0].c[0]
    0x02, // array3[0].z[0].c[1]
    0x01, // array3[0].z[1].c[0]
    0x02, // array3[0].z[1].c[1]
};

/**
 * \brief  Test instance of demarshaled attribute data. 
 *         Used for comparing/storing the results of the marshaler/demarshaler.
 */
xme_wp_deMarshalerTest_topic_test_attribute_attribute0_t
xme_wp_dataHandlerMock_demarshaledAttribute_attribute0 =
0xABCD;

/**
 * \brief  Test instance of marshaled attribute data. 
 *         Used for comparing/storing the results of the demarshaler/marshaler.
 */
uint8_t 
xme_wp_dataHandlerMock_marshaledAttribute_attribute0[] =
{
    0xAB, 0xCD
};

/**
 * \brief  Test instance of demarshaled attribute data. 
 *         Used for comparing/storing the results of the marshaler/demarshaler.
 */
xme_wp_deMarshalerTest_topic_test_attribute_attribute1_t
xme_wp_dataHandlerMock_demarshaledAttribute_attribute1 =
{
    SFINIT(flag0, true),
    SFINIT(flag1, false)
};

/**
 * \brief  Test instance of marshaled attribute data. 
 *         Used for comparing/storing the results of the demarshaler/marshaler.
 */
uint8_t 
xme_wp_dataHandlerMock_marshaledAttribute_attribute1[] =
{
    0x01, // flag0
    0x00, // flag1
};

/**
 * \brief  Counts how often completeWriteOperation has been called.
 */
uint32_t xme_wp_dataHandlerMock_completeWriteOperationCallCount = 0;

/**
 * \brief  Counts how often completeReadOperation has been called.
 */
uint32_t xme_wp_dataHandlerMock_completeReadOperationCallCount = 0;

/**
 * \brief  Will be set to true, when a port is written after completeWriteOperation
 *         has been called.
 */
bool xme_wp_dataHandlerMock_writePortAfterCompleteWriteOperation = false;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
size_t
xme_wp_dataHandlerMock_getDemarshaledDataSize(void)
{
    // Note that the size of data structures might be different when using the
    // C or the C++ compiler, therefore we do not determine the size inside of
    // the C++ test class.
    return sizeof(xme_wp_dataHandlerMock_demarshaledData);
}

size_t
xme_wp_dataHandlerMock_getMarshaledDataSize(void)
{
    return sizeof(xme_wp_dataHandlerMock_marshaledData);
}

size_t
xme_wp_dataHandlerMock_getMarshaledAttributeDataSize
(
    xme_core_attribute_key_t key
)
{
    if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0) == key)
    {
        return sizeof(xme_wp_dataHandlerMock_marshaledAttribute_attribute0);
    }
    else if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1) == key)
    {
        return sizeof(xme_wp_dataHandlerMock_marshaledAttribute_attribute1);
    }

    return 0;
}

size_t
xme_wp_dataHandlerMock_getDemarshaledAttributeDataSize
(
    xme_core_attribute_key_t key
)
{
    if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0) == key)
    {
        return sizeof(xme_wp_dataHandlerMock_demarshaledAttribute_attribute0);
    }
    else if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1) == key)
    {
        return sizeof(xme_wp_dataHandlerMock_demarshaledAttribute_attribute1);
    }

    return 0;
}

/**
 * \brief  Used instead of the original dataHandler function.
 *         
 * \param  port Consult the following table for a mapping of port to read data
 *              port | data
 *              XME_CORE_DATAMANAGER_DATAPACKETID_MAX | No data read, bytesRead == bufferSize
 *              0 | xme_wp_dataHandlerMock_demarshaledData
 *              everything else | xme_wp_dataHandlerMock_marshaledData
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
)
{
    XME_UNUSED_PARAMETER(bufferSize);

    if (XME_CORE_DATAMANAGER_DATAPACKETID_MAX == port)
    {
        *bytesRead = bufferSize;
    }
    else if (0 == port)
    {
        xme_hal_mem_copy(buffer, &xme_wp_dataHandlerMock_demarshaledData, sizeof(xme_wp_deMarshalerTest_topic_test_t));
        *bytesRead = sizeof(xme_wp_deMarshalerTest_topic_test_t);
    } 
    else
    {
        xme_hal_mem_copy(buffer, &xme_wp_dataHandlerMock_marshaledData, sizeof(xme_wp_dataHandlerMock_marshaledData));
        *bytesRead = sizeof(xme_wp_dataHandlerMock_marshaledData);
    }

    return XME_STATUS_SUCCESS;
}

/**
 * \brief  Used instead of the original dataHandler function.
 *
 * \param  port Consult the following table for a mapping of port to written data
 *              port | data
 *              XME_CORE_DATAMANAGER_DATAPACKETID_MAX | No data written
 *              0 | xme_wp_dataHandlerMock_marshaledData
 *              everything else | xme_wp_dataHandlerMock_demarshaledData
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
)
{
    if (XME_CORE_DATAMANAGER_DATAPACKETID_MAX == port)
    {
        // Do nothing
    }
    else if (0 == port)
    {
        xme_hal_mem_copy(&xme_wp_dataHandlerMock_marshaledData, buffer, bufferSize);
    } 
    else
    {
        xme_hal_mem_copy(&xme_wp_dataHandlerMock_demarshaledData, buffer, bufferSize);
    }

    return XME_STATUS_SUCCESS;
}

/**
 * \brief  Used instead of the original dataHandler function.
 *         
 * \details Port value | key | location from which data is read
 *          0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute0
 *          0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute1
 *          1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_marshaledAttribute_attribute0
 *          1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_marshaledAttribute_attribute1
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
)
{
    XME_UNUSED_PARAMETER(bufferSize);

    if (port == 0)
    {
        if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0) == attributeKey)
        {
            xme_hal_mem_copy(buffer, &xme_wp_dataHandlerMock_demarshaledAttribute_attribute0, sizeof(xme_wp_dataHandlerMock_demarshaledAttribute_attribute0));
            *bytesRead = sizeof(xme_wp_dataHandlerMock_demarshaledAttribute_attribute0);
        }
        else if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1) == attributeKey)
        {
            xme_hal_mem_copy(buffer, &xme_wp_dataHandlerMock_demarshaledAttribute_attribute1, sizeof(xme_wp_dataHandlerMock_demarshaledAttribute_attribute1));
            *bytesRead = sizeof(xme_wp_dataHandlerMock_demarshaledAttribute_attribute1);
        }
    }
    else
    {
        if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0) == attributeKey)
        {
            xme_hal_mem_copy(buffer, &xme_wp_dataHandlerMock_marshaledAttribute_attribute0, sizeof(xme_wp_dataHandlerMock_marshaledAttribute_attribute0));
                            *bytesRead = sizeof(xme_wp_dataHandlerMock_marshaledAttribute_attribute0);
        }
        else if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1) == attributeKey)
        {
            xme_hal_mem_copy(buffer, &xme_wp_dataHandlerMock_marshaledAttribute_attribute1, sizeof(xme_wp_dataHandlerMock_marshaledAttribute_attribute1));
                            *bytesRead = sizeof(xme_wp_dataHandlerMock_marshaledAttribute_attribute1);
        }
    }

    return XME_STATUS_SUCCESS;
}

/**
 * \brief  Used instead of the original dataHandler function.
 *         
 * \detail  Port value | key | location to which data is writen
 *          0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_marshaledAttribute_attribute0
 *          0 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_marshaledAttribute_attribute1
 *          1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute0
 *          1 | XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1 | xme_wp_dataHandlerMock_demarshaledAttribute_attribute1
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void const * const buffer,
    unsigned int bufferSize
)
{
    if (xme_wp_dataHandlerMock_completeWriteOperationCallCount > 0) {
        xme_wp_dataHandlerMock_writePortAfterCompleteWriteOperation = true;
    }

    if (port == 0)
    {
        if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0) == attributeKey)
        {
            xme_hal_mem_copy(&xme_wp_dataHandlerMock_marshaledAttribute_attribute0, buffer, bufferSize);
        }
        else if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1) == attributeKey)
        {
            xme_hal_mem_copy(&xme_wp_dataHandlerMock_marshaledAttribute_attribute1, buffer, bufferSize);
        }
    } 
    else
    {
        if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0) == attributeKey)
        {
            xme_hal_mem_copy(&xme_wp_dataHandlerMock_demarshaledAttribute_attribute0, buffer, bufferSize);
        }
        else if (XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1) == attributeKey)
        {
            xme_hal_mem_copy(&xme_wp_dataHandlerMock_demarshaledAttribute_attribute1, buffer, bufferSize);
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_UNUSED_PARAMETER(port);

    xme_wp_dataHandlerMock_completeWriteOperationCallCount++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_UNUSED_PARAMETER(port);

    xme_wp_dataHandlerMock_completeReadOperationCallCount++;

    return XME_STATUS_SUCCESS;
}

XME_EXTERN_C_END
