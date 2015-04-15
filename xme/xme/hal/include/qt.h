/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: qt.h 7671 2014-03-04 20:47:40Z kainz $
 */

/** 
 * \file
 * \brief Qt abstraction.
 *
 * Important: To use the Qt abstraction correctly it is required to add
 * find_package(Qt4 REQUIRED) in the protected region
 * APPLICATION_XXX_CMAKELISTS_TXT_ADDITIONAL_2 of the CMakeApplication.txt of
 * the respective node using the Qt abstraction.
 */

#ifndef XME_HAL_QT_H
#define XME_HAL_QT_H

/** 
 * \defgroup hal_Qt Qt.
 * @{
 *
 * \brief Qt abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"


/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \typedef xme_hal_qt_callback_t
 *
 * \brief Callback executed by GUI thread.
 */
typedef void (*xme_hal_qt_callback_t) (void* userData);

/**
 * \typedef xme_hal_qt_applicationHandle_t
 *
 * \brief Handle to Qt application.
 */
typedef void* xme_hal_qt_applicationHandle_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the Qt system for this node, if not already done.
 *
 * \retval XME_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_STATUS_INTERNAL_ERROR if something went wrong.
 */
xme_status_t
xme_hal_qt_init(void);

/**
 * \brief Triggers execution of callback inside Qt GUI thread.
 *
 * \param[in] callback Callback which shall be called.
 * \param[in] userData Parameter provided to callback upon calling.
 */
void
xme_hal_qt_triggerExecution
(
    xme_hal_qt_callback_t callback,
    void* userData
);

/**
 * \brief  Get Qt application object.
 *
 * \return Return handle to Qt application.
 */
xme_hal_qt_applicationHandle_t
xme_hal_qt_getApplication(void);

/**
 * \brief Finalizes the Qt system for this node, if no one else is using Qt
 *        anymore.
 */
void
xme_hal_qt_fini(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_QT_H
