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
 * $Id: manifestRepository.h 6183 2013-12-19 17:11:05Z wiesmueller $
 */

/**
 * \file
 *         Manifest Repository.
 */

#ifndef XME_CORE_MANIFESTREPOSITORY_H
#define XME_CORE_MANIFESTREPOSITORY_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/node.h"
#include "xme/core/manifestTypes.h"

#include "xme/xme_opt.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_manifestRepository_iterator_t
 *
 * \brief  Iterator handle for Manifest Repository.
 */
typedef xme_hal_table_rowHandle_t xme_core_manifestRepository_iterator_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the manifest repository component.
 *         Exactly one component of this type must be present on every node.
 *
 * \retval XME_SUCCESS if the manifest repository component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if manifest repository component initialization failed.
 */ 
xme_status_t
xme_core_manifestRepository_init(void);

/**
 * \brief  Frees all resources occupied by the manifest repository component.
 *         Exactly one component of this type must be present on every node.
 */
void
xme_core_manifestRepository_fini(void);

/**
 * \brief  Retrieves the manifest corresponding to the given component type.
 *
 * \param[in] componentType Component type specified in the manifest.
 * \param[out] outComponentManifest The output component manifest matching component id.
 *
 * \retval XME_STATUS_SUCCESS if the component manifest is stored in the manifest
 *         repository, and there is a matching with the component type. 
 * \retval XME_STATUS_NOT_FOUND if the component type was not found in manifest repository.
 * \retval XME_STATUS_INVALID_PARAMETER if componentType is XME_CORE_COMPONENT_TYPE_INVALID.
 */
xme_status_t
xme_core_manifestRepository_findComponentManifest
(
    xme_core_componentType_t componentType,
    xme_core_componentManifest_t* outComponentManifest
);

/**
 * \brief Adds a new component manifest to the repository.
 *
 * \param[in] componentManifest Component manifest to be stored. It will be copied into
 *            the repository.
 * \param[in] replace Determines behavior when there already exists a manifest in the
 *            repository with the same component type as the added one. When replace is 
 *            true, the existing manifest will be replaced, otherwise the addition will
 *            fail (see return values).
 *
 * \retval XME_STATUS_SUCCESS if the component manifest has been added to the repository.
 * \retval XME_STATUS_INTERNAL_ERROR if the manifest cannot be added to the repository.
 * \retval XME_STATUS_UNSUPPORTED if the manifest type is not supported by the repository.
 * \retval XME_STATUS_INVALID_PARAMETER if input parameters are incorrect.
 * \retval XME_STATUS_ALREADY_EXIST if replace is false, and there is an existing
 *         manifest for the component type already in the repository.
 */
xme_status_t
xme_core_manifestRepository_addComponentManifest
(
    const xme_core_componentManifest_t* componentManifest,
    bool replace
);

/**
 * \brief Removes the manifest for the given component id from the repository.
 *
 * \param[in] componentType Type of component to remove.
 *
 * \retval XME_STATUS_SUCCESS if the manifest has been successfully removed to the repository.
 * \retval XME_STATUS_INTERNAL_ERROR if the manifest cannot be removed from the repository.
 * \retval XME_STATUS_INVALID_PARAMETER if input parameter is not a valid component id.
 * \retval XME_STATUS_NOT_FOUND if there are no manifests associated to that component type.
 */
xme_status_t
xme_core_manifestRepository_removeComponentManifest
(
    xme_core_componentType_t componentType
);

/**
 * \brief  Creates a new Manifest Repository iterator.
 *
 * \return Returns new Manifest Repository iterator handle.
 */
xme_core_manifestRepository_iterator_t
xme_core_manifestRepository_initIterator(void);

/**
 * \brief  Determines if there are more manifests to iterate over.
 *
 * \details This function can be used as a loop invariant. Use
 *          xme_core_manifestRepository_next() to retrieve the actual manifest.
 *
 * \note If the iterator has not been initialized before, the return value is
 *       undefined.
 *
 * \param[in] iterator Manifest Repository iterator.
 *
 * \retval true if there are more manifests to iterate over.
 * \retval false if there are no more manifests to iterate over.
 */
bool
xme_core_manifestRepository_hasNext
(
    xme_core_manifestRepository_iterator_t iterator
);

/**
 * \brief  Retrieves the next manifest according the the given iterator.
 *
 * \details When calling this function, the iterator moves to the next item,
 *          if any. If there is no more item, this function will return NULL
 *          and iterator will be reset. Subsequent calls will then trigger
 *          a new iteration loop. Manifest repository items may not be
 *          obtained in the order that they have been added to the Ranifest
 *          Repository.
 *
 * \note This method is used jointly with xme_core_manifestRepository_hasNext()
 *       and xme_core_manifestRepository_initIterator().
 *
 * \note If the iterator has not been initialized before, the return value is
 *       undefined.
 *
 * \param[in,out] iterator Address of a variable representing the current
 *                iterator.
 *
 * \return On success, returns the next manifest in the iteration.
 *         If iterator is NULL or there are no more manifests, returns NULL.
 */
xme_core_componentManifest_t*
xme_core_manifestRepository_next
(
    xme_core_manifestRepository_iterator_t* const iterator
);

/**
 * \brief  Finalizes a vertex iterator.
 *
 * \param[in] iterator Manifest Repository iterator.
 */
void
xme_core_manifestRepository_finiIterator
(
    xme_core_manifestRepository_iterator_t iterator
);

/**
 * \brief  Returns the count of valid functions available for that component.
 *
 * \param[in] componentManifest Pointer to Component manifest for which function count has to be returned.
 * \return Returns the count of valid functions.
 */
uint16_t
xme_core_manifestRepository_getFunctionCount
(
    const xme_core_componentManifest_t* const componentManifest
);

/**
 * \brief  Returns the count of valid ports available for that component.
 *
 * \param[in] componentManifest Pointer to Component manifest for which port count has to be returned.
 * \return Returns the count of valid ports.
 */
uint16_t
xme_core_manifestRepository_getPortCount
(
    const xme_core_componentManifest_t* const componentManifest
);


XME_EXTERN_C_END

#endif // #ifndef XME_CORE_MANIFESTREPOSITORY_H
