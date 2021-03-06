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
 * $Id: plugAndPlayClientSendManifestFunction.c 7837 2014-03-14 12:33:13Z wiesmueller $
 */

/**
 * \file
 *         Source file for function pnpClientSendManifest in component pnpClient.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include-gen/plugAndPlayClientSendManifestFunction.h"

#include "xme/core/plugAndPlay/include-gen/plugAndPlayClientSendManifestFunctionWrapper.h"
#include "xme/core/plugAndPlay/include-gen/plugAndPlayClientComponent.h"
#include "xme/core/plugAndPlay/include-gen/plugAndPlayClientComponentWrapper.h"
#include "xme/core/plugAndPlay/include-gen/pnpClientManifest.h"

#include "xme/core/logUtils.h"

#include "xme/hal/include/mem.h"

// PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_C_INCLUDES) ENABLED START
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"
// PROTECTED REGION END

/******************************************************************************/
/***   Definitions                                                          ***/
/******************************************************************************/

// PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_C_DEFINITIONS) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief  Variable holding the value of the optional output port 'outManifest'.
 *
 * \details If necessary initialize this in the init function.
 *          The value of this variable will be written to the port at the end of
 *          the step function.
 */
static xme_core_topic_pnp_componentInstanceManifest_t
portOutManifestData;

// PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_C_VARIABLES) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

// PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_C_PROTOTYPES) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_pnp_pnpClientSendManifestFunction_init
(
    xme_core_pnp_pnpClientComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_INITIALIZE_C) ENABLED START

    // Nothing to do

    XME_UNUSED_PARAMETER(componentConfig);

    return XME_STATUS_SUCCESS;

    // PROTECTED REGION END
}

void
xme_core_pnp_pnpClientSendManifestFunction_step
(
    xme_core_pnp_pnpClientComponent_config_t* const componentConfig
)
{
    xme_status_t status[1];
    
    xme_core_topic_login_loginAcknowledgment_t portInLoginAcknowledgmentData; // Optional port.
    xme_core_topic_pnp_componentInstanceManifest_t* portOutManifestDataPtr = &portOutManifestData;
    
    (void)xme_hal_mem_set(&portInLoginAcknowledgmentData, 0u, sizeof(xme_core_topic_login_loginAcknowledgment_t));
    
    status[0] = xme_core_pnp_pnpClientComponentWrapper_readPortInLoginAcknowledgment(&portInLoginAcknowledgmentData);
    
    {
        // PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_STEP_C) ENABLED START

        static bool sentManifest = false; 
        xme_core_node_nodeId_t currentNodeID = xme_core_node_getCurrentNodeId();

        XME_UNUSED_PARAMETER(componentConfig);

        portOutManifestDataPtr = NULL;

        if (XME_CORE_NODE_INVALID_NODE_ID != currentNodeID) // We can only announce instance manifests, when the node is logged in
        {
            // Check if we should send the login instance manifest
            if (status[0] == XME_STATUS_SUCCESS && // We received a login acknowledgment
                currentNodeID == portInLoginAcknowledgmentData.nodeId && // The received ack is meant for this node
                !sentManifest) // We did not already send the login component instance manifest
            {
                xme_status_t returnValue;
                
                sentManifest = true;

                returnValue = xme_core_pnp_pnpClient_getManifest(&portOutManifestData, true);

                if (XME_STATUS_SUCCESS == returnValue)
                {
                    portOutManifestDataPtr = &portOutManifestData;
                    XME_LOG(XME_LOG_NOTE, "[PlugAndPlayClient] Sending Component Instance Manifest to Plug and Play Manager.\n");
                }
                else if (XME_STATUS_NOT_FOUND != returnValue)
                {
                    XME_LOG(XME_LOG_ERROR, "[PlugAndPlayClient] Error creating login component instance manifest.\n");
                }
            }
            else
            {
                // Check if there are any new components to announce
                xme_status_t status = xme_core_pnp_pnpClient_getManifest(&portOutManifestData, false);
                if (XME_STATUS_SUCCESS == status)
                {
                    portOutManifestDataPtr = &portOutManifestData;
                    XME_LOG(XME_LOG_NOTE, "[PlugAndPlayClient] Sending Component Instance Manifest to Plug and Play Manager.\n");
                }
            }
        }

        // PROTECTED REGION END
    }
    
    status[0] = xme_core_pnp_pnpClientComponentWrapper_writePortOutManifest(portOutManifestDataPtr);
    
    {
        // PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_STEP_2_C) ENABLED START

         if (XME_STATUS_SUCCESS != status[0])
         {
            XME_LOG(XME_LOG_DEBUG, "[plugAndPlaySendManifestFunction] Failed to write the Manifest to the port with status %d\n", status[0]);
         }

        // PROTECTED REGION END
    }
}

void
xme_core_pnp_pnpClientSendManifestFunction_fini
(
    xme_core_pnp_pnpClientComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_TERMINATE_C) ENABLED START

    // Nothing to do

    XME_UNUSED_PARAMETER(componentConfig);

    // PROTECTED REGION END
}

// PROTECTED REGION ID(XME_CORE_PNP_PNPCLIENTSENDMANIFESTFUNCTION_IMPLEMENTATION_C) ENABLED START

// PROTECTED REGION END
