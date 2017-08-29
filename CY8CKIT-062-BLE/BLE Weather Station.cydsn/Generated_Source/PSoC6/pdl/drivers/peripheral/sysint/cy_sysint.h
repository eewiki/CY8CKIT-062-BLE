/***************************************************************************//**
* \file cy_sysint.h
* \version 1.0
*
* \brief
* Provides an API declaration of the SysInt driver
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
* \defgroup group_sysint System Interrupt (SysInt)
* \{
* The SysInt driver provides an API to configure the device peripheral interrupts.
* It provides a lightweight interface to complement the CMSIS core NVIC API.
* The provided functions are applicable for all cores in a device and they can
* be used to configure and connect device peripheral interrupts to one or more
* cores.
*
* \n
* <b> Initialization </b>
*
* To configure an interrupt, call Cy_SysInt_Init().
* Populate the configuration structure (cy_stc_sysint_t) and pass it as a parameter
* along with the ISR address. This initializes the interrupt and
* instructs the CPU to jump to the specified ISR vector upon a valid trigger.
* 
* Populating the interrupt configuration structure differs slightly for the CM0+ core.
* This core supports only up to 32 peripheral interrupt channels. To allow all device
* interrupts to be routable to the NVIC of this core, there exists a 240:1 multiplexer
* at each of the 32 NVIC channels. Hence the configuration structure (cy_stc_sysint_t)
* must specify the device interrupt source (cm0pSrc) that feeds into the CM0+ NVIC
* mux (intrSrc).
*
* \n
* <b> Enable </b>
* 
* After initializing an interrupt, use the CMSIS Core NVIC_EnableIRQ() function to
* enable it. Given an initialization structure named config, the function should be 
* called as follows:
*
* NVIC_EnableIRQ(config.intrSrc)
*
* \n
* <b>Writing an interrupt service routine</b>
*
* Servicing interrupts in the Peripheral Drivers should follow a prescribed
* recipe to ensure all interrupts are serviced and duplicate interrupts are not
* received. Any peripheral-specific register that must be written to clear the
* source of the interrupt should be written as soon as possible in the interrupt
* service routine. However, note that due to buffering on the output bus to the
* peripherals, the write clearing the interrupt may be delayed. After performing
* the normal interrupt service that should respond to the interrupting
* condition, the interrupt register that was initially written to clear the
* register should be read before returning from the interrupt service routine.
* This read ensures that the initial write has been flushed out to the hardware.
* Note, no additional processing should be performed based on the result of this
* read, as this read is just intended to ensure the write operation is flushed.
*
* This final read may indicate a pending interrupt. What this means is that in
* the interval between when the write actually happened at the peripheral and
* when the read actually happened at the peripheral, an interrupting condition
* occurred. This is ok and a return from the interrupt is still the correct
* action. As soon as conditions warrant, meaning interrupts are enabled and
* there are no higher priority interrupts pending, the interrupt will be
* triggered again to service the additional condition.
*
* \section group_sysint_section_configuration_considerations Configuration Considerations
*
* For the Cortex-M0+ core, NVIC mux positions 28, 29, 30, and 31 are reserved for 
* internal use by Cypress. These should not be used by the application code.
*
* Deep-sleep wakeup-capability is determined by the CPUSS_CM0_DPSLP_IRQ_NR 
* parameter, where the first N number of muxes (NvicMux0 ... NvicMuxN-1) have the 
* capability to trigger deep-sleep interrupts. A deep-sleep capable interrupt source 
* must be connected to one of these muxes to be able to trigger in deep-sleep. 
* Refer to the IRQn_Type definition in the device header.
*
* \section group_sysint_more_information More Information
*
* Refer to the technical reference manual (TRM) and the device datasheet.
*
* \section group_sysint_MISRA MISRA-C Compliance
*
* The sysint driver does not have any specific deviations.
*
* \section group_sysint_changelog Changelog
* <table class="doxtable">
*   <tr><th>Version</th><th>Changes</th><th>Reason for Change</th></tr>
*   <tr>
*     <td>1.0</td>
*     <td>Initial version</td>
*     <td></td>
*   </tr>
* </table>
*
* \defgroup group_sysint_macros Macros
* \defgroup group_sysint_globals Global variables
* \defgroup group_sysint_functions Functions
* \defgroup group_sysint_data_structures Data structures
* \defgroup group_sysint_enums Enumerated Types
*/


#if !defined(CY_SYSINT_H)
#define CY_SYSINT_H

#include <stddef.h>
#include "syslib/cy_syslib.h"
#include "cy_device_headers.h"

#if defined(__cplusplus)
extern "C" {
#endif


/***************************************
*       Global Variable
***************************************/

/**
* \addtogroup group_sysint_globals
* \{
*/

extern const cy_israddress __Vectors[]; /**< Vector table in Flash */
extern cy_israddress __ramVectors[]; /**< Relocated vector table in SRAM */

/** \} group_sysint_globals */


/***************************************
*       Global Interrupt
***************************************/

/**
* \addtogroup group_sysint_macros
* \{
*/

/** Driver major version */
#define CY_SYSINT_DRV_VERSION_MAJOR    1

/** Driver minor version */
#define CY_SYSINT_DRV_VERSION_MINOR    0

/** SysInt driver ID */
#define CY_SYSINT_ID CY_PDL_DRV_ID(0x15u)

/** \} group_sysint_macros */


/***************************************
*       Enumeration
***************************************/

/**
* \addtogroup group_sysint_enums
* \{
*/

/**
* SysInt Driver error codes
*/
typedef enum 
{
    CY_SYSINT_SUCCESS   = 0x00u,                                      /**< Returned successful */
    CY_SYSINT_BAD_PARAM = CY_SYSINT_ID | CY_PDL_STATUS_ERROR | 0x01u, /**< Bad parameter was passed */
} cy_en_sysint_status_t;

/** \} group_sysint_enums */


/***************************************
*       Configuration Structure
***************************************/

/**
* \addtogroup group_sysint_data_structures
* \{
*/

/**
* Initialization configuration structure for a single interrupt channel
*/
typedef struct {
    IRQn_Type       intrSrc;        /**< Interrupt source */
#if (CY_CPU_CORTEX_M0P)
    cy_en_intr_t    cm0pSrc;        /**< (CM0+ only) Maps cm0pSrc device interrupts to intrSrc */
#endif
    uint32_t        intrPriority;   /**< Interrupt priority number (Refer to __NVIC_PRIO_BITS) */
} cy_stc_sysint_t;

/** \} group_sysint_data_structures */


/***************************************
*              Constants
***************************************/

/** \cond INTERNAL */

#define CY_INT_IRQ_BASE            (16u)    /**< Start location of interrupts in the vector table */
#define CY_SYSINT_STATE_MASK       (1ul)    /**< Mask for interrupt state */
#define CY_SYSINT_STIR_MASK        (0xfful) /**< Mask for software trigger interrupt register */
#define CY_SYSINT_CM0P_MUX_MASK    (0xfful) /**< CM0+ NVIC multiplexer mask */
#define CY_SYSINT_CM0P_MUX_SHIFT   (2u)     /**< CM0+ NVIC multiplexer shift */
#define CY_SYSINT_CM0P_MUX_SCALE   (3u)     /**< CM0+ NVIC multiplexer scaling value */

#define CY_SYSINT_CM0P_MUX0        (0u)     /**< CM0+ NVIC multiplexer register 0 */
#define CY_SYSINT_CM0P_MUX1        (1u)     /**< CM0+ NVIC multiplexer register 1 */
#define CY_SYSINT_CM0P_MUX2        (2u)     /**< CM0+ NVIC multiplexer register 2 */
#define CY_SYSINT_CM0P_MUX3        (3u)     /**< CM0+ NVIC multiplexer register 3 */
#define CY_SYSINT_CM0P_MUX4        (4u)     /**< CM0+ NVIC multiplexer register 4 */
#define CY_SYSINT_CM0P_MUX5        (5u)     /**< CM0+ NVIC multiplexer register 5 */
#define CY_SYSINT_CM0P_MUX6        (6u)     /**< CM0+ NVIC multiplexer register 6 */
#define CY_SYSINT_CM0P_MUX7        (7u)     /**< CM0+ NVIC multiplexer register 7 */

#define CY_SYSINT_CM0P_MUX_ERROR   (0xfffffffful)   /**< Invalid CM0+ NVIC multiplexer error code */

/** \endcond */


/***************************************
*       Function Prototypes
***************************************/

/**
* \addtogroup group_sysint_functions
* \{
*/
cy_en_sysint_status_t Cy_SysInt_Init(const cy_stc_sysint_t* config, cy_israddress userIsr);
cy_israddress Cy_SysInt_SetVector(IRQn_Type intrSrc, cy_israddress userIsr);
cy_israddress Cy_SysInt_GetVector(IRQn_Type intrSrc);
__STATIC_INLINE uint32_t Cy_SysInt_GetState(IRQn_Type intrSrc);
__STATIC_INLINE void Cy_SysInt_SetIntSourceNMI(IRQn_Type intrSrc);
__STATIC_INLINE IRQn_Type Cy_SysInt_GetIntSourceNMI(void);
#if (CY_CPU_CORTEX_M0P)
    void Cy_SysInt_SetIntSource(IRQn_Type intrSrc, cy_en_intr_t cm0pSrc);
    cy_en_intr_t Cy_SysInt_GetIntSource(IRQn_Type intrSrc);
#else
    __STATIC_INLINE void Cy_SysInt_SoftwareTrig(IRQn_Type intrSrc);
#endif


/*******************************************************************************
* Function Name: Cy_SysInt_GetState
****************************************************************************//**
*
* \brief Gets the enabled/disabled state of the Interrupt.
*
* Note that for CM0+, this function returns the state of the CM0+ interrupt mux 
* output feeding into the NVIC.
*
* \param intrSrc
* Interrupt source
*
* \return
* 1 if enabled, 0 if disabled
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_SysInt_GetState(IRQn_Type intrSrc)
{
    return (*(NVIC->ISER) >> intrSrc) & CY_SYSINT_STATE_MASK;
}


/*******************************************************************************
* Function Name: Cy_SysInt_SetIntSourceNMI
****************************************************************************//**
*
* \brief Sets the interrupt source of NMI.
*
* The interrupt source must be a positive number. Setting the value to 
* "unconnected_IRQn" (240) disconnects the interrupt source from the NMI. 
*
* Note that for CM0+, this function sets the interrupt mux output feeding into 
* the NVIC as the source for the NMI.
*
* \param intrSrc
* Interrupt source
*
*******************************************************************************/
__STATIC_INLINE void Cy_SysInt_SetIntSourceNMI(IRQn_Type intrSrc)
{
    #if CY_CPU_CORTEX_M0P
        CPUSS->CM0_NMI_CTL = (uint32_t)intrSrc;
    #else
        CPUSS->CM4_NMI_CTL = (uint32_t)intrSrc;
    #endif
}


/*******************************************************************************
* Function Name: Cy_SysInt_GetIntSourceNMI
****************************************************************************//**
*
* \brief Gets the interrupt source of the NMI.
*
* \return
* Interrupt Source. A value of "unconnected_IRQn" (240) means that there is no 
* interrupt source for the NMI, and it can be only be triggered through software.
*
*******************************************************************************/
__STATIC_INLINE IRQn_Type Cy_SysInt_GetIntSourceNMI(void)
{
    #if CY_CPU_CORTEX_M0P
        return (IRQn_Type)(CPUSS->CM0_NMI_CTL);
    #else
        return (IRQn_Type)(CPUSS->CM4_NMI_CTL);
    #endif
}


#if (!CY_CPU_CORTEX_M0P) || defined (CY_DOXYGEN)
    
/*******************************************************************************
* Function Name: Cy_SysInt_SoftwareTrig
****************************************************************************//**
*
* \brief Triggers an interrupt using software (Not applicable for CM0+).
*
* <b>Note</b> Only privileged software can enable unprivileged access to the
* Software Trigger Interrupt Register (STIR).
*
* \param intrSrc
* Interrupt source
*
*******************************************************************************/
__STATIC_INLINE void Cy_SysInt_SoftwareTrig(IRQn_Type intrSrc)
{
    NVIC->STIR = (uint32_t)intrSrc & CY_SYSINT_STIR_MASK;
}

#endif

/** \} group_sysint_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_SYSINT_H */

/** \} group_sysint */

/* [] END OF FILE */
