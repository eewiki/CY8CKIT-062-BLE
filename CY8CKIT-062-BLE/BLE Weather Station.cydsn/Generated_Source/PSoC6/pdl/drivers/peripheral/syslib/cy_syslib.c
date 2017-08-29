/***************************************************************************//**
* \file cy_syslib.c
* \version 1.0
*
*  Description:
*   Provides system API implementation for the syslib driver.
*
********************************************************************************
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_syslib.h"
#include "ipc/cy_ipc_drv.h"
#if !defined(NDEBUG)
    #include <string.h>
#endif /* NDEBUG */

/* Flash wait states (LP mode at 1.1v) */
#define CY_SYSLIB_FLASH_LP_WS_0_FREQ_MAX    ( 24UL)
#define CY_SYSLIB_FLASH_LP_WS_1_FREQ_MAX    ( 48UL)
#define CY_SYSLIB_FLASH_LP_WS_2_FREQ_MAX    ( 72UL)
#define CY_SYSLIB_FLASH_LP_WS_3_FREQ_MAX    ( 96UL)
#define CY_SYSLIB_FLASH_LP_WS_4_FREQ_MAX    (120UL)

/* Flash wait states (ULP mode at 0.9v) */
#define CY_SYSLIB_FLASH_ULP_WS_0_FREQ_MAX    ( 16UL)
#define CY_SYSLIB_FLASH_ULP_WS_1_FREQ_MAX    ( 33UL)
#define CY_SYSLIB_FLASH_ULP_WS_2_FREQ_MAX    ( 50UL)

/* ROM wait states for the slow clock domain (LP mode at 1.1v) */
#define CY_SYSLIB_ROM_LP_SLOW_WS_0_FREQ_MAX  (100UL)
#define CY_SYSLIB_ROM_LP_SLOW_WS_1_FREQ_MAX  (120UL)

/* ROM wait states for the slow clock domain (ULP mode at 0.9v) */
#define CY_SYSLIB_ROM_ULP_SLOW_WS_0_FREQ_MAX ( 25UL)
#define CY_SYSLIB_ROM_ULP_SLOW_WS_1_FREQ_MAX ( 50UL)

/* ROM wait states for the fast clock domain (LP mode at 1.1v) */
#define CY_SYSLIB_ROM_LP_FAST_WS_0_FREQ_MAX  (120UL)

/* ROM wait states for the slow clock domain (ULP mode at 0.9v) */
#define CY_SYSLIB_ROM_ULP_FAST_WS_0_FREQ_MAX ( 50UL)

/* SRAM wait states for the slow clock domain (LP mode at 1.1v) */
#define CY_SYSLIB_RAM_LP_SLOW_WS_0_FREQ_MAX  (100UL)
#define CY_SYSLIB_RAM_LP_SLOW_WS_1_FREQ_MAX  (120UL)

/* SRAM wait states for the slow clock domain (ULP mode at 0.9v) */
#define CY_SYSLIB_RAM_ULP_SLOW_WS_0_FREQ_MAX ( 25UL)
#define CY_SYSLIB_RAM_ULP_SLOW_WS_1_FREQ_MAX ( 50UL)

/* SRAM wait states for the fast clock domain (LP mode at 1.1v) */
#define CY_SYSLIB_RAM_LP_FAST_WS_0_FREQ_MAX  (120UL)

/* SRAM wait states for the fast clock domain (ULP mode at 0.9v) */
#define CY_SYSLIB_RAM_ULP_FAST_WS_0_FREQ_MAX ( 50UL)


#if !defined(NDEBUG)
    char_t cy_assertFileName[CY_MAX_FILE_NAME_SIZE];
    uint32_t cy_assertLine;
#endif /* NDEBUG */

#if (CY_ARM_FAULT_DEBUG == CY_ARM_FAULT_DEBUG_ENABLED)
    cy_stc_fault_frame_t cy_faultFrame;
#endif /* (CY_ARM_FAULT_DEBUG == CY_ARM_FAULT_DEBUG_ENABLED) */

static void Cy_SysLib_SetWaitStates_ULP(uint32_t clkHfMHz);
static void Cy_SysLib_SetWaitStates_LP(uint32_t clkHfMHz);

#if defined(__ARMCC_VERSION)
    static __ASM void Cy_SysLib_AsmInfiniteLoop(void) { b . };
#endif  /* (__ARMCC_VERSION) */
    

/*******************************************************************************
* Function Name: Cy_SysLib_Delay
****************************************************************************//**
*
* The function delays by the specified number of milliseconds.
* By default, the number of cycles to delay is calculated based on the
* SystemCoreClock.
* If the parameter is bigger than CY_DELAY_MS_OVERFLOW(32kHz delay), run an
* additional loop to prevent an overflow.
*
*  \param milliseconds  The number of milliseconds to delay.
*
*******************************************************************************/
void Cy_SysLib_Delay(uint32_t milliseconds)
{
    while (milliseconds > CY_DELAY_MS_OVERFLOW)
    {
        /* This loop prevents overflow.
         * At 100MHz, milliseconds * cy_delayFreqKhz it overflows at about 42 seconds
         */
        Cy_SysLib_DelayCycles(cy_delay32kMs);
        milliseconds -= CY_DELAY_MS_OVERFLOW;
    }

    Cy_SysLib_DelayCycles(milliseconds * cy_delayFreqKhz);
}


/*******************************************************************************
* Function Name: Cy_SysLib_DelayUs
****************************************************************************//**
*
* The function delays by the specified number of microseconds.
* By default, the number of cycles to delay is calculated based on the
* SystemCoreClock.
*
*  \param microseconds  The number of microseconds to delay.
*
*  \note If the CPU frequency is a small non-integer number, the actual delay
*        can be up to twice as long as the nominal value. The actual delay
*        cannot be shorter than the nominal one.
*
*******************************************************************************/
void Cy_SysLib_DelayUs(uint16_t microseconds)
{
    Cy_SysLib_DelayCycles((uint32_t) microseconds * cy_delayFreqMhz);
}


/*******************************************************************************
* Function Name: Cy_SysLib_Halt
****************************************************************************//**
*
*  This function halts the CPU but only the CPU which calls the function.
*  It doesn't affect other CPUs.
*
*  \param reason  The value to be used during debugging.
*
*  \note The function executes the BKPT instruction for halting CPU and is
*        intended to be used for debug purposes. A regular use case requires
*        Debugger attachment before the function call.
*        The BKPT instruction causes the CPU to enter the Debug state. Debug
*        tools can use this to investigate the system state, when the
*        instruction at a particular address is reached.
*
*  \note Execution of a BKPT instruction without a debugger attached produces
*        a fault. The fault results in the HardFault exception being taken
*        or causes Lockup state if it occurs in the NMI or HardFault handler.
*        The default HardFault handler make a software reset if the build option
*        is release mode (NDEBUG). If the build option is debug mode, the system
*        will stay in the infinite loop of the Cy_SysLib_ProcessingFault()
*        function.
*
*******************************************************************************/
__NO_RETURN void Cy_SysLib_Halt(uint32_t reason)
{
    if(0u != reason)
    {
        /* To remove an unreferenced local variable warning */
    }

    #if defined (__ARMCC_VERSION)
        __breakpoint(0x0);
        while(1u) {}
    #elif defined(__GNUC__)
        __asm("    bkpt    1");
        __builtin_unreachable();
    #elif defined (__ICCARM__)
        __asm("    bkpt    1");
    #else
        #error "An unsupported toolchain"
    #endif  /* (__ARMCC_VERSION) */
}


#if !defined(NDEBUG) || defined(CY_DOXYGEN)
/*******************************************************************************
* Macro Name: Cy_SysLib_AssertFailed
****************************************************************************//**
*
*  This function stores the ASSERT location of the file name (including path
*  to file) and line number in a non-zero init area for debugging. Also it calls
*  Cy_SysLib_Halt() function to halt the processor.
*
*  \param file  The file name of the ASSERT location.
*  \param line  The line number of the ASSERT location.
*
* \note Stored file name and line number could be accessed by cy_assertFileName
*       and cy_assertLine global variables.
* \note This function has WEAK option, so the user can redefine the function
*       for a custom processing.
*
*******************************************************************************/
__WEAK void Cy_SysLib_AssertFailed(const char_t * file, uint32_t line)
{
    (void) strncpy(cy_assertFileName, file, CY_MAX_FILE_NAME_SIZE);
    cy_assertLine = line;
    Cy_SysLib_Halt(0UL);
}
#endif  /* !defined(NDEBUG) || defined(CY_DOXYGEN) */


/*******************************************************************************
* Function Name: Cy_SysLib_ClearFlashCacheAndBuffer
****************************************************************************//**
*
*  This function invalidates the flash cache and buffer. It ensures the valid
*  data is read from flash instead of using outdated data from the cache.
*  The caches' LRU structure is also reset to their default state.
*
*  \note The operation takes a maximum of three clock cycles on the slowest of
*        the clk_slow and clk_fast clocks.
*
*******************************************************************************/
void Cy_SysLib_ClearFlashCacheAndBuffer(void)
{
    FLASHC->FLASH_CMD = FLASHC_FLASH_CMD_INV_Msk;
}


/*******************************************************************************
* Function Name: Cy_SysLib_GetResetReason
****************************************************************************//**
*
*  The function returns the cause for the latest reset(s) that occurred in
*  the system. The reset causes include HFCLK error, system faults, and device
*  reset on a wakeup from Hibernate mode.
*  The return results are consolidated reset causes from reading RES_CAUSE,
*  RES_CAUSE2 and PWR_HIBERNATE token registers.
*
*  \return The cause of a system reset.
*
*  | Name                          | Value
*  |-------------------------------|---------------------
*  | CY_SYSLIB_RESET_HWWDT         | 0x00001 (bit0)
*  | CY_SYSLIB_RESET_ACT_FAULT     | 0x00002 (bit1)
*  | CY_SYSLIB_RESET_DPSLP_FAULT   | 0x00004 (bit2)
*  | CY_SYSLIB_RESET_CSV_WCO_LOSS  | 0x00008 (bit3)
*  | CY_SYSLIB_RESET_SOFT          | 0x00010 (bit4)
*  | CY_SYSLIB_RESET_SWWDT0        | 0x00020 (bit5)
*  | CY_SYSLIB_RESET_SWWDT1        | 0x00040 (bit6)
*  | CY_SYSLIB_RESET_SWWDT2        | 0x00080 (bit7)
*  | CY_SYSLIB_RESET_SWWDT3        | 0x00100 (bit8)
*  | CY_SYSLIB_RESET_HFCLK_LOSS    | 0x10000 (bit16)
*  | CY_SYSLIB_RESET_HFCLK_ERR     | 0x20000 (bit17)
*  | CY_SYSLIB_RESET_HIB_WAKEUP    | 0x40000 (bit18)
*
*  \note CY_SYSLIB_RESET_CSV_WCO_LOSS, CY_SYSLIB_RESET_HFCLK_LOSS and
*        CY_SYSLIB_RESET_HFCLK_ERR causes of a system reset available only if
*        WCO CSV present in the device.
*
*******************************************************************************/
uint32_t Cy_SysLib_GetResetReason(void)
{
    uint32_t retVal;

    retVal = SRSS->RES_CAUSE;

#if (SRSS_WCOCSV_PRESENT != 0u)
    retVal |= ((CY_LO16(SRSS->RES_CAUSE2) > 0u) ? CY_SYSLIB_RESET_HFCLK_LOSS : 0u) |
              ((CY_HI16(SRSS->RES_CAUSE2) > 0u) ? CY_SYSLIB_RESET_HFCLK_ERR  : 0u);
#endif /* (SRSS_WCOCSV_PRESENT != 0u) */

    if(0u != _FLD2VAL(SRSS_PWR_HIBERNATE_TOKEN, SRSS->PWR_HIBERNATE))
    {
        retVal |= CY_SYSLIB_RESET_HIB_WAKEUP;
        SRSS->PWR_HIBERNATE &= ~SRSS_PWR_HIBERNATE_TOKEN_Msk;
    }

    return (retVal);
}


#if (SRSS_WCOCSV_PRESENT != 0u) || defined(CY_DOXYGEN)
/*******************************************************************************
* Function Name: Cy_SysLib_GetNumHfclkResetCause
****************************************************************************//**
*
* This function returns the number of HF_CLK which is a reset cause (RES_CAUSE2)
* by a loss or an error of the high frequency clock.
*
* The Clock supervisors (CSV) can make a reset as CSV_FREQ_ACTION setting
* when a CSV frequency anomaly is detected. The function returns the index
* of HF_CLK, if a reset occurred due to an anomaly HF_CLK. There are two
* different options for monitoring on HF_CLK which are a frequency loss
* and a frequency error.
*
* \return
*   - The number HF_CLK from Clock Supervisor High Frequency Loss:  Bits[15:0]
*   - The number HF_CLK from Clock Supervisor High Frequency error: Bits[31:16]
*
*******************************************************************************/
uint32_t Cy_SysLib_GetNumHfclkResetCause(void)
{
    return (SRSS->RES_CAUSE2);
}
#endif /* (SRSS_WCOCSV_PRESENT != 0u) || defined(CY_DOXYGEN) */


/*******************************************************************************
* Function Name: Cy_SysLib_ClearResetReason
****************************************************************************//**
*
*  This function clears the values of RES_CAUSE and RES_CAUSE2.
*
*******************************************************************************/
void Cy_SysLib_ClearResetReason(void)
{
    /* RES_CAUSE and RES_CAUSE2 register's bits are RW1C (every bit is cleared upon writing 1),
     * so write all ones to clear all the reset reasons.
     */
    SRSS->RES_CAUSE  = 0xFFFFFFFFu;
    SRSS->RES_CAUSE2 = 0xFFFFFFFFu;
}


#if (CY_CPU_CORTEX_M0P) || defined(CY_DOXYGEN)
/*******************************************************************************
* Function Name: Cy_SysLib_SoftResetCM4
****************************************************************************//**
*
*  This function performs a CM4 Core software reset using the CM4_PWR_CTL
*  register. The register is accessed by CM0 Core by using command transferred
*  to SROM API through IPC channel. Once sent the command, the API waits for
*  the IPC channel release.
*
*  \note This function should be called only when the CM4 core is in Deep
*        Sleep mode.
*  \note This function will not reset CM0+ Core.
*  \note This function waits for an IPC channel release state.
*
*******************************************************************************/
void Cy_SysLib_SoftResetCM4(void)
{
    volatile uint32_t msg = CY_IPC_DATA_FOR_CM4_SOFT_RESET;

    /* Tries to acquire the IPC structure and pass the arguments to SROM API.
    *  SROM API parameters:
    *   ipcPtr: IPC_STRUCT0 - IPC Structure 0 reserved for M0+ Secure Access.
    *   notifyEvent_Intr: 1u - IPC Interrupt Structure 1 is used for Releasing IPC 0 (M0+ NMI Handler).
    *   msgPtr: &msg - The address of SRAM with the API's parameters.
    */
    if(CY_IPC_DRV_SUCCESS != Cy_IPC_Drv_SendMsgPtr(IPC_STRUCT0, 1u, (void *) &msg))
    {
        CY_ASSERT(0u != 0u);
    }
    
    while (Cy_IPC_Drv_IsLockAcquired(IPC_STRUCT0))
    {
        /* Waits until SROM API runs the command (sent over the IPC channel) and releases the IPC_STRUCT0. */
    }
}
#endif /* CY_CPU_CORTEX_M0P || defined(CY_DOXYGEN) */


/*******************************************************************************
* Function Name: Cy_SysLib_GetUniqueId
****************************************************************************//**
*
* This function returns the silicon unique ID.
* The ID includes Die lot[3]#, Die Wafer#, Die X, Die Y, DIE Sort# and
* Die Minor#
*
* \return
* A combined 64-bit unique ID.
* [63:56] - DIE_MINOR
* [55:48] - DIE_SORT
* [47:40] - DIE_Y
* [39:32] - DIE_X
* [31:24] - DIE_WAFER
* [23:16] - DIE_LOT[2]
* [15: 8] - DIE_LOT[1]
* [ 7: 0] - DIE_LOT[0]
*
*******************************************************************************/
uint64_t Cy_SysLib_GetUniqueId(void)
{
    uint64_t uniqueId;

    uniqueId = ((uint64_t)SFLASH->DIE_MINOR   << CY_UNIQUE_ID_DIE_MINOR_Pos) |
               ((uint64_t)SFLASH->DIE_SORT    << CY_UNIQUE_ID_DIE_SORT_Pos)  |
               ((uint64_t)SFLASH->DIE_Y       << CY_UNIQUE_ID_DIE_Y_Pos)     |
               ((uint64_t)SFLASH->DIE_X       << CY_UNIQUE_ID_DIE_X_Pos)     |
               ((uint64_t)SFLASH->DIE_WAFER   << CY_UNIQUE_ID_DIE_WAFER_Pos) |
               ((uint64_t)SFLASH->DIE_LOT[2u] << CY_UNIQUE_ID_DIE_LOT_2_Pos) |
               ((uint64_t)SFLASH->DIE_LOT[1u] << CY_UNIQUE_ID_DIE_LOT_1_Pos) |
               ((uint64_t)SFLASH->DIE_LOT[0u] << CY_UNIQUE_ID_DIE_LOT_0_Pos);

    return (uniqueId);
}


#if (CY_ARM_FAULT_DEBUG == CY_ARM_FAULT_DEBUG_ENABLED) || defined(CY_DOXYGEN)
/*******************************************************************************
* Function Name: Cy_SysLib_FaultHandler
****************************************************************************//**
*
* This function stores the ARM Cortex registers into a non-zero init area for
* debugging. This function calls Cy_SysLib_ProcessingFault() after storing all
* the information.
*
* \param faultStackAddr The address of the stack pointer, indicates the lowest
*                       address in the fault stack frame to be stored.
* \note This function stores the fault stack frame only for the first occurred
*       fault.
* \note This function sets two flags into the fault structure: one checks
*       which core cause fault, another checks the type of fault for CM4 core.
*       The flags should be cleared after analysis of the stored
*       register values, to have possibility to distinguish the next fault.
* \note The PDL doesn't provide an API which can analyse the stored register
*       values. A user has to add additional functions for the analysis,
*       if necessary.
* \note The CY_ARM_FAULT_DEBUG macro defines if the Fault Handler is enabled.
*       By default it is set to CY_ARM_FAULT_DEBUG_ENABLED and enables the
*       Fault Handler.
*       If there is a necessity to save memory or have some specific custom
*       handler, etc. then CY_ARM_FAULT_DEBUG should be redefined as
*       CY_ARM_FAULT_DEBUG_DISABLED. To do this the following definition should
*       be added to the compiler Command Line (through the project Build
*       Settings): "-D CY_ARM_FAULT_DEBUG=0".
*
*******************************************************************************/
void Cy_SysLib_FaultHandler(uint32_t const *faultStackAddr)
{
    /* Stores general registers */
    cy_faultFrame.r0  = faultStackAddr[CY_R0_Pos];
    cy_faultFrame.r1  = faultStackAddr[CY_R1_Pos];
    cy_faultFrame.r2  = faultStackAddr[CY_R2_Pos];
    cy_faultFrame.r3  = faultStackAddr[CY_R3_Pos];
    cy_faultFrame.r12 = faultStackAddr[CY_R12_Pos];
    cy_faultFrame.lr  = faultStackAddr[CY_LR_Pos];
    cy_faultFrame.pc  = faultStackAddr[CY_PC_Pos];
    cy_faultFrame.psr = faultStackAddr[CY_PSR_Pos];

    #if (CY_CPU_CORTEX_M4)
        /* Stores the Configurable Fault Status Register state with the fault cause */
        cy_faultFrame.cfsr = SCB->CFSR;
        cy_faultFrame.faultType = CY_NON_FPU_FAULT_TYPE;

        #if ((defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)) && \
             (defined (__FPU_USED   ) && (__FPU_USED    == 1U)))
            /* Checks cumulative exception bits for floating-point exceptions */
            if(0U != (__get_FPSCR() & (CY_FPSCR_IXC_Msk | CY_FPSCR_IDC_Msk)))
            {
                cy_faultFrame.faultType = CY_FPU_FAULT_TYPE;
                cy_faultFrame.s0    = faultStackAddr[CY_S0_Pos];
                cy_faultFrame.s1    = faultStackAddr[CY_S1_Pos];
                cy_faultFrame.s2    = faultStackAddr[CY_S2_Pos];
                cy_faultFrame.s3    = faultStackAddr[CY_S3_Pos];
                cy_faultFrame.s4    = faultStackAddr[CY_S4_Pos];
                cy_faultFrame.s5    = faultStackAddr[CY_S5_Pos];
                cy_faultFrame.s6    = faultStackAddr[CY_S6_Pos];
                cy_faultFrame.s7    = faultStackAddr[CY_S7_Pos];
                cy_faultFrame.s8    = faultStackAddr[CY_S8_Pos];
                cy_faultFrame.s9    = faultStackAddr[CY_S9_Pos];
                cy_faultFrame.s10   = faultStackAddr[CY_S10_Pos];
                cy_faultFrame.s11   = faultStackAddr[CY_S11_Pos];
                cy_faultFrame.s12   = faultStackAddr[CY_S12_Pos];
                cy_faultFrame.s13   = faultStackAddr[CY_S13_Pos];
                cy_faultFrame.s14   = faultStackAddr[CY_S14_Pos];
                cy_faultFrame.s15   = faultStackAddr[CY_S15_Pos];
                cy_faultFrame.fpscr = faultStackAddr[CY_FPSCR_Pos];
            }
        #endif /* CY_CPU_CORTEX_M4 && __FPU_PRESENT */
    #endif /* CY_CPU_CORTEX_M4 */

    Cy_SysLib_ProcessingFault();
}


/*******************************************************************************
* Function Name: Cy_SysLib_ProcessingFault
****************************************************************************//**
*
* This function determines how to process the current fault state. By default
* in case of exception the system will stay in the infinite loop of this
* function.
*
* \note This function has WEAK option, so the user can redefine the function
*       for a custom processing.
*       For example the function redefinition could be constructed from fault
*       stack processing and NVIC_SystemReset() function call.
*
*******************************************************************************/
__WEAK void Cy_SysLib_ProcessingFault(void)
{
    #if defined(__ARMCC_VERSION)
        /* Assembly implementation of infinite loop
         * is used for armcc compiler to preserve call stack. 
         * Otherwise the compiler destroys call stack,
         * because treat this API as no return function.
         */
        Cy_SysLib_AsmInfiniteLoop();
    #else
        while(1u) {}
    #endif  /* (__ARMCC_VERSION) */
}
#endif /* (CY_ARM_FAULT_DEBUG == CY_ARM_FAULT_DEBUG_ENABLED) || defined(CY_DOXYGEN) */


/*******************************************************************************
* Function Name: Cy_SysLib_SetWaitStates
****************************************************************************//**
*
* Sets the number of clock cycles the cache will wait for, before it samples
* data coming back from ROM, SRAM, and Flash.
*
* Call this function before increasing the HFClk0 High Frequency clock.
* Call this function optionally after lowering the HFClk0 High Frequency clock
* in order to improve the CPU performance.
*
* Also, call this function before switching core supply regulator voltage (LDO
* or SIMO Buck) from 1.1V to 0.9V.
* Call this function optionally after switching core supply regulator voltage
* from 0.9V to 1.1V in order to improve the CPU performance.
*
* \param ulpMode The device power mode.
* true, if the device should be switched to the ULP mode (nominal voltage of
* the core supply regulator should be switched to 0.9V);
* false, if the device should be switched to the LP mode (nominal voltage of
* the core supply regulator should be switched to 1.1V).
*
* \note Refer to the device TRM for low power modes description.
*
* \param clkHfMHz The HFClk0 clock frequency in MHz. Specifying frequency above
* maximum supported will set the wait states as for the maximum frequency.
*
*******************************************************************************/
void Cy_SysLib_SetWaitStates(bool ulpMode, uint32_t clkHfMHz)
{
    if (ulpMode)
    {
        Cy_SysLib_SetWaitStates_ULP(clkHfMHz);
    }
    else
    {
        Cy_SysLib_SetWaitStates_LP(clkHfMHz);
    }
}


/*******************************************************************************
* Function Name: Cy_SysLib_SetWaitStates_LP
****************************************************************************//**
*
*  Sets the ROM, SRAM, and Flash wait states for the low power mode.
*  This function is called by \ref Cy_SysLib_SetWaitStates().
*
* \param clkHfMHz The HFClk0 clock frequency in MHz. Specifying frequency above
* maximum supported will set the wait states as for the maximum frequency.
*
*******************************************************************************/
static void Cy_SysLib_SetWaitStates_LP(uint32_t clkHfMHz)
{
    uint32_t waitStates;

    /* ROM */
    if (clkHfMHz < CY_SYSLIB_ROM_LP_SLOW_WS_0_FREQ_MAX)
    {
        waitStates = 0UL;
    } else
    {
        waitStates = 1UL;
    }
    CPUSS->ROM_CTL = _CLR_SET_FLD32U(CPUSS->ROM_CTL, CPUSS_ROM_CTL_SLOW_WS, waitStates);

    waitStates = 0UL;
    CPUSS->ROM_CTL = _CLR_SET_FLD32U(CPUSS->ROM_CTL, CPUSS_ROM_CTL_FAST_WS, waitStates);

    /* SRAM */
    if (clkHfMHz < CY_SYSLIB_RAM_LP_SLOW_WS_0_FREQ_MAX)
    {
        waitStates = 0UL;
    } else
    {
        waitStates = 1UL;
    }
    CPUSS->RAM0_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM0_CTL0, CPUSS_RAM0_CTL0_SLOW_WS, waitStates);
    #if defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL)
        CPUSS->RAM1_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM1_CTL0, CPUSS_RAM1_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL) */
    #if defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL)
        CPUSS->RAM2_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM2_CTL0, CPUSS_RAM2_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL) */

    waitStates = 0UL;
    CPUSS->RAM0_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM0_CTL0, CPUSS_RAM0_CTL0_FAST_WS, waitStates);
    #if defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL)
        CPUSS->RAM1_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM1_CTL0, CPUSS_RAM1_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL) */
    #if defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL)
        CPUSS->RAM2_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM2_CTL0, CPUSS_RAM2_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL) */

    /* Flash */
    if (clkHfMHz <= CY_SYSLIB_FLASH_LP_WS_0_FREQ_MAX)
    {
        waitStates = 0UL;
    } else if (clkHfMHz <= CY_SYSLIB_FLASH_LP_WS_1_FREQ_MAX)
    {
        waitStates = 1UL;
    } else if (clkHfMHz <= CY_SYSLIB_FLASH_LP_WS_2_FREQ_MAX)
    {
        waitStates = 2UL;
    } else if (clkHfMHz <= CY_SYSLIB_FLASH_LP_WS_3_FREQ_MAX)
    {
        waitStates = 3UL;
    } else
    {
        waitStates = 4UL;
    }

    FLASHC->FLASH_CTL = _CLR_SET_FLD32U(FLASHC->FLASH_CTL, FLASHC_FLASH_CTL_MAIN_WS, waitStates);
}


/*******************************************************************************
* Function Name: Cy_SysLib_SetWaitStates_ULP
****************************************************************************//**
*
*  Sets the ROM, SRAM, and Flash wait states for the ultra-low power mode.
*  This function is called by \ref Cy_SysLib_SetWaitStates().
*
* \param clkHfMHz The HFClk0 clock frequency in MHz. Specifying frequency above
* maximum supported will set the wait states as for the maximum frequency.
*
*******************************************************************************/
static void Cy_SysLib_SetWaitStates_ULP(uint32_t clkHfMHz)
{
    uint32_t waitStates;

    /* ROM */
    if (clkHfMHz < CY_SYSLIB_ROM_ULP_SLOW_WS_0_FREQ_MAX)
    {
        waitStates = 0UL;
    } else
    {
        waitStates = 1UL;
    }
    CPUSS->ROM_CTL = _CLR_SET_FLD32U(CPUSS->ROM_CTL, CPUSS_ROM_CTL_SLOW_WS, waitStates);

    waitStates = 0UL;
    CPUSS->ROM_CTL = _CLR_SET_FLD32U(CPUSS->ROM_CTL, CPUSS_ROM_CTL_FAST_WS, waitStates);

    /* SRAM */
    if (clkHfMHz < CY_SYSLIB_RAM_ULP_SLOW_WS_0_FREQ_MAX)
    {
        waitStates = 0UL;
    } else
    {
        waitStates = 1UL;
    }
    CPUSS->RAM0_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM0_CTL0, CPUSS_RAM0_CTL0_SLOW_WS, waitStates);
    #if defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL)
        CPUSS->RAM1_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM1_CTL0, CPUSS_RAM1_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL) */
    #if defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL)
        CPUSS->RAM2_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM2_CTL0, CPUSS_RAM2_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL) */

    waitStates = 0UL;
    CPUSS->RAM0_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM0_CTL0, CPUSS_RAM0_CTL0_FAST_WS, waitStates);
    #if defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL)
        CPUSS->RAM1_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM1_CTL0, CPUSS_RAM1_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC1_PRESENT) && (RAMC1_PRESENT == 1UL) */
    #if defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL)
        CPUSS->RAM2_CTL0 = _CLR_SET_FLD32U(CPUSS->RAM2_CTL0, CPUSS_RAM2_CTL0_SLOW_WS, waitStates);
    #endif /* defined (RAMC2_PRESENT) && (RAMC2_PRESENT == 1UL) */

    /* Flash */
    if (clkHfMHz <= CY_SYSLIB_FLASH_ULP_WS_0_FREQ_MAX)
    {
        waitStates = 0UL;
    } else if (clkHfMHz <= CY_SYSLIB_FLASH_ULP_WS_1_FREQ_MAX)
    {
        waitStates = 1UL;
    } else
    {
        waitStates = 2UL;
    }
    FLASHC->FLASH_CTL = _CLR_SET_FLD32U(FLASHC->FLASH_CTL, FLASHC_FLASH_CTL_MAIN_WS, waitStates);
}

/* [] END OF FILE */
