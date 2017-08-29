/***************************************************************************//**
* \file cyip_sflash.h
*
* \brief
* SFLASH IP definitions
*
* \note
* Generated 5/26/2017 by CyDeviceHeaderGenerator v1.1.0.67
* from the register map configuration rev#983818
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _CYIP_SFLASH_H_
#define _CYIP_SFLASH_H_

#include "cyip_headers.h"

/*******************************************************************************
*                                    SFLASH
*******************************************************************************/

#define SFLASH_SECTION_SIZE                     0x00008000UL

/**
  * \brief FLASH Supervisory Region (SFLASH)
  */
typedef struct {
   __IM uint16_t RESERVED;
  __IOM uint16_t SILICON_ID;                    /*!< 0x00000002 Indicates siliconID of the device */
   __IM uint32_t RESERVED1[383];
  __IOM uint8_t  DIE_LOT[3];                    /*!< 0x00000600 Lot Number (3 bytes) */
  __IOM uint8_t  DIE_WAFER;                     /*!< 0x00000603 Wafer Number */
  __IOM uint8_t  DIE_X;                         /*!< 0x00000604 X Position on Wafer, CRI Pass/Fail Bin */
  __IOM uint8_t  DIE_Y;                         /*!< 0x00000605 Y Position on Wafer, CHI Pass/Fail Bin */
  __IOM uint8_t  DIE_SORT;                      /*!< 0x00000606 Sort1/2/3 Pass/Fail Bin */
  __IOM uint8_t  DIE_MINOR;                     /*!< 0x00000607 Minor Revision Number */
  __IOM uint8_t  DIE_DAY;                       /*!< 0x00000608 Day number */
  __IOM uint8_t  DIE_MONTH;                     /*!< 0x00000609 Month number */
  __IOM uint8_t  DIE_YEAR;                      /*!< 0x0000060A Year number */
   __IM uint8_t  RESERVED2[501];
  __IOM uint32_t BLE_DEVICE_ADDRESS[128];       /*!< 0x00000800 BLE_DEVICE_ADDRESS */
  __IOM uint32_t USER_FREE_ROW1[128];           /*!< 0x00000A00 USER_FREE_ROW1 */
  __IOM uint32_t USER_FREE_ROW2[128];           /*!< 0x00000C00 USER_FREE_ROW2 */
  __IOM uint32_t USER_FREE_ROW3[128];           /*!< 0x00000E00 USER_FREE_ROW3 */
   __IM uint32_t RESERVED3[302];
  __IOM uint8_t  DEVICE_UID[16];                /*!< 0x000014B8 Unique Identifier Number for each device */
  __IOM uint8_t  MASTER_KEY[16];                /*!< 0x000014C8 Master key to change other keys */
  __IOM uint32_t STANDARD_SMPU_STRUCT_SLAVE_ADDR[16]; /*!< 0x000014D8 Standard SMPU STRUCT Slave Address value */
  __IOM uint32_t STANDARD_SMPU_STRUCT_SLAVE_ATTR[16]; /*!< 0x00001518 Standard SMPU STRUCT Slave Attribute value */
  __IOM uint32_t STANDARD_SMPU_STRUCT_MASTER_ATTR[16]; /*!< 0x00001558 Standard SMPU STRUCT Master Attribute value */
  __IOM uint32_t STANDARD_MPU_STRUCT[16];       /*!< 0x00001598 Standard MPU STRUCT */
  __IOM uint32_t STANDARD_PPU_STRUCT[16];       /*!< 0x000015D8 Standard PPU STRUCT */
  __IOM uint32_t CSP_PANEL_ID;                  /*!< 0x00001618 CSP Panel Id to record panel ID of CSP die */
   __IM uint32_t RESERVED4[121];
  __IOM uint16_t PILO_FREQ_STEP;                /*!< 0x00001800 Resolution step for PILO at class in BCD format */
   __IM uint16_t RESERVED5;
  __IOM uint32_t CSDV2_CSD0_ADC_VREF0;          /*!< 0x00001804 CSD 1p2 & 1p6 voltage levels for accuracy */
  __IOM uint32_t CSDV2_CSD0_ADC_VREF1;          /*!< 0x00001808 CSD 2p3 & 0p8 voltage levels for accuracy */
  __IOM uint32_t CSDV2_CSD0_ADC_VREF2;          /*!< 0x0000180C CSD calibration spare voltage level for accuracy */
   __IM uint32_t RESERVED6[508];
  __IOM uint32_t FLASH_BOOT_IDENTIFIER;         /*!< 0x00002000 Flash Boot Firmware Identifier */
  __IOM uint32_t FLASH_BOOT_ATTRIBUTE;          /*!< 0x00002004 N/A */
  __IOM uint32_t FLASH_BOOT_CMAC_ADDRESS;       /*!< 0x00002008 Flash boot CMAC address */
  __IOM uint32_t FLASH_BOOT_ADDRESS;            /*!< 0x0000200C Flash Boot Code Address */
  __IOM uint32_t FLASH_BOOT_NEXT_ADDRESS;       /*!< 0x00002010 Flash Boot Next Code Address to be executed */
  __IOM uint32_t FLASH_BOOT_ID_TYPE;            /*!< 0x00002014 Contains processor details */
  __IOM uint32_t FLASH_BOOT_VECTOR_TABLE[48];   /*!< 0x00002018 Flash Boot Vector Table */
  __IOM uint8_t  FLASH_BOOT_CODE[8192];         /*!< 0x000020D8 Flash Boot Code 8KB */
  __IOM uint8_t  FLASH_BOOT_CMAC[16];           /*!< 0x000040D8 Secure Hash/CMAC */
} SFLASH_Type;                                  /*!< Size = 16616 (0x40E8) */


/* SFLASH.SILICON_ID */
#define SFLASH_SILICON_ID_ID_Pos                0UL
#define SFLASH_SILICON_ID_ID_Msk                0xFFFFUL
/* SFLASH.DIE_LOT */
#define SFLASH_DIE_LOT_LOT_Pos                  0UL
#define SFLASH_DIE_LOT_LOT_Msk                  0xFFUL
/* SFLASH.DIE_WAFER */
#define SFLASH_DIE_WAFER_WAFER_Pos              0UL
#define SFLASH_DIE_WAFER_WAFER_Msk              0xFFUL
/* SFLASH.DIE_X */
#define SFLASH_DIE_X_X_Pos                      0UL
#define SFLASH_DIE_X_X_Msk                      0xFFUL
/* SFLASH.DIE_Y */
#define SFLASH_DIE_Y_Y_Pos                      0UL
#define SFLASH_DIE_Y_Y_Msk                      0xFFUL
/* SFLASH.DIE_SORT */
#define SFLASH_DIE_SORT_S1_PASS_Pos             0UL
#define SFLASH_DIE_SORT_S1_PASS_Msk             0x1UL
#define SFLASH_DIE_SORT_S2_PASS_Pos             1UL
#define SFLASH_DIE_SORT_S2_PASS_Msk             0x2UL
#define SFLASH_DIE_SORT_S3_PASS_Pos             2UL
#define SFLASH_DIE_SORT_S3_PASS_Msk             0x4UL
#define SFLASH_DIE_SORT_CRI_PASS_Pos            3UL
#define SFLASH_DIE_SORT_CRI_PASS_Msk            0x8UL
#define SFLASH_DIE_SORT_CHI_PASS_Pos            4UL
#define SFLASH_DIE_SORT_CHI_PASS_Msk            0x10UL
#define SFLASH_DIE_SORT_ENG_PASS_Pos            5UL
#define SFLASH_DIE_SORT_ENG_PASS_Msk            0x20UL
/* SFLASH.DIE_MINOR */
#define SFLASH_DIE_MINOR_MINOR_Pos              0UL
#define SFLASH_DIE_MINOR_MINOR_Msk              0xFFUL
/* SFLASH.DIE_DAY */
#define SFLASH_DIE_DAY_MINOR_Pos                0UL
#define SFLASH_DIE_DAY_MINOR_Msk                0xFFUL
/* SFLASH.DIE_MONTH */
#define SFLASH_DIE_MONTH_MINOR_Pos              0UL
#define SFLASH_DIE_MONTH_MINOR_Msk              0xFFUL
/* SFLASH.DIE_YEAR */
#define SFLASH_DIE_YEAR_MINOR_Pos               0UL
#define SFLASH_DIE_YEAR_MINOR_Msk               0xFFUL
/* SFLASH.BLE_DEVICE_ADDRESS */
#define SFLASH_BLE_DEVICE_ADDRESS_ADDR_Pos      0UL
#define SFLASH_BLE_DEVICE_ADDRESS_ADDR_Msk      0xFFFFFFFFUL
/* SFLASH.USER_FREE_ROW1 */
#define SFLASH_USER_FREE_ROW1_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW1_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.USER_FREE_ROW2 */
#define SFLASH_USER_FREE_ROW2_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW2_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.USER_FREE_ROW3 */
#define SFLASH_USER_FREE_ROW3_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW3_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.DEVICE_UID */
#define SFLASH_DEVICE_UID_DATA_Pos              0UL
#define SFLASH_DEVICE_UID_DATA_Msk              0xFFUL
/* SFLASH.MASTER_KEY */
#define SFLASH_MASTER_KEY_DATA_Pos              0UL
#define SFLASH_MASTER_KEY_DATA_Msk              0xFFUL
/* SFLASH.STANDARD_SMPU_STRUCT_SLAVE_ADDR */
#define SFLASH_STANDARD_SMPU_STRUCT_SLAVE_ADDR_DATA32_Pos 0UL
#define SFLASH_STANDARD_SMPU_STRUCT_SLAVE_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.STANDARD_SMPU_STRUCT_SLAVE_ATTR */
#define SFLASH_STANDARD_SMPU_STRUCT_SLAVE_ATTR_DATA32_Pos 0UL
#define SFLASH_STANDARD_SMPU_STRUCT_SLAVE_ATTR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.STANDARD_SMPU_STRUCT_MASTER_ATTR */
#define SFLASH_STANDARD_SMPU_STRUCT_MASTER_ATTR_DATA32_Pos 0UL
#define SFLASH_STANDARD_SMPU_STRUCT_MASTER_ATTR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.STANDARD_MPU_STRUCT */
#define SFLASH_STANDARD_MPU_STRUCT_DATA32_Pos   0UL
#define SFLASH_STANDARD_MPU_STRUCT_DATA32_Msk   0xFFFFFFFFUL
/* SFLASH.STANDARD_PPU_STRUCT */
#define SFLASH_STANDARD_PPU_STRUCT_DATA32_Pos   0UL
#define SFLASH_STANDARD_PPU_STRUCT_DATA32_Msk   0xFFFFFFFFUL
/* SFLASH.CSP_PANEL_ID */
#define SFLASH_CSP_PANEL_ID_DATA32_Pos          0UL
#define SFLASH_CSP_PANEL_ID_DATA32_Msk          0xFFFFFFFFUL
/* SFLASH.PILO_FREQ_STEP */
#define SFLASH_PILO_FREQ_STEP_STEP_Pos          0UL
#define SFLASH_PILO_FREQ_STEP_STEP_Msk          0xFFFFUL
/* SFLASH.CSDV2_CSD0_ADC_VREF0 */
#define SFLASH_CSDV2_CSD0_ADC_VREF0_VREF_HI_LEVELS_1P2_Pos 0UL
#define SFLASH_CSDV2_CSD0_ADC_VREF0_VREF_HI_LEVELS_1P2_Msk 0xFFFFUL
#define SFLASH_CSDV2_CSD0_ADC_VREF0_VREF_HI_LEVELS_1P6_Pos 16UL
#define SFLASH_CSDV2_CSD0_ADC_VREF0_VREF_HI_LEVELS_1P6_Msk 0xFFFF0000UL
/* SFLASH.CSDV2_CSD0_ADC_VREF1 */
#define SFLASH_CSDV2_CSD0_ADC_VREF1_VREF_HI_LEVELS_2P1_Pos 0UL
#define SFLASH_CSDV2_CSD0_ADC_VREF1_VREF_HI_LEVELS_2P1_Msk 0xFFFFUL
#define SFLASH_CSDV2_CSD0_ADC_VREF1_VREF_HI_LEVELS_0P8_Pos 16UL
#define SFLASH_CSDV2_CSD0_ADC_VREF1_VREF_HI_LEVELS_0P8_Msk 0xFFFF0000UL
/* SFLASH.CSDV2_CSD0_ADC_VREF2 */
#define SFLASH_CSDV2_CSD0_ADC_VREF2_VREF_HI_LEVELS_2P6_Pos 0UL
#define SFLASH_CSDV2_CSD0_ADC_VREF2_VREF_HI_LEVELS_2P6_Msk 0xFFFFUL
#define SFLASH_CSDV2_CSD0_ADC_VREF2_UNUSED_Pos  16UL
#define SFLASH_CSDV2_CSD0_ADC_VREF2_UNUSED_Msk  0xFFFF0000UL
/* SFLASH.FLASH_BOOT_IDENTIFIER */
#define SFLASH_FLASH_BOOT_IDENTIFIER_DATA32_Pos 0UL
#define SFLASH_FLASH_BOOT_IDENTIFIER_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_ATTRIBUTE */
#define SFLASH_FLASH_BOOT_ATTRIBUTE_DATA32_Pos  0UL
#define SFLASH_FLASH_BOOT_ATTRIBUTE_DATA32_Msk  0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_CMAC_ADDRESS */
#define SFLASH_FLASH_BOOT_CMAC_ADDRESS_DATA32_Pos 0UL
#define SFLASH_FLASH_BOOT_CMAC_ADDRESS_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_ADDRESS */
#define SFLASH_FLASH_BOOT_ADDRESS_DATA32_Pos    0UL
#define SFLASH_FLASH_BOOT_ADDRESS_DATA32_Msk    0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_NEXT_ADDRESS */
#define SFLASH_FLASH_BOOT_NEXT_ADDRESS_DATA32_Pos 0UL
#define SFLASH_FLASH_BOOT_NEXT_ADDRESS_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_ID_TYPE */
#define SFLASH_FLASH_BOOT_ID_TYPE_DATA32_Pos    0UL
#define SFLASH_FLASH_BOOT_ID_TYPE_DATA32_Msk    0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_VECTOR_TABLE */
#define SFLASH_FLASH_BOOT_VECTOR_TABLE_DATA32_Pos 0UL
#define SFLASH_FLASH_BOOT_VECTOR_TABLE_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_CODE */
#define SFLASH_FLASH_BOOT_CODE_DATA_Pos         0UL
#define SFLASH_FLASH_BOOT_CODE_DATA_Msk         0xFFUL
/* SFLASH.FLASH_BOOT_CMAC */
#define SFLASH_FLASH_BOOT_CMAC_DATA_Pos         0UL
#define SFLASH_FLASH_BOOT_CMAC_DATA_Msk         0xFFUL


#endif /* _CYIP_SFLASH_H_ */


/* [] END OF FILE */
