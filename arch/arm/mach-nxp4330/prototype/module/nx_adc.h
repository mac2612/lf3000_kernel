//------------------------------------------------------------------------------
//
//	Copyright (C) Nexell Co. 2012
//
//  This confidential and proprietary software may be used only as authorized by a
//  licensing agreement from Nexell Co.
//  The entire notice above must be reproduced on all authorized copies and copies
//  may only be made to the extent permitted by a licensing agreement from Nexell Co.
//
//	Module		:
//	File		: nx_adc.h
//	Description	:
//	Author		:
//	History		:
//------------------------------------------------------------------------------
#ifndef __NX_ADC_H__
#define __NX_ADC_H__

#include "../base/nx_prototype.h"

#ifdef	__cplusplus
extern "C"
{
#endif

//------------------------------------------------------------------------------
/// @defgroup	ADC
//------------------------------------------------------------------------------
//@{

//--------------------------------------------------------------------------
/// @brief	ADC Register Set Structure
//------------------------------------------------------------------------------
// ADC Register List
//    ADCCON
//       [15]     R     FLAG     End of conversion flag                        0
//       [14]     R/W   APEN     Prescaler enable                              0
//       [13:6]   R/W   APSV     A/D converter clock prescaler value [7:0]     ff
//       [5:3]    R/W   o_ASEL   A/D converter input select [2:0]              0
//       [2]      R/W   o_STBY   A/D converter standby mode                    1
//       [1]      R/W   ASRM     A/D conversion start-by-read mode             0
//       [0]      R/W   ADEN     A/D conversion start enable                   0
//    ADCDAT
//       [15:10]                    Reserved                                   0
//       [9:0]    R     ADCDAT      A/D converted data                         0
//    ADCINTENB
//       [15:1]                     Reserved                                   0
//       [0]      R/W   ADCIntrEnb  ADC Interrupt Enable                       0
//    ADCINTCLR
//       [15:1]                     Reserved                                   0
//       [0]      R/W   ADCIntrClr  ADC Interrupt Status & Clear               0
//------------------------------------------------------------------------------
typedef struct
{
	volatile U16	ADCCON;					///< 0x00 : ADC Control Register
	volatile U16	RESERVED_0;				///< 0x02 : RESERVED_0
	volatile U16	ADCDAT;					///< 0x04 : ADC Output Data Register
	volatile U16	RESERVED_1;				///< 0x06 : RESERVED_1
	volatile U16	ADCINTENB;				///< 0x08 : ADC Interrupt Enable Register
	volatile U16	RESERVED_2;				///< 0x0A : RESERVED_2
	volatile U16	ADCINTCLR;				///< 0x0C : ADC Interrutp Pending and Clear Register
} NX_ADC_RegisterSet;

//------------------------------------------------------------------------------
///	@name	ADC Interface
//------------------------------------------------------------------------------
//@{
void	NX_ADC_SetPrescalerValue( U32 ModuleIndex, U32 value );
U32		NX_ADC_GetPrescalerValue( U32 ModuleIndex );
void	NX_ADC_SetPrescalerEnable( U32 ModuleIndex, CBOOL enable );
CBOOL	NX_ADC_GetPrescalerEnable( U32 ModuleIndex );
void	NX_ADC_SetInputChannel( U32 ModuleIndex, U32 channel );
U32		NX_ADC_GetInputChannel( U32 ModuleIndex );
void	NX_ADC_SetStandbyMode( U32 ModuleIndex, CBOOL enable );
CBOOL	NX_ADC_GetStandbyMode( U32 ModuleIndex );
void	NX_ADC_Start( U32 ModuleIndex );
CBOOL	NX_ADC_IsBusy( U32 ModuleIndex );
U32		NX_ADC_GetConvertedData( U32 ModuleIndex );
//@}

//------------------------------------------------------------------------------
///	@name	Basic Interface
//------------------------------------------------------------------------------
//@{
CBOOL NX_ADC_Initialize( void );
U32   NX_ADC_GetNumberOfModule( void );

U32   NX_ADC_GetSizeOfRegisterSet( void );
void  NX_ADC_SetBaseAddress( U32 ModuleIndex, U32 BaseAddress );
U32   NX_ADC_GetBaseAddress( U32 ModuleIndex );
U32   NX_ADC_GetPhysicalAddress ( U32 ModuleIndex );
CBOOL NX_ADC_OpenModule( U32 ModuleIndex );
CBOOL NX_ADC_CloseModule( U32 ModuleIndex );
CBOOL NX_ADC_CheckBusy( U32 ModuleIndex );
//@}

//------------------------------------------------------------------------------
///	@name	reset Interface
//------------------------------------------------------------------------------
//@{
U32 NX_ADC_GetResetNumber ( U32 ModuleIndex );
//@}

//------------------------------------------------------------------------------
///	@name	Interrupt Interface
//------------------------------------------------------------------------------
//@{
/// @brief	interrupt index for IntNum
typedef enum
{
    NX_ADC_INT_EOC = 0, ///< ADC convert end interrupt.
} NX_ADC_INT;
U32   NX_ADC_GetInterruptNumber ( U32 ModuleIndex );
void  NX_ADC_SetInterruptEnable( U32 ModuleIndex, U32 IntNum, CBOOL Enable );
CBOOL NX_ADC_GetInterruptEnable( U32 ModuleIndex, U32 IntNum );
CBOOL NX_ADC_GetInterruptPending( U32 ModuleIndex, U32 IntNum );
void  NX_ADC_ClearInterruptPending( U32 ModuleIndex, U32 IntNum );
void  NX_ADC_SetInterruptEnableAll( U32 ModuleIndex, CBOOL Enable );
CBOOL NX_ADC_GetInterruptEnableAll( U32 ModuleIndex );
CBOOL NX_ADC_GetInterruptPendingAll( U32 ModuleIndex );
void  NX_ADC_ClearInterruptPendingAll( U32 ModuleIndex );
S32   NX_ADC_GetInterruptPendingNumber( U32 ModuleIndex );
//@}

#ifdef	__cplusplus
}
#endif


#endif // __NX_ADC_H__
