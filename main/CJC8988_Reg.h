/**************************************************************************
**   FileName: CJC8988_Reg.h                                              *
**     Author: tf                                                         *
**  Copyright: Wuhan GuangHuaXin Technology (C)2012. All rights reserved  *
**   HomePage: www.csc-ic.com                                             *
**       Date: 2012-03-15 13:58:09                                        *
**Description: The register address seek list of CJC8988                  *
***************************************************************************/
#ifndef CJC8988Reg_H_
#define CJC8988Reg_H_



#define     CJC8988_Write_Address                      0x1b
#define     CJC8988_Read_Address                       0x1b


/* register addr */

#define     CJC8988_REG_NUM                             0x100   // Total number of registers

#define     CJC8988_R0_LEFT_INPUT_VOLUME                0x01   // Audio input left channel volume; R/W
#define     CJC8988_R1_RIGHT_INPUT_VOLUME               0x03   // Audio input right channel volume; R/W
#define     CJC8988_R2_LOUT1_VOLUME                     0x05   // Audio output letf channel1 volume; R/W
#define     CJC8988_R3_ROUT1_VOLUME                     0x07   // Audio output right channel1 volume; R/W

#define     CJC8988_R5_ADC_DAC_CONTROL                  0x0A   // ADC and DAC CONTROL; R/W
#define     CJC8988_R7_AUDIO_INTERFACE                  0x0E   // Digital Audio interface format; R/W
#define     CJC8988_R8_SAMPLE_RATE                      0x10   // Clock and Sample rate control; R/W

#define     CJC8988_R10_LEFT_DAC_VOLUME                 0x15   // Left channel DAC digital volume; R/W
#define     CJC8988_R11_RIGHT_DAC_VOLUME                0x17   // Right channel DAC digital volume; R/W

#define     CJC8988_R12_BASS_CONTROL                    0x18   // BASS control; R/W
#define     CJC8988_R13_TREBLE_CONTROL                  0x1A   // Treble control; R/W
#define     CJC8988_R16_3D_CONTROL                      0x20   // 3D control; R/W

#define     CJC8988_R21_LEFT_ADC_VOLUME                 0x2B   // Left ADC digital volume; R/W
#define     CJC8988_R22_RIGHT_ADC_VOLUME                0x2D   // Right ADC digital volume; R/W

#define     CJC8988_R23_ADDITIONAL_CONTROL1             0x2E   // Additional control; R/W
#define     CJC8988_R24_ADDITIONAL_CONTROL2             0x30   // Additional control; R/W
#define     CJC8988_R27_ADDITIONAL_CONTROL3             0x36   // Additional control; R/W

#define     CJC8988_R31_ADC_INPUT_MODE                  0x3E   // ADC input mode; R/W
#define     CJC8988_R32_ADCL_SIGNAL_PATH                0x40   // Left ADC signal path control; R/W
#define     CJC8988_R33_ADCR_SIGNAL_PATH                0x42   // Right ADC signal path control; R/W

#define     CJC8988_R34_LEFT_OUT_MIX1_L                 0x44   // Left DAC to Left mixer disable; R/W
#define     CJC8988_R34_LEFT_OUT_MIX1_H                 0x45   // Left DAC to left mixer enable; R/W
#define     CJC8988_R35_LEFT_OUT_MIX2_L                 0x46   // Right DAC to left mixer disable; R/W
#define     CJC8988_R35_LEFT_OUT_MIX2_H                 0X47   // Right DAC to left mixer enable; R/W
#define     CJC8988_R36_RIGHT_OUT_MIX1_L                0x48   // Left DAC to Right mixer disable; R/W
#define     CJC8988_R36_RIGHT_OUT_MIX1_H                0x49   // Left DAC to Right mixer enable; R/W
#define     CJC8988_R37_RIGHT_OUT_MIX2_L                0x4A   // Right DAC to Right mixer disable; R/W
#define     CJC8988_R37_RIGHT_OUT_MIX2_H                0x4B   // Right DAC to Right mixer enable; R/W

#define     CJC8988_R40_LOUT2_VOLUME                    0x51   // Audio output left channel1 volume; R/W
#define     CJC8988_R41_ROUT2_VOLUME                    0x53   // Audio output right channel1 volume; R/W
#define     CJC8988_R43_LOW_POWER_PLAYBACK              0x86   // Low power playback; R/W

#define     CJC8988_R25_PWR_MGMT1_L                     0x32   // Power management1 and VMIDSEL; R/W
#define     CJC8988_R25_PWR_MGMT1_H                     0x33   // Power management1 and VMIDSEL; R/W
#define     CJC8988_R26_PWR_MGMT2_L                     0x34   // Power management2 and DAC left Power down; R/W
#define     CJC8988_R26_PWR_MGMT2_H                     0x35   // Power management2 and DAC right Power up; R/W
void CJC8988_DAC_TO_LOUT1(void);
void CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_8(void);
void CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_44(void);
#endif
