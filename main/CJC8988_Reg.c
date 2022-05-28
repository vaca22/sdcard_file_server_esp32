#include <driver/i2c.h>
#include "CJC8988_Reg.h"
unsigned char I2C_WriteReg(unsigned char Device_Addr, unsigned char Reg_Addr, unsigned char Reg_Data){
    uint8_t data[2];
    data[0]=Reg_Addr;
    data[1]=Reg_Data;
    ESP_ERROR_CHECK(i2c_master_write_to_device(0,Device_Addr,data,2,10));
    return 0;
}
unsigned char I2C_ReadReg(unsigned char Device_Addr, unsigned char Reg_Addr){
    return 0;
}
unsigned int Delay_ms(unsigned int t){
 vTaskDelay(t);
 return 0;
}

//*************************************************************************
// CJC8988 Wirte register operation
//
unsigned char CJC8988_WriteReg(unsigned char Reg_Addr, unsigned char Reg_Data)
{
    I2C_WriteReg(CJC8988_Write_Address, Reg_Addr, Reg_Data);
    return 0;
}

//*************************************************************************
// CJC8988 Read register operation
//
unsigned char CJC8988_ReadReg(unsigned char Reg_Addr)
{
    return(I2C_ReadReg(CJC8988_Read_Address, Reg_Addr));
}

//*************************************************************************
// CJC8988 Register Initial Value
//
struct CJC8988_reg{
    unsigned char reg_index;
    unsigned char reg_value;
    };

//*************************************************************************
// LINE Input 1 to ADC mode, slave mode, I2S format output, MCLK=12.288MHz, BCLK =3.072MHz, Fs=48KHz;
//
static struct CJC8988_reg    LINPUT1_TO_ADC[] = {
    {CJC8988_R0_LEFT_INPUT_VOLUME,       0x17},   // Audio input left channel volume; R/W
    {CJC8988_R1_RIGHT_INPUT_VOLUME,      0x17},   // Audio input right channel volume; R/W
    {CJC8988_R2_LOUT1_VOLUME,            0x79},   // Audio output letf channel1 volume; R/W
    {CJC8988_R3_ROUT1_VOLUME,            0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R5_ADC_DAC_CONTROL,         0x00},   // ADC and DAC CONTROL; R/W
    {CJC8988_R7_AUDIO_INTERFACE,         0x0A},   // Digital Audio interface format; R/W
    {CJC8988_R8_SAMPLE_RATE,             0x00},   // Clock and Sample rate control; R/W
    {CJC8988_R10_LEFT_DAC_VOLUME,        0xff},   // Left channel DAC digital volume; R/W
    {CJC8988_R11_RIGHT_DAC_VOLUME,       0xff},   // Right channel DAC digital volume; R/W

    {CJC8988_R12_BASS_CONTROL,           0x0f},   // BASS control; R/W
    {CJC8988_R13_TREBLE_CONTROL,         0x0f},   // Treble control; R/W
    {CJC8988_R16_3D_CONTROL,             0x00},   // 3D control; R/W
       
    {CJC8988_R21_LEFT_ADC_VOLUME,        0xc3},   // Left ADC digital volume; R/W
    {CJC8988_R22_RIGHT_ADC_VOLUME,       0xc3},   // Right ADC digital volume; R/W
       
    {CJC8988_R23_ADDITIONAL_CONTROL1,    0xc0},   // Additional control 1; R/W
    {CJC8988_R24_ADDITIONAL_CONTROL2,    0x04},   // Additional control 2; R/W
    {CJC8988_R27_ADDITIONAL_CONTROL3,    0x00},   // Additional control 3; R/W

    {CJC8988_R31_ADC_INPUT_MODE,         0x00},   // ADC input mode; R/W
    
    {CJC8988_R32_ADCL_SIGNAL_PATH,       0x00},   // Left ADC signal path control; R/W
    {CJC8988_R33_ADCR_SIGNAL_PATH,       0x00},   // Right ADC signal path control; R/W
  
    {CJC8988_R34_LEFT_OUT_MIX1_L,        0x52},   // Left DAC to left mixer disable; R/W
    {CJC8988_R35_LEFT_OUT_MIX2_L,        0x50},   // Right DAC to left mixer disable; R/W
    {CJC8988_R36_RIGHT_OUT_MIX1_L,       0x52},   // Left DAC to right mixer disable; R/W
    {CJC8988_R37_RIGHT_OUT_MIX2_L,       0x50},   // Right DAC to right mixer disable; R/W

    {CJC8988_R40_LOUT2_VOLUME,           0x79},   // Audio output left channel1 volume; R/W
    {CJC8988_R41_ROUT2_VOLUME,           0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R43_LOW_POWER_PLAYBACK,     0x00},   // Low power playback; R/W
        
    {CJC8988_R25_PWR_MGMT1_H,            0x7c},   // Power management1 and VMIDSEL; R/W
    {CJC8988_R26_PWR_MGMT2_L,            0x00},   // Power management2 and DAC left power down; R/W;
};
  
#define CJC8988_LINPUT1_TO_ADC_REG_NUM    29

//*************************************************************************
// DAC to LOUT1 mode, slave mode, I2S format input, MCLK=12.288MHz, BCLK =3.072MHz, Fs=48KHz;
//
static struct CJC8988_reg    DAC_TO_LOUT1[] = {
    {CJC8988_R0_LEFT_INPUT_VOLUME,       0x17},   // Audio input left channel volume; R/W
    {CJC8988_R1_RIGHT_INPUT_VOLUME,      0x17},   // Audio input right channel volume; R/W
    {CJC8988_R2_LOUT1_VOLUME,            0x79},   // Audio output letf channel1 volume; R/W
    {CJC8988_R3_ROUT1_VOLUME,            0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R5_ADC_DAC_CONTROL,         0x00},   // ADC and DAC CONTROL; R/W
    {CJC8988_R7_AUDIO_INTERFACE,         0x0A},   // Digital Audio interface format; R/W
    {CJC8988_R8_SAMPLE_RATE,             0x00},   // Clock and Sample rate control; R/W
    {CJC8988_R10_LEFT_DAC_VOLUME,        0xff},   // Left channel DAC digital volume; R/W
    {CJC8988_R11_RIGHT_DAC_VOLUME,       0xff},   // Right channel DAC digital volume; R/W

    {CJC8988_R12_BASS_CONTROL,           0x0f},   // BASS control; R/W
    {CJC8988_R13_TREBLE_CONTROL,         0x0f},   // Treble control; R/W
    {CJC8988_R16_3D_CONTROL,             0x00},   // 3D control; R/W
       
    {CJC8988_R21_LEFT_ADC_VOLUME,        0xc3},   // Left ADC digital volume; R/W
    {CJC8988_R22_RIGHT_ADC_VOLUME,       0xc3},   // Right ADC digital volume; R/W
       
    {CJC8988_R23_ADDITIONAL_CONTROL1,    0xc0},   // Additional control 1; R/W
    {CJC8988_R24_ADDITIONAL_CONTROL2,    0x00},   // Additional control 2; R/W
    {CJC8988_R27_ADDITIONAL_CONTROL3,    0x00},   // Additional control 3; R/W

    {CJC8988_R31_ADC_INPUT_MODE,         0x00},   // ADC input mode; R/W
    
    {CJC8988_R32_ADCL_SIGNAL_PATH,       0x00},   // Left ADC signal path control; R/W
    {CJC8988_R33_ADCR_SIGNAL_PATH,       0x00},   // Right ADC signal path control; R/W
  
    {CJC8988_R34_LEFT_OUT_MIX1_H,        0x52},   // Left DAC to left mixer enable; R/W
    {CJC8988_R35_LEFT_OUT_MIX2_L,        0x50},   // Right DAC to left mixer disable; R/W
    {CJC8988_R36_RIGHT_OUT_MIX1_L,       0x52},   // Left DAC to right mixer disable; R/W
    {CJC8988_R37_RIGHT_OUT_MIX2_H,       0x50},   // Right DAC to right mixer enable; R/W

    {CJC8988_R40_LOUT2_VOLUME,           0x79},   // Audio output left channel1 volume; R/W
    {CJC8988_R41_ROUT2_VOLUME,           0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R43_LOW_POWER_PLAYBACK,     0x08},   // Low power playback; R/W
        
    {CJC8988_R25_PWR_MGMT1_H,            0x40},   // Power management1 and VMIDSEL; R/W
    {CJC8988_R26_PWR_MGMT2_H,            0xf8},   // Power management2 and DAC left power up; R/W;
};
  
#define CJC8988_DAC_TO_LOUT1_REG_NUM    29

//*************************************************************************
// LINE Input 1 to LOUT 1 mode, slave mode, I2S format, MCLK=12.288MHz, BCLK =3.072MHz, Fs=48KHz;
//
static struct CJC8988_reg    LINPUT1_TO_LOUT1[] = {
    {CJC8988_R0_LEFT_INPUT_VOLUME,       0x17},   // Audio input left channel volume; R/W
    {CJC8988_R1_RIGHT_INPUT_VOLUME,      0x17},   // Audio input right channel volume; R/W
    {CJC8988_R2_LOUT1_VOLUME,            0x79},   // Audio output letf channel1 volume; R/W
    {CJC8988_R3_ROUT1_VOLUME,            0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R5_ADC_DAC_CONTROL,         0x00},   // ADC and DAC CONTROL; R/W
    {CJC8988_R7_AUDIO_INTERFACE,         0x0A},   // Digital Audio interface format; R/W
    {CJC8988_R8_SAMPLE_RATE,             0x00},   // Clock and Sample rate control; R/W
    {CJC8988_R10_LEFT_DAC_VOLUME,        0xff},   // Left channel DAC digital volume; R/W
    {CJC8988_R11_RIGHT_DAC_VOLUME,       0xff},   // Right channel DAC digital volume; R/W

    {CJC8988_R12_BASS_CONTROL,           0x0f},   // BASS control; R/W
    {CJC8988_R13_TREBLE_CONTROL,         0x0f},   // Treble control; R/W
    {CJC8988_R16_3D_CONTROL,             0x00},   // 3D control; R/W
       
    {CJC8988_R21_LEFT_ADC_VOLUME,        0xc3},   // Left ADC digital volume; R/W
    {CJC8988_R22_RIGHT_ADC_VOLUME,       0xc3},   // Right ADC digital volume; R/W
       
    {CJC8988_R23_ADDITIONAL_CONTROL1,    0xc0},   // Additional control 1; R/W
    {CJC8988_R24_ADDITIONAL_CONTROL2,    0x00},   // Additional control 2; R/W
    {CJC8988_R27_ADDITIONAL_CONTROL3,    0x00},   // Additional control 3; R/W

    {CJC8988_R31_ADC_INPUT_MODE,         0x00},   // ADC input mode; R/W
    
    {CJC8988_R32_ADCL_SIGNAL_PATH,       0x00},   // Left ADC signal path control; R/W
    {CJC8988_R33_ADCR_SIGNAL_PATH,       0x00},   // Right ADC signal path control; R/W
  
    {CJC8988_R34_LEFT_OUT_MIX1_L,        0xA0},   // Left DAC to left mixer disable; R/W
    {CJC8988_R35_LEFT_OUT_MIX2_L,        0x50},   // Right DAC to left mixer disable; R/W
    {CJC8988_R36_RIGHT_OUT_MIX1_L,       0x50},   // Left DAC to right mixer disable; R/W
    {CJC8988_R37_RIGHT_OUT_MIX2_L,       0xA0},   // Right DAC to right mixer disable; R/W

    {CJC8988_R40_LOUT2_VOLUME,           0x79},   // Audio output left channel1 volume; R/W
    {CJC8988_R41_ROUT2_VOLUME,           0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R43_LOW_POWER_PLAYBACK,     0x00},   // Low power playback; R/W
        
    {CJC8988_R25_PWR_MGMT1_H,            0x40},   // Power management1 and VMIDSEL; R/W
    {CJC8988_R26_PWR_MGMT2_H,            0x78},   // Power management2 and DAC left power up; R/W;
};
  
#define CJC8988_LINPUT1_TO_LOUT1_REG_NUM    29

//*************************************************************************
// LINE Input 2 to ADC and DAC to LOUT 2 mode, slave mode, I2S format, MCLK=12MHz, BCLK = 12MHz, Fs=8KHz;
//
static struct CJC8988_reg    LINPUT2_ADC_TO_DAC_LOUT2_8[] = {
    {CJC8988_R0_LEFT_INPUT_VOLUME,       0x17},   // Audio input left channel volume; R/W
    {CJC8988_R1_RIGHT_INPUT_VOLUME,      0x17},   // Audio input right channel volume; R/W
    {CJC8988_R2_LOUT1_VOLUME,            0x79},   // Audio output letf channel1 volume; R/W
    {CJC8988_R3_ROUT1_VOLUME,            0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R5_ADC_DAC_CONTROL,         0x00},   // ADC and DAC CONTROL; R/W
    {CJC8988_R7_AUDIO_INTERFACE,         0x02},   // Digital Audio interface format; R/W
    {CJC8988_R8_SAMPLE_RATE,             0x0d},   // Clock and Sample rate control; R/W
    {CJC8988_R10_LEFT_DAC_VOLUME,        0xff},   // Left channel DAC digital volume; R/W
    {CJC8988_R11_RIGHT_DAC_VOLUME,       0xff},   // Right channel DAC digital volume; R/W

    {CJC8988_R12_BASS_CONTROL,           0x0f},   // BASS control; R/W
    {CJC8988_R13_TREBLE_CONTROL,         0x0f},   // Treble control; R/W
    {CJC8988_R16_3D_CONTROL,             0x00},   // 3D control; R/W
       
    {CJC8988_R21_LEFT_ADC_VOLUME,        0xc3},   // Left ADC digital volume; R/W
    {CJC8988_R22_RIGHT_ADC_VOLUME,       0xc3},   // Right ADC digital volume; R/W
       
    {CJC8988_R23_ADDITIONAL_CONTROL1,    0xc0},   // Additional control 1; R/W
    {CJC8988_R24_ADDITIONAL_CONTROL2,    0x00},   // Additional control 2; R/W
    {CJC8988_R27_ADDITIONAL_CONTROL3,    0x00},   // Additional control 3; R/W

    {CJC8988_R31_ADC_INPUT_MODE,         0x00},   // ADC input mode; R/W
    
    {CJC8988_R32_ADCL_SIGNAL_PATH,       0x40},   // Left ADC signal path control; R/W
    {CJC8988_R33_ADCR_SIGNAL_PATH,       0x40},   // Right ADC signal path control; R/W
  
    {CJC8988_R34_LEFT_OUT_MIX1_H,        0x52},   // Left DAC to left mixer disable; R/W
    {CJC8988_R35_LEFT_OUT_MIX2_L,        0x50},   // Right DAC to left mixer disable; R/W
    {CJC8988_R36_RIGHT_OUT_MIX1_L,       0x52},   // Left DAC to right mixer disable; R/W
    {CJC8988_R37_RIGHT_OUT_MIX2_H,       0x50},   // Right DAC to right mixer disable; R/W

    {CJC8988_R40_LOUT2_VOLUME,           0x79},   // Audio output left channel1 volume; R/W
    {CJC8988_R41_ROUT2_VOLUME,           0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R43_LOW_POWER_PLAYBACK,     0x08},   // Low power playback; R/W
        
    {CJC8988_R25_PWR_MGMT1_H,            0x7C},   // Power management1 and VMIDSEL; R/W
    {CJC8988_R26_PWR_MGMT2_H,            0xf8},   // Power management2 and DAC left power up; R/W;
};
  
#define CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_8_REG_NUM    29

//*************************************************************************
// LINE Input 2 to ADC and DAC to LOUT 2 mode, slave mode, I2S format, MCLK=12MHz, BCLK = 12MHz, Fs=32KHz;
//
static struct CJC8988_reg    LINPUT2_ADC_TO_DAC_LOUT2_32[] = {
    {CJC8988_R0_LEFT_INPUT_VOLUME,       0x17},   // Audio input left channel volume; R/W
    {CJC8988_R1_RIGHT_INPUT_VOLUME,      0x17},   // Audio input right channel volume; R/W
    {CJC8988_R2_LOUT1_VOLUME,            0x79},   // Audio output letf channel1 volume; R/W
    {CJC8988_R3_ROUT1_VOLUME,            0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R5_ADC_DAC_CONTROL,         0x00},   // ADC and DAC CONTROL; R/W
    {CJC8988_R7_AUDIO_INTERFACE,         0x02},   // Digital Audio interface format; R/W
    {CJC8988_R8_SAMPLE_RATE,             0x19},   // Clock and Sample rate control; R/W
    {CJC8988_R10_LEFT_DAC_VOLUME,        0xff},   // Left channel DAC digital volume; R/W
    {CJC8988_R11_RIGHT_DAC_VOLUME,       0xff},   // Right channel DAC digital volume; R/W

    {CJC8988_R12_BASS_CONTROL,           0x0f},   // BASS control; R/W
    {CJC8988_R13_TREBLE_CONTROL,         0x0f},   // Treble control; R/W
    {CJC8988_R16_3D_CONTROL,             0x00},   // 3D control; R/W
       
    {CJC8988_R21_LEFT_ADC_VOLUME,        0xc3},   // Left ADC digital volume; R/W
    {CJC8988_R22_RIGHT_ADC_VOLUME,       0xc3},   // Right ADC digital volume; R/W
       
    {CJC8988_R23_ADDITIONAL_CONTROL1,    0xc0},   // Additional control 1; R/W
    {CJC8988_R24_ADDITIONAL_CONTROL2,    0x00},   // Additional control 2; R/W
    {CJC8988_R27_ADDITIONAL_CONTROL3,    0x00},   // Additional control 3; R/W

    {CJC8988_R31_ADC_INPUT_MODE,         0x00},   // ADC input mode; R/W
    
    {CJC8988_R32_ADCL_SIGNAL_PATH,       0x40},   // Left ADC signal path control; R/W
    {CJC8988_R33_ADCR_SIGNAL_PATH,       0x40},   // Right ADC signal path control; R/W
  
    {CJC8988_R34_LEFT_OUT_MIX1_H,        0x52},   // Left DAC to left mixer disable; R/W
    {CJC8988_R35_LEFT_OUT_MIX2_L,        0x50},   // Right DAC to left mixer disable; R/W
    {CJC8988_R36_RIGHT_OUT_MIX1_L,       0x52},   // Left DAC to right mixer disable; R/W
    {CJC8988_R37_RIGHT_OUT_MIX2_H,       0x50},   // Right DAC to right mixer disable; R/W

    {CJC8988_R40_LOUT2_VOLUME,           0x79},   // Audio output left channel1 volume; R/W
    {CJC8988_R41_ROUT2_VOLUME,           0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R43_LOW_POWER_PLAYBACK,     0x08},   // Low power playback; R/W
        
    {CJC8988_R25_PWR_MGMT1_H,            0x7C},   // Power management1 and VMIDSEL; R/W
    {CJC8988_R26_PWR_MGMT2_H,            0xf8},   // Power management2 and DAC left power up; R/W;
};
  
#define CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_32_REG_NUM    29

//*************************************************************************
// LINE Input 2 to ADC and DAC to LOUT 2 mode, slave mode, I2S format, MCLK=12MHz, BCLK = 12MHz, Fs=44.1KHz;
//
static struct CJC8988_reg    LINPUT2_ADC_TO_DAC_LOUT2_44[] = {
    {CJC8988_R0_LEFT_INPUT_VOLUME,       0x17},   // Audio input left channel volume; R/W
    {CJC8988_R1_RIGHT_INPUT_VOLUME,      0x17},   // Audio input right channel volume; R/W
    {CJC8988_R2_LOUT1_VOLUME,            0x79},   // Audio output letf channel1 volume; R/W
    {CJC8988_R3_ROUT1_VOLUME,            0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R5_ADC_DAC_CONTROL,         0x00},   // ADC and DAC CONTROL; R/W
    {CJC8988_R7_AUDIO_INTERFACE,         0x02},   // Digital Audio interface format; R/W
    {CJC8988_R8_SAMPLE_RATE,             0x23},   // Clock and Sample rate control; R/W
    {CJC8988_R10_LEFT_DAC_VOLUME,        0xff},   // Left channel DAC digital volume; R/W
    {CJC8988_R11_RIGHT_DAC_VOLUME,       0xff},   // Right channel DAC digital volume; R/W

    {CJC8988_R12_BASS_CONTROL,           0x0f},   // BASS control; R/W
    {CJC8988_R13_TREBLE_CONTROL,         0x0f},   // Treble control; R/W
    {CJC8988_R16_3D_CONTROL,             0x00},   // 3D control; R/W
       
    {CJC8988_R21_LEFT_ADC_VOLUME,        0xc3},   // Left ADC digital volume; R/W
    {CJC8988_R22_RIGHT_ADC_VOLUME,       0xc3},   // Right ADC digital volume; R/W
       
    {CJC8988_R23_ADDITIONAL_CONTROL1,    0xc0},   // Additional control 1; R/W
    {CJC8988_R24_ADDITIONAL_CONTROL2,    0x00},   // Additional control 2; R/W
    {CJC8988_R27_ADDITIONAL_CONTROL3,    0x00},   // Additional control 3; R/W

    {CJC8988_R31_ADC_INPUT_MODE,         0x00},   // ADC input mode; R/W
    
    {CJC8988_R32_ADCL_SIGNAL_PATH,       0x40},   // Left ADC signal path control; R/W
    {CJC8988_R33_ADCR_SIGNAL_PATH,       0x40},   // Right ADC signal path control; R/W
  
    {CJC8988_R34_LEFT_OUT_MIX1_H,        0x52},   // Left DAC to left mixer disable; R/W
    {CJC8988_R35_LEFT_OUT_MIX2_L,        0x50},   // Right DAC to left mixer disable; R/W
    {CJC8988_R36_RIGHT_OUT_MIX1_L,       0x52},   // Left DAC to right mixer disable; R/W
    {CJC8988_R37_RIGHT_OUT_MIX2_H,       0x50},   // Right DAC to right mixer disable; R/W

    {CJC8988_R40_LOUT2_VOLUME,           0x79},   // Audio output left channel1 volume; R/W
    {CJC8988_R41_ROUT2_VOLUME,           0x79},   // Audio output right channel1 volume; R/W
    {CJC8988_R43_LOW_POWER_PLAYBACK,     0x08},   // Low power playback; R/W
        
    {CJC8988_R25_PWR_MGMT1_H,            0x7C},   // Power management1 and VMIDSEL; R/W
    {CJC8988_R26_PWR_MGMT2_H,            0xf8},   // Power management2 and DAC left power up; R/W;
};
  
#define CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_44_REG_NUM    29

//*************************************************************************
// CJC8988 LINE Input 1 to ADC mode, slave mode, I2S format output, MCLK=12.288MHz, BCLK =3.072MHz, Fs=48KHz;
//
void CJC8988_LINPUT1_TO_ADC(void)
{
    int i;
    for (i = 0; i < CJC8988_LINPUT1_TO_ADC_REG_NUM; i++)
    {    
        CJC8988_WriteReg(LINPUT1_TO_ADC[i].reg_index, LINPUT1_TO_ADC[i].reg_value);
    }    
}

//*************************************************************************
// CJC8988 DAC to LOUT1, slave mode, I2S format output, MCLK=12.288MHz, BCLK =3.072MHz, Fs=48KHz;
//
void CJC8988_DAC_TO_LOUT1(void)
{
    int i;
    for (i = 0; i < CJC8988_DAC_TO_LOUT1_REG_NUM; i++)
    {    
        CJC8988_WriteReg(DAC_TO_LOUT1[i].reg_index, DAC_TO_LOUT1[i].reg_value);
    }    
}

//*************************************************************************
// LINE Input 1 to LOUT 1 mode, slave mode, I2S format output, MCLK=12.288MHz, BCLK =3.072MHz, Fs=48KHz;
//
void CJC8988_LINPUT1_TO_LOUT1(void)
{
    int i;
    for (i = 0; i < CJC8988_LINPUT1_TO_LOUT1_REG_NUM; i++)
    {    
        CJC8988_WriteReg(LINPUT1_TO_LOUT1[i].reg_index, LINPUT1_TO_LOUT1[i].reg_value);
    }    
}

//*************************************************************************
// CJC8988 LINE Input 2 to ADC and DAC to LOUT 2 mode, slave mode, I2S format, MCLK=12MHz, BCLK =12MHz, Fs=8KHz;
//
void CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_8(void)
{
    int i;
    for (i = 0; i < CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_8_REG_NUM; i++)
    {    
        CJC8988_WriteReg(LINPUT2_ADC_TO_DAC_LOUT2_8[i].reg_index, LINPUT2_ADC_TO_DAC_LOUT2_8[i].reg_value);
    }    
}

//*************************************************************************
// CJC8988 LINE Input 2 to ADC and DAC to LOUT 2 mode, slave mode, I2S format, MCLK=12MHz, BCLK =12MHz, Fs=32KHz;
//
void CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_32(void)
{
    int i;
    for (i = 0; i < CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_32_REG_NUM; i++)
    {    
        CJC8988_WriteReg(LINPUT2_ADC_TO_DAC_LOUT2_32[i].reg_index, LINPUT2_ADC_TO_DAC_LOUT2_32[i].reg_value);
    }    
}

//*************************************************************************
// CJC8988 LINE Input 2 to ADC and DAC to LOUT 2 mode, slave mode, I2S format, MCLK=12MHz, BCLK =12MHz, Fs=44.1KHz;
//
void CJC8988_LINPUT2_ADC_TO_DAC_LOUT2_44(void)
{
    int i;
    for (i = 0; i < 29; i++)
    {    
        CJC8988_WriteReg(LINPUT2_ADC_TO_DAC_LOUT2_44[i].reg_index, LINPUT2_ADC_TO_DAC_LOUT2_44[i].reg_value);
    }    
}






