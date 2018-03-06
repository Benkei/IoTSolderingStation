/********************************************************************
* @file MAX31856.h
*
* @author Devin Alexander
*
* @version 1.0
*
* Started: SEPTEMBER 14th 2017
*
* Updated:
*
* @brief Header file for MAX31856 class
*
***********************************************************************
*
* @copyright
* Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/

#ifndef _MAX31856_h
#define _MAX31856_h

//*****************************************************************************
//Define all the addresses of the registers in the MAX31856
//*****************************************************************************
#define ADDRESS_CR0_READ                   0x00         //Factory Default 00h
#define ADDRESS_CR0_WRITE                  0x80
#define ADDRESS_CR1_READ                   0x01         //Factory Default 03h
#define ADDRESS_CR1_WRITE                  0x81
#define ADDRESS_MASK_READ                  0x02         //Factory Default FFh
#define ADDRESS_MASK_WRITE                 0x82
#define ADDRESS_CJHF_READ                  0x03         //Factory Default 7Fh
#define ADDRESS_CJHF_WRITE                 0x83
#define ADDRESS_CJLF_READ                  0x04         //Factory Default C0h
#define ADDRESS_CJLF_WRITE                 0x84
#define ADDRESS_LTHFTH_READ                0x05         //Factory Default 7Fh
#define ADDRESS_LTHFTH_WRITE               0x85
#define ADDRESS_LTHFTL_READ                0x06         //Factory Default FFh
#define ADDRESS_LTHFTL_WRITE               0x86
#define ADDRESS_LTLFTH_READ                0x07         //Factory Default 80h
#define ADDRESS_LTLFTH_WRITE               0x87
#define ADDRESS_LTLFTL_READ                0x08         //Factory Default 00h
#define ADDRESS_LTLFTL_WRITE               0x88
#define ADDRESS_CJTO_READ                  0x09         //Factory Default 00h
#define ADDRESS_CJTO_WRITE                 0x89
#define ADDRESS_CJTH_READ                  0x0A         //Factory Default 00h
#define ADDRESS_CJTH_WRITE                 0x8A
#define ADDRESS_CJTL_READ                  0x0B         //Factory Default 00h
#define ADDRESS_CJTL_WRITE                 0x8B
#define ADDRESS_LTCBH_READ                 0x0C
#define ADDRESS_LTCBM_READ                 0x0D
#define ADDRESS_LTCBL_READ                 0x0E
#define ADDRESS_SR_READ                    0x0F



#define MAX31856_CR0_REG              0x00
#define MAX31856_CR0_AUTOCONVERT      0x80
#define MAX31856_CR0_1SHOT            0x40
#define MAX31856_CR0_OCFAULT2         0x30
#define MAX31856_CR0_OCFAULT1         0x20
#define MAX31856_CR0_OCFAULT0         0x10
#define MAX31856_CR0_OCFAULT_DISABLED 0x00
#define MAX31856_CR0_CJ               0x08
#define MAX31856_CR0_FAULT            0x04
#define MAX31856_CR0_FAULTCLR         0x02

#define MAX31856_CR1_REG              0x01
#define MAX31856_MASK_REG             0x02
#define MAX31856_CJHF_REG             0x03
#define MAX31856_CJLF_REG             0x04
#define MAX31856_LTHFTH_REG           0x05
#define MAX31856_LTHFTL_REG           0x06
#define MAX31856_LTLFTH_REG           0x07
#define MAX31856_LTLFTL_REG           0x08
#define MAX31856_CJTO_REG             0x09
#define MAX31856_CJTH_REG             0x0A
#define MAX31856_CJTL_REG             0x0B
#define MAX31856_LTCBH_REG            0x0C
#define MAX31856_LTCBM_REG            0x0D
#define MAX31856_LTCBL_REG            0x0E
#define MAX31856_SR_REG               0x0F

#define MAX31856_FAULT_CJRANGE        0x80
#define MAX31856_FAULT_TCRANGE        0x40

#define MAX31856_FAULT_CJHIGH         0x20
#define MAX31856_FAULT_CJLOW          0x10
#define MAX31856_FAULT_TCHIGH         0x08
#define MAX31856_FAULT_TCLOW          0x04
#define MAX31856_FAULT_OVUV           0x02
#define MAX31856_FAULT_OPEN           0x01

//*****************************************************************************    
//Define parameters for control register one (CR1)
//*****************************************************************************    
#define MAX31856_CR1_AVG_TC_SAMPLES_1               0x00		//Power on default value
#define MAX31856_CR1_AVG_TC_SAMPLES_2               0x10
#define MAX31856_CR1_AVG_TC_SAMPLES_4               0x20
#define MAX31856_CR1_AVG_TC_SAMPLES_8               0x30
#define MAX31856_CR1_AVG_TC_SAMPLES_16              0x40


enum MAX31856_ThermocoupleType
{
	TCTYPE_B = 0b0000,
	TCTYPE_E = 0b0001,
	TCTYPE_J = 0b0010,
	TCTYPE_K = 0b0011,
	TCTYPE_N = 0b0100,
	TCTYPE_R = 0b0101,
	TCTYPE_S = 0b0110,
	TCTYPE_T = 0b0111,
	VMODE_G8 = 0b1000,
	VMODE_G32 = 0b1100,
};

// Register 00h/80h: Configuration 0 Register (CR0)
enum CR0
{
	CONV_MODE_NORMALLY_OFF = 0x00,    //Power On Default value
	CONV_MODE_NORMALLY_ON = 0x80,

	ONE_SHOT_MODE_NO_CONVERSION = 0x00,    //defaults to this value
	ONE_SHOT_MODE_ONE_CONVERSION = 0x40,    //^

	OC_DETECT_DISABLED = 0x00,
	OC_DETECT_ENABLED_R_LESS_5k = 0x10,
	OC_DETECT_ENABLED_TC_LESS_2ms = 0x20,
	OC_DETECT_ENABLED_TC_MORE_2ms = 0x30,

	COLD_JUNC_ENABLE = 0x00,    //Power On Default value
	COLD_JUNC_DISABLE = 0x08,    //speed of conversion is speed up by 25ms when this optionis selected (Disable the cold junc)

	FAULT_MODE_COMPARATOR = 0x00,    //Power On Default value
	FAULT_MODE_INTERUPT = 0x04,

	FAULTCLR_DEFAULT_VAL = 0x00,    //defaults to this value
	FAULTCLR_RETURN_FAULTS_TO_ZERO = 0x02,    //^

	FILTER_OUT_60Hz = 0x00,    //Preset value
	FILTER_OUT_50Hz = 0x01,    //^
};
enum CR1
{
	AVG_TC_SAMPLES_1 = 0x00,
	AVG_TC_SAMPLES_2 = 0x10,
	AVG_TC_SAMPLES_4 = 0x20,
	AVG_TC_SAMPLES_8 = 0x30,
	AVG_TC_SAMPLES_16 = 0x40,

	TC_TYPE_B = 0x00,
	TC_TYPE_E = 0x01,
	TC_TYPE_J = 0x02,
	TC_TYPE_K = 0x03,
	TC_TYPE_N = 0x04,
	TC_TYPE_R = 0x05,
	TC_TYPE_S = 0x06,
	TC_TYPE_T = 0x07,
	TC_TYPE_VOLT_MODE_GAIN_8 = 0x08,
	TC_TYPE_VOLT_MODE_GAIN_32 = 0x0C,
};
enum ConversionMode
{
	Off = 0b00000000,
	Automatic = 0b01000000,
	OneShot = 0b10000000,
	_MASK = 0b11000000,
};


#include <SPI.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

/**
* MAX31856 Class
*/
class MAX31856 {
public:
	MAX31856(int8_t spi_cs, SPIClass& spi);

	void begin(void);

	void SetConfiguration0Flags(CR0 flags);
	void SetConfiguration1Flags(CR1 flags);


	int32_t GetConversionTime();

	void SetConversionMode(ConversionMode mode);
	ConversionMode GetConversionMode();

	void setThermocoupleType(MAX31856_ThermocoupleType type);

	MAX31856_ThermocoupleType getThermocoupleType(void);

	uint8_t readFault(void);

	void setColdJunctionFaultThreshholds(int8_t low, int8_t high);

	void setTempFaultThreshholds(float flow, float fhigh);

	void oneShotTemperature(void);

	float readCJTemperature(void);

	float readTCTemperature(void);

	bool setColdJunctionOffset(float temperature);

private:
	int8_t spi_cs;
	SPIClass& spi;
	CR0 config0;
	CR1 config1;
	int32_t conversionTime;
	uint32_t conversionCount;
	uint32_t lastReadTime;

	uint8_t readRegister8(uint8_t addr);
	uint16_t readRegister16(uint8_t addr);
	uint32_t readRegister24(uint8_t addr);
	void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
	void writeRegister8(uint8_t addr, uint8_t data);
	void calculateDelayTime();
};

#endif

