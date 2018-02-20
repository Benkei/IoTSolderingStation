// MAX31856.h

#ifndef _MAX31856_h
#define _MAX31856_h

#define MAX31856_CR0_REG           0x00
#define MAX31856_CR0_AUTOCONVERT   0x80
#define MAX31856_CR0_1SHOT         0x40
#define MAX31856_CR0_OCFAULT1      0x20
#define MAX31856_CR0_OCFAULT0      0x10
#define MAX31856_CR0_CJ            0x08
#define MAX31856_CR0_FAULT         0x04
#define MAX31856_CR0_FAULTCLR      0x02

#define MAX31856_CR1_REG           0x01
#define MAX31856_MASK_REG          0x02
#define MAX31856_CJHF_REG          0x03
#define MAX31856_CJLF_REG          0x04
#define MAX31856_LTHFTH_REG        0x05
#define MAX31856_LTHFTL_REG        0x06
#define MAX31856_LTLFTH_REG        0x07
#define MAX31856_LTLFTL_REG        0x08
#define MAX31856_CJTO_REG          0x09
#define MAX31856_CJTH_REG          0x0A
#define MAX31856_CJTL_REG          0x0B
#define MAX31856_LTCBH_REG         0x0C
#define MAX31856_LTCBM_REG         0x0D
#define MAX31856_LTCBL_REG         0x0E
#define MAX31856_SR_REG            0x0F

#define MAX31856_FAULT_CJRANGE     0x80
#define MAX31856_FAULT_TCRANGE     0x40
#define MAX31856_FAULT_CJHIGH      0x20
#define MAX31856_FAULT_CJLOW       0x10
#define MAX31856_FAULT_TCHIGH      0x08
#define MAX31856_FAULT_TCLOW       0x04
#define MAX31856_FAULT_OVUV        0x02
#define MAX31856_FAULT_OPEN        0x01

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


#include <SPI.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class MAX31856 {
public:
	MAX31856(int8_t spi_cs, SPIClass* spi);

	void Begin(void);

	void setThermocoupleType(MAX31856_ThermocoupleType type);

	MAX31856_ThermocoupleType getThermocoupleType(void);

	uint8_t readFault(void);

	void setColdJunctionFaultThreshholds(int8_t low, int8_t high);

	void setTempFaultThreshholds(float flow, float fhigh);

	void oneShotTemperature(void);

	float readCJTemperature(void);

	float readThermocoupleTemperature(void);


private:
	int8_t spi_cs;
	SPIClass* spi;
	uint8_t readRegister8(uint8_t addr);
	uint16_t readRegister16(uint8_t addr);
	uint32_t readRegister24(uint8_t addr);
	void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
	void writeRegister8(uint8_t addr, uint8_t data);
	uint8_t spixfer(uint8_t x);
};

#endif

