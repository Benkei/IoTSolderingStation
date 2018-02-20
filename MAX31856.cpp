// 
// 
// 

#include "MAX31856.h"

static SPISettings max31856_spisettings = SPISettings(500000, MSBFIRST, SPI_MODE1);


MAX31856::MAX31856(int8_t spi_cs, SPIClass* spi) {
	this->spi_cs = spi_cs;
	this->spi = spi;
}

void MAX31856::Begin() {
	pinMode(spi_cs, OUTPUT);
	digitalWrite(spi_cs, HIGH);

	Serial.println("start spi");
	//start and configure hardware SPI
	spi->begin();

	Serial.println("set reg");
	// assert on any fault
	writeRegister8(MAX31856_MASK_REG, 0x0);

	writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);

	Serial.println("set type");
	setThermocoupleType(TCTYPE_K);
}


void MAX31856::setThermocoupleType(MAX31856_ThermocoupleType type) {
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0xF0; // mask off bottom 4 bits
	t |= (uint8_t)type & 0x0F;
	writeRegister8(MAX31856_CR1_REG, t);
}

MAX31856_ThermocoupleType MAX31856::getThermocoupleType(void) {
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0x0F;

	return (MAX31856_ThermocoupleType)(t);
}

uint8_t MAX31856::readFault(void) {
	return readRegister8(MAX31856_SR_REG);
}

void MAX31856::setColdJunctionFaultThreshholds(int8_t low, int8_t high) {
	writeRegister8(MAX31856_CJLF_REG, low);
	writeRegister8(MAX31856_CJHF_REG, high);
}

void MAX31856::setTempFaultThreshholds(float flow, float fhigh) {
	int16_t low, high;

	flow *= 16;
	low = flow;

	fhigh *= 16;
	high = fhigh;

	writeRegister8(MAX31856_LTHFTH_REG, high >> 8);
	writeRegister8(MAX31856_LTHFTL_REG, high);

	writeRegister8(MAX31856_LTLFTH_REG, low >> 8);
	writeRegister8(MAX31856_LTLFTL_REG, low);
}

void MAX31856::oneShotTemperature(void) {

	writeRegister8(MAX31856_CJTO_REG, 0x0);

	uint8_t t = readRegister8(MAX31856_CR0_REG);

	t &= ~MAX31856_CR0_AUTOCONVERT; // turn off autoconvert!
	t |= MAX31856_CR0_1SHOT;

	writeRegister8(MAX31856_CR0_REG, t);

	delay(250); // MEME FIX autocalculate based on oversampling
}

float MAX31856::readCJTemperature(void) {
	oneShotTemperature();

	int16_t temp16 = readRegister16(MAX31856_CJTH_REG);
	float tempfloat = temp16;
	tempfloat /= 256.0;

	return tempfloat;
}

float MAX31856::readThermocoupleTemperature(void) {
	oneShotTemperature();

	int32_t temp24 = readRegister24(MAX31856_LTCBH_REG);
	if (temp24 & 0x800000) {
		temp24 |= 0xFF000000;  // fix sign
	}

	temp24 >>= 5;  // bottom 5 bits are unused

	float tempfloat = temp24;
	tempfloat *= 0.0078125;

	return tempfloat;
}


/**********************************************/

uint8_t MAX31856::readRegister8(uint8_t addr) {
	uint8_t ret = 0;
	readRegisterN(addr, &ret, 1);

	return ret;
}

uint16_t MAX31856::readRegister16(uint8_t addr) {
	uint8_t buffer[2] = { 0, 0 };
	readRegisterN(addr, buffer, 2);

	uint16_t ret = buffer[0];
	ret <<= 8;
	ret |= buffer[1];

	return ret;
}

uint32_t MAX31856::readRegister24(uint8_t addr) {
	uint8_t buffer[3] = { 0, 0, 0 };
	readRegisterN(addr, buffer, 3);

	uint32_t ret = buffer[0];
	ret <<= 8;
	ret |= buffer[1];
	ret <<= 8;
	ret |= buffer[2];

	return ret;
}

void MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {
	addr &= 0x7F; // make sure top bit is not set

	spi->beginTransaction(max31856_spisettings);

	digitalWrite(spi_cs, LOW);

	spixfer(addr);

	//Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
	while (n--) {
		buffer[0] = spixfer(0xFF);
		//Serial.print(" 0x"); Serial.print(buffer[0], HEX);
		buffer++;
	}
	//Serial.println();

	spi->endTransaction();

	digitalWrite(spi_cs, HIGH);
}


void MAX31856::writeRegister8(uint8_t addr, uint8_t data) {
	addr |= 0x80; // make sure top bit is set

	spi->beginTransaction(max31856_spisettings);

	digitalWrite(spi_cs, LOW);

	spixfer(addr);
	spixfer(data);

	//Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);

	spi->endTransaction();

	digitalWrite(spi_cs, HIGH);
}



uint8_t MAX31856::spixfer(uint8_t x) {
	return spi->transfer(x);
}
