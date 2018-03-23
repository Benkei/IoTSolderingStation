// 
// 
// 

#include "MAX31856.h"

static SPISettings max31856_spisettings = SPISettings(500000, MSBFIRST, SPI_MODE1);


MAX31856::MAX31856(int8_t spi_cs, SPIClass& spi) : spi_cs(spi_cs), spi(spi)
{}

void MAX31856::begin()
{
	pinMode(spi_cs, OUTPUT);
	digitalWrite(spi_cs, HIGH);

	// assert on any fault
	writeRegister8(MAX31856_MASK_REG, 0x0); // error mask to FAULT pin

	SetConfiguration0Flags(
		(CR0)(
			CR0::CONV_MODE_NORMALLY_OFF
			| CR0::OC_DETECT_ENABLED_R_LESS_5k
			| CR0::COLD_JUNC_ENABLE
			| CR0::FAULT_MODE_COMPARATOR
			| CR0::FAULTCLR_DEFAULT_VAL
			| CR0::FILTER_OUT_50Hz)
	);

	SetConfiguration1Flags(
		(CR1)(CR1::AVG_TC_SAMPLES_1 | CR1::TC_TYPE_VOLT_MODE_GAIN_8)
	);
}

void MAX31856::SetConfiguration0Flags(CR0 flags)
{
	config0 = flags;
	calculateDelayTime();
	writeRegister8(MAX31856_CR0_REG, flags);
}

void MAX31856::SetConfiguration1Flags(CR1 flags)
{
	config1 = flags;
	calculateDelayTime();
	writeRegister8(MAX31856_CR1_REG, flags);
}

int32_t MAX31856::GetConversionTime()
{
	return conversionTime;
}

void MAX31856::SetConversionMode(const ConversionMode mode)
{
	conversionCount = 0;
	config0 = (CR0)(config0 & ~ConversionMode::_MASK); // mask off
	config0 = (CR0)(config0 | mode);
	calculateDelayTime();
	SetConfiguration0Flags(config0);
}

ConversionMode MAX31856::GetConversionMode()
{
	return (ConversionMode)(config0 & ConversionMode::_MASK); // mask off first 3 bits
}


void MAX31856::setThermocoupleType(MAX31856_ThermocoupleType type)
{
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0xF0; // mask off bottom 4 bits
	t |= (uint8_t)type & 0x0F;
	writeRegister8(MAX31856_CR1_REG, t);
}

MAX31856_ThermocoupleType MAX31856::getThermocoupleType(void)
{
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0x0F;

	return (MAX31856_ThermocoupleType)t;
}

uint8_t MAX31856::readFault(void)
{
	return readRegister8(MAX31856_SR_REG);
}

void MAX31856::setColdJunctionFaultThreshholds(int8_t low, int8_t high)
{
	writeRegister8(MAX31856_CJLF_REG, low);
	writeRegister8(MAX31856_CJHF_REG, high);
}

void MAX31856::setTempFaultThreshholds(float flow, float fhigh)
{
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

void MAX31856::oneShotTemperature(void)
{

	writeRegister8(ADDRESS_CJTO_READ, 0x0);

	uint8_t t = readRegister8(MAX31856_CR0_REG);

	t &= ~MAX31856_CR0_AUTOCONVERT; // turn off autoconvert!
	t |= MAX31856_CR0_1SHOT;

	writeRegister8(MAX31856_CR0_REG, t);

	delay(250); // MEME FIX autocalculate based on oversampling
}

float MAX31856::readCJTemperature(void)
{
	//oneShotTemperature();
	/*
	int16_t temp16 = readRegister16(MAX31856_CJTH_REG);
	float tempfloat = temp16;
	tempfloat /= 256.0;

	return tempfloat;
	*/


	int32_t temp;
	uint8_t buf_read[2], buf_write = MAX31856_CJTH_REG;

	readRegisterN(buf_write, buf_read, 2);

	//Convert the registers contents into the correct value
	temp = ((int32_t)(buf_read[0] << 6));        //Shift the MSB into place
	temp |= ((int32_t)(buf_read[1] >> 2));        //Shift the LSB into place
	float val = ((float)(temp / 64.0));             //Divide the binary string by 2 to the 6th power

	return val;
}

float MAX31856::readTCTemperature(void)
{
	//oneShotTemperature();
	int32_t temp24 = readRegister24(MAX31856_LTCBH_REG);
	if (temp24 & 0x800000)
	{
		temp24 |= 0xFF000000;  // fix sign
	}

	temp24 >>= 5;  // bottom 5 bits are unused

	float tempfloat = temp24;
	tempfloat *= 0.0078125;

	return tempfloat;

	/*
	//initialize other info for the read functionality
	int32_t temp;
	uint8_t buf_read[3], buf_write[3] = { ADDRESS_LTCBH_READ, ADDRESS_LTCBM_READ, ADDRESS_LTCBL_READ };

	spi.beginTransaction(max31856_spisettings);


	for (int i = 0; i < 3; i++)
	{
		digitalWrite(spi_cs, LOW);
		spi.write(buf_write[i]);
		buf_read[i] = spi.transfer(0xFF);
		digitalWrite(spi_cs, HIGH);
	}

	spi.endTransaction();



	//Convert the registers contents into the correct value
	temp = ((buf_read[0] & 0xFF) << 11);       //Shift Byte 2 into place
	temp |= ((buf_read[1] & 0xFF) << 3);        //Shift Byte 1 into place
	temp |= ((buf_read[2] & 0xFF) >> 5);        //Shift Byte 0 into place
	for (byte i = 0; i < 24; i++)
	{
		Serial.print((temp & (1 << i)) ? "1" : "0");
	}
	Serial.println(" temp: " + String(temp));
	Serial.println("temp: " + String(temp / 4096.0f));
	float val = (temp / 128.0f);                  //Divide the binary string by 2 to the 7th power
	return val;
	*/
}

bool MAX31856::setColdJunctionOffset(float temperature)
{
	if (temperature > 7.9375 || temperature < -8.0)
	{
		//LOG("Input value to offest the cold junction point is non valid. enter in value in range -8 to +7.9375\r\n");
		return false;
	}
	int8_t temp_val = temperature * 16.0f; //normalize the value to get rid of decimal and shorten it to size of register
	writeRegister8(ADDRESS_CJTO_READ, temp_val); //write the byte to cold junction offset register
	return true;
}


/**********************************************/

uint8_t MAX31856::readRegister8(uint8_t addr)
{
	uint8_t ret = 0;
	readRegisterN(addr, &ret, 1);

	return ret;
}

uint16_t MAX31856::readRegister16(uint8_t addr)
{
	uint8_t buffer[2] = { 0, 0 };
	readRegisterN(addr, buffer, 2);

	uint16_t ret = buffer[0];
	ret <<= 8;
	ret |= buffer[1];

	return ret;
}

uint32_t MAX31856::readRegister24(uint8_t addr)
{
	uint8_t buffer[3] = { 0, 0, 0 };
	readRegisterN(addr, buffer, 3);

	uint32_t ret = buffer[0];
	ret <<= 8;
	ret |= buffer[1];
	ret <<= 8;
	ret |= buffer[2];

	return ret;
}

void MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n)
{
	addr &= 0x7F; // make sure top bit is not set

	spi.beginTransaction(max31856_spisettings);

	digitalWrite(spi_cs, LOW);

	//spi.transfer(addr);
	spi.write(addr);

	//Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
	while (n--)
	{
		buffer[0] = spi.transfer(0xFF);
		//Serial.print(" 0x"); Serial.print(buffer[0], HEX);
		buffer++;
	}
	//Serial.println();

	spi.endTransaction();

	digitalWrite(spi_cs, HIGH);
}


void MAX31856::writeRegister8(uint8_t addr, uint8_t data)
{
	addr |= 0x80; // make sure top bit is set

	spi.beginTransaction(max31856_spisettings);

	digitalWrite(spi_cs, LOW);

	//spi.transfer(addr);
	//spi.transfer(data);
	spi.write(addr);
	spi.write(data);

	//Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);

	spi.endTransaction();

	digitalWrite(spi_cs, HIGH);
}

void MAX31856::calculateDelayTime()
{
	uint32_t temp_int;
	bool filter50Hz = (config0 & CR0::FILTER_OUT_50Hz) == CR0::FILTER_OUT_50Hz;
	bool oneShotMode = (config0 & CR0::ONE_SHOT_MODE_ONE_CONVERSION) == CR0::ONE_SHOT_MODE_ONE_CONVERSION;
	bool cjDisable = (config0 & CR0::COLD_JUNC_DISABLE) == CR0::COLD_JUNC_DISABLE;
	uint8_t samples = 0;
	switch (config1 & ~0x0F)
	{
		//case AVG_TC_SAMPLES_1: break;
	case AVG_TC_SAMPLES_2: samples = 1; break;
	case AVG_TC_SAMPLES_4: samples = 3; break;
	case AVG_TC_SAMPLES_8: samples = 7; break;
	case AVG_TC_SAMPLES_16: samples = 15; break;
	}

	if (oneShotMode || conversionCount == 0)
	{
		// 1-Shot conversion or first conversion in auto - conversion mode
		// 50Hz
		// Typ:169ms Max:185ms
		// 60Hz
		// Typ:143ms Max:155ms

		if (filter50Hz)
			temp_int = 169 + samples * 40.00f;
		else
			temp_int = 143 + samples * 33.33f;
	}
	else
	{
		// Auto conversion mode, conversions 2 through n
		// 50Hz
		// Typ:98ms Max:110ms
		// 60Hz
		// Typ:82ms Max:90ms

		if (filter50Hz)
			temp_int = 98 + samples * 20.00f;
		else
			temp_int = 82 + samples * 16.67f;
	}

	if (cjDisable) //cold junction is disabled enabling 25 millisecond faster conversion times
		temp_int -= 25;

	conversionTime = temp_int;
}

