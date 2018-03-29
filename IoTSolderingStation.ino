
#include <PID_v1.h>
#include <U8x8lib.h>
#include <U8g2lib.h>
#include "MAX31856.h"
#include "RotaryEncoder.h"
#include "esp_attr.h"
#include <SPI.h>

// https://github.com/ThingPulse/esp8266-oled-ssd1306
// https://github.com/adafruit/Adafruit_MAX31856

#define ROTARY_A GPIO_NUM_27
#define ROTARY_B GPIO_NUM_26

#define HEAT_PIN GPIO_NUM_25

#define OLED_CS GPIO_NUM_2 
#define OLED_DC GPIO_NUM_0 
#define OLED_RESET GPIO_NUM_15 

#define MAX31856_CS GPIO_NUM_4
#define MAX31856_DRDY GPIO_NUM_16

#define VIB_SWITCH GPIO_NUM_33


SPIClass hspi(HSPI);
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI display(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);
//SSD1306 display(0x3c, 5, 4);
RotaryEncoder rotary(ROTARY_A, ROTARY_B);
// Use software SPI: CS
MAX31856 tempReader(MAX31856_CS, hspi);


double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 200;
unsigned long windowStartTime;

volatile bool heater = false;

volatile float tcTemp = 0;
volatile float tcVoltage = 0;
volatile float tcVoltageTarget = 0;
volatile float cjTemp = 0;

volatile uint16_t tipTempTarget = 0;

volatile uint32_t tipLastVibration = 0;

volatile uint16_t tipTempSleep = 100;
volatile uint16_t tipSleepTime = 1; // minutes



const int16_t lutTemperatur[] = {
	21	,
	26	,
	30	,
	33	,
	37	,
	41	,
	44	,
	48	,
	52	,
	56	,
	60	,
	98	,
	135,
	170,
	205,
	233,
	267,
	303,
	340,
	380,
	418,
	453,
	480,
	500,
};
const int16_t lutVolt[] = {
	0	,
	1	,
	2	,
	3	,
	4	,
	5	,
	6	,
	7	,
	8	,
	9	,
	10	,
	20	,
	30	,
	40	,
	50	,
	60	,
	70	,
	80	,
	90	,
	100,
	110,
	121,
	130,
	140,
};


void setup()
{
	Serial.begin(115200);

	delay(1000);

	Serial.println("RUN");

	rotary.begin();
	rotary.setup(5, 10, 1000 / 10, 1000 / 5, 0, 400);

	pinMode(HEAT_PIN, OUTPUT);
	digitalWrite(HEAT_PIN, LOW);

	pinMode(MAX31856_DRDY, INPUT);

	pinMode(VIB_SWITCH, INPUT_PULLUP);

	//start and configure hardware SPI
	hspi.begin();

	display.begin();
	display.setFont(u8g2_font_6x10_mf);

	tempReader.begin();
	Serial.print("Thermocouple type: ");
	switch (tempReader.getThermocoupleType())
	{
	case TCTYPE_B: Serial.println("B Type"); break;
	case TCTYPE_E: Serial.println("E Type"); break;
	case TCTYPE_J: Serial.println("J Type"); break;
	case TCTYPE_K: Serial.println("K Type"); break;
	case TCTYPE_N: Serial.println("N Type"); break;
	case TCTYPE_R: Serial.println("R Type"); break;
	case TCTYPE_S: Serial.println("S Type"); break;
	case TCTYPE_T: Serial.println("T Type"); break;
	case VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
	case VMODE_G32: Serial.println("Voltage x32 Gain mode"); break;
	default: Serial.println("Unknown"); break;
	}



	windowStartTime = millis();

	//initialize the variables we're linked to
	Setpoint = 0;

	//tell the PID to range between 0 and the full window size
	myPID.SetOutputLimits(0, WindowSize);
	myPID.SetSampleTime(200);

	//turn the PID on
	myPID.SetMode(AUTOMATIC);


	xTaskCreate(
		taskRotary,
		"taskRotary",
		1000,
		NULL,
		3,
		NULL);

	xTaskCreate(
		taskDisplayRender,          /* Task function. */
		"taskLoop",        /* String with name of task. */
		10000,            /* Stack size in words. */
		NULL,             /* Parameter passed as input of the task */
		2,                /* Priority of the task. */
		NULL);            /* Task handle. */

	xTaskCreate(
		taskTipControl,          /* Task function. */
		"taskTipControl",        /* String with name of task. */
		1000,            /* Stack size in words. */
		NULL,             /* Parameter passed as input of the task */
		1,                /* Priority of the task. */
		NULL);            /* Task handle. */

	xTaskCreate(
		taskTipVibration,
		"taskTipVibration",
		1024,
		NULL,
		4,
		NULL);
}

void loop()
{
	//Setpoint = tipTempTarget;
	//Input = tcTemp;

	//myPID.Compute();


	//// turn the output pin on/off based on pid output	
	//if (millis() - windowStartTime > WindowSize)
	//{
	//	//time to shift the Relay Window
	//	windowStartTime += WindowSize;
	//}
	//if (Output < millis() - windowStartTime)
	//{
	//	//digitalWrite(HEAT_PIN, HIGH);
	//	heater = false;
	//}
	//else
	//{
	//	//digitalWrite(HEAT_PIN, LOW);
	//	heater = true;
	//}
}

void taskTipVibration(void* params)
{
	tipLastVibration = millis();
	bool tipMove = false;
	while (1)
	{
		bool vib = digitalRead(VIB_SWITCH) == HIGH;
		if (vib != tipMove)
		{
			tipMove = !tipMove;
			tipLastVibration = millis();

			Serial.println("tip moved " + String(tipLastVibration));
		}
		vTaskDelay(10 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

void taskRotary(void* params)
{
	while (1)
	{
		rotary.tick(millis());
		vTaskDelay(1 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

volatile uint32_t tempTime = 0;
void taskTipControl(void* params)
{
	Serial.println("Start tip control task");
	Serial.println("Tip temp time " + String(tempReader.GetConversionTime()) + "ms");

	unsigned long tf = 0;
	unsigned long tflast = 0;
	bool heatTipMode = true;
	bool readTemperature = false;
	uint32_t readTempBegin = UINT32_MAX;
	uint32_t heatTime = UINT32_MAX;

	while (1)
	{
		tipTempTarget = rotary.getPosition();

		bool hasTemp = digitalRead(MAX31856_DRDY) == LOW;

		if (tipTempTarget == 0)
		{
			heatTime = UINT32_MAX;

			// stop heater
			if (digitalRead(HEAT_PIN))
			{
				digitalWrite(HEAT_PIN, LOW);
				Serial.println("stop temp");
				vTaskDelay(5 / portTICK_RATE_MS);
			}

			if (hasTemp || readTempBegin == UINT32_MAX || millis() - readTempBegin > 200)
			{
				auto type = tempReader.getThermocoupleType();

				if (heatTipMode || readTempBegin == UINT32_MAX || type != MAX31856_ThermocoupleType::VMODE_G8)
				{
					heatTipMode = false;
					SetupMAXAutoMode();
					readTempBegin = millis();

					Serial.println("normal tip mode");
					continue;
				}

				tempTime = millis() - readTempBegin;
				readTempBegin = millis();

				cjTemp = tempReader.readCJTemperature();

				auto fault = CheckMAXFault() & (MAX31856_FAULT_OVUV | MAX31856_FAULT_OPEN);
				if (fault == 0) {
					tcVoltage = tempReader.readTCTemperature();
					tcTemp = VoltageToTemperature(tcVoltage);
				}
			}
		}
		else
		{
			if (heatTime == UINT32_MAX)
			{
				if (!heatTipMode || readTempBegin == UINT32_MAX)
				{
					heatTipMode = true;
					// stop heater
					if (digitalRead(HEAT_PIN))
					{
						digitalWrite(HEAT_PIN, LOW);
						Serial.println("stop temp");
						vTaskDelay(5 / portTICK_RATE_MS);
					}

					SetupMAXOneShotMode();
					readTempBegin = millis();
				}
				else if (hasTemp || millis() - readTempBegin > 200)
				{
					tempTime = millis() - readTempBegin;
					cjTemp = tempReader.readCJTemperature();
					// error start next temp check
					readTempBegin = UINT32_MAX;

					auto fault = CheckMAXFault() & (MAX31856_FAULT_OVUV | MAX31856_FAULT_OPEN);
					if (fault == 0) {
						tcVoltage = tempReader.readTCTemperature();
						tcTemp = VoltageToTemperature(tcVoltage);

						uint16_t temp = tipTempSleep > tipTempTarget && (millis() - tipLastVibration) > (tipSleepTime*(60 * 1000)) ? tipTempSleep : tipTempTarget;
						tcVoltageTarget = TemperatureToVoltage(temp);

						if (tcVoltageTarget > tcVoltage)
						{
							float delta = sqrt(tcVoltageTarget - tcVoltage);
							long time = (delta * 200);

							heatTime = millis() + time;

							// tip heating require
							Serial.println("tip heating: " + String(tcTemp) + "/" + String(temp) + " " + String(time) + "ms");
						}
					}
				}
			}
			if (heatTime != UINT32_MAX)
			{
				if (millis() < heatTime)
				{
					digitalWrite(HEAT_PIN, HIGH);
				}
				else
				{
					// heat done
					digitalWrite(HEAT_PIN, LOW);

					heatTime = UINT32_MAX;
				}
			}
		}


		/*

		if (digitalRead(HEAT_PIN))
		{
			digitalWrite(HEAT_PIN, LOW);
			Serial.println("stop temp");
			vTaskDelay(tflast / portTICK_RATE_MS);
		}

		//Serial.println("begin temp " + String(hasTemp));

		// Check and print any faults
		uint8_t fault = tempReader.readFault();
		bool tcFault = fault & (MAX31856_FAULT_OVUV | MAX31856_FAULT_OPEN);

		if (fault)
		{
			if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
			// ignored in voltage mode.
			//if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
			if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
			if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
			if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
			if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
			if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
			if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");

			// ignored in voltage mode.
			fault &= ~MAX31856_FAULT_TCRANGE;

			if (tf == 0 && fault & MAX31856_FAULT_OVUV)
				tf = millis();
		}

		if (tf != 0)
		{
			tf = millis() - tf;
			Serial.println("fault time " + String(tf) + "ms");
			tflast = max(tflast, tf) * 2;
			tf = 0;
		}


		if (!hasTemp)
		{
			if (tipTempTarget > tcTemp)
			{
				// stop cold junction messurment
				tempReader.SetConfiguration0Flags(
					(CR0)(CR0::OC_DETECT_ENABLED_R_LESS_5k
						| CR0::COLD_JUNC_ENABLE
						| CR0::FAULT_MODE_COMPARATOR
						| CR0::FAULTCLR_DEFAULT_VAL
						| CR0::FILTER_OUT_50Hz)
				);
			}
			else
			{
				tempReader.SetConfiguration0Flags(
					(CR0)(CR0::OC_DETECT_ENABLED_R_LESS_5k
						| CR0::COLD_JUNC_ENABLE
						| CR0::FAULT_MODE_COMPARATOR
						| CR0::FAULTCLR_DEFAULT_VAL
						| CR0::FILTER_OUT_50Hz)
				);
			}

			//tempReader.setThermocoupleType(MAX31856_ThermocoupleType::VMODE_G8);
			tempReader.SetConversionMode(ConversionMode::OneShot);

			unsigned long t = millis();
			while (!hasTemp)
			{
				//Serial.println("wait temp");
				vTaskDelay(1);
				hasTemp = digitalRead(MAX31856_DRDY) == LOW;

				if (millis() - t > 1000)
				{
					Serial.println("temp timeout");
					break;
				}
			}
			t = millis() - t;
			tempTime = t;

			//vTaskDelay(tempReader.GetConversionTime() / portTICK_RATE_MS);

			tempReader.SetConversionMode(ConversionMode::Off);

			//Serial.println("end temp " + String(t) + " " + String(tempReader.GetConversionMode()));
		}

		//tcTemp = tempReader.readTCTemperature();
		cjTemp = tempReader.readCJTemperature();

		if (!tcFault)
		{
			float voltage = tempReader.readTCTemperature();
			tcTemp = VoltageToTemperature(voltage);
			tcVoltage = voltage;
			tcVoltageTarget = TemperatureToVoltage(tipTempTarget);
		}


		if (tcVoltageTarget > tcVoltage && fault == 0 && tipTempTarget > 0)
			//if (tipTempTarget > tcTemp && fault == 0 && tipTempTarget > 0)
		{
			vTaskDelay(1 / portTICK_RATE_MS);

			digitalWrite(HEAT_PIN, HIGH);

			//long delta = sqrt(tipTempTarget - tcTemp);
			float delta = sqrt(tcVoltageTarget - tcVoltage);
			long time = 40 + (delta * 200);

			// tip heating require
			Serial.println("tip heating: " + String(tcTemp) + "/" + String(tipTempTarget) + " " + String(time) + "ms");

			vTaskDelay(time / portTICK_RATE_MS);
		}

		*/
	}

	vTaskDelete(NULL);
}

uint8_t CheckMAXFault() {
	uint8_t fault = tempReader.readFault();
	//bool tcFault = fault & (MAX31856_FAULT_OVUV | MAX31856_FAULT_OPEN);

	if (fault)
	{
		if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
		// ignored in voltage mode.
		//if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
		if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
		if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
		if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
		if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
		if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
		if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");

		// ignored in voltage mode.
		fault &= ~MAX31856_FAULT_TCRANGE;
	}
	return fault;
}

void SetupMAXAutoMode() {
	tempReader.SetConfiguration1Flags((CR1)(CR1::AVG_TC_SAMPLES_1 | CR1::TC_TYPE_VOLT_MODE_GAIN_8));

	tempReader.SetConfiguration0Flags(
		(CR0)(CR0::CONV_MODE_NORMALLY_ON
			| CR0::OC_DETECT_ENABLED_R_LESS_5k
			| CR0::COLD_JUNC_ENABLE
			| CR0::FAULT_MODE_COMPARATOR
			| CR0::FAULTCLR_DEFAULT_VAL
			| CR0::FILTER_OUT_50Hz)
	);
}

void SetupMAXOneShotMode() {
	tempReader.SetConfiguration1Flags((CR1)(CR1::AVG_TC_SAMPLES_1 | CR1::TC_TYPE_VOLT_MODE_GAIN_8));

	tempReader.SetConfiguration0Flags(
		(CR0)(CR0::ONE_SHOT_MODE_ONE_CONVERSION
			| CR0::OC_DETECT_ENABLED_R_LESS_5k
			| CR0::COLD_JUNC_ENABLE
			| CR0::FAULT_MODE_COMPARATOR
			| CR0::FAULTCLR_DEFAULT_VAL
			| CR0::FILTER_OUT_50Hz)
	);
}

float VoltageToTemperature(float voltage)
{
	int8_t idx = 0;
	for (size_t i = 1; i < sizeof(lutVolt) / sizeof(lutVolt[0]); i++)
	{
		if (lutVolt[i] >= voltage)
		{
			idx = i - 1;
			break;
		}
	}

	auto range = lutVolt[idx + 1] - lutVolt[idx];
	float delta = (voltage - lutVolt[idx]) / range;

	auto rangeT = lutTemperatur[idx + 1] - lutTemperatur[idx];
	float temp = lutTemperatur[idx] + rangeT * delta;

	return temp;
}

float TemperatureToVoltage(int16_t temperatur)
{
	int8_t idx = 0;
	for (size_t i = 1; i < sizeof(lutTemperatur) / sizeof(lutTemperatur[0]); i++)
	{
		if (lutTemperatur[i] >= temperatur)
		{
			idx = i - 1;
			break;
		}
	}

	float range = lutTemperatur[idx + 1] - lutTemperatur[idx];
	float delta = (temperatur - lutTemperatur[idx]) / range;

	auto rangeT = lutVolt[idx + 1] - lutVolt[idx];
	float voltage = lutVolt[idx] + rangeT * delta;

	return voltage;
}



void taskDisplayRender(void* params)
{
	Serial.println("Start render task");

	uint32_t time;
	uint16_t dtime = 0;

	while (1)
	{
		time = millis();

		display.clearBuffer();

		display.drawStr(0, 10, ("dtime: " + String(dtime)).c_str());
		display.drawStr(0, 21, (String(tempTime) + "ms").c_str());
		display.drawStr(0, 34, ("Temp: " + String(tcTemp) + " " + String(cjTemp)).c_str());
		display.drawStr(0, 46, ("Target Temp: " + String(rotary.getPosition())).c_str());
		display.drawStr(0, 58, ("Tip V: " + String(tcVoltage) + "/" + String(tcVoltageTarget)).c_str());
		//display.drawStr(0, 58, (String(heater) + " I: " + String(Input) + " O: " + String(Output)).c_str());

		display.sendBuffer();


		dtime = millis() - time;

		vTaskDelay(33 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}

