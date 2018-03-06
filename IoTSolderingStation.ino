
#include "MAX31856.h"
#include <Wire.h>
#include <SSD1306.h>
#include "RotaryEncoder.h"
#include "esp_attr.h"
#include <SPI.h>

// https://github.com/ThingPulse/esp8266-oled-ssd1306
// https://github.com/adafruit/Adafruit_MAX31856

#define OLED_CLOCK 4
#define OLED_DATA 5
#define ROTARY_A 25
#define ROTARY_B 26
#define HEAT_PIN 16

SPIClass hspi(HSPI);
SSD1306 display(0x3c, 5, 4);
RotaryEncoder rotary(ROTARY_A, ROTARY_B);
// Use software SPI: CS
MAX31856 tempReader(2, hspi);

volatile float tcTemp = 0;
volatile float cjTemp = 0;

volatile uint16_t tipTempTarget = 0;


void setup()
{
	Serial.begin(115200);

	delay(1000);

	Serial.println("RUN");

	rotary.begin();
	rotary.setup(5, 10, 1000 / 10, 1000 / 5, 0, 400);
	pinMode(ROTARY_A, INPUT_PULLUP);
	digitalWrite(ROTARY_A, HIGH);
	pinMode(ROTARY_B, INPUT_PULLUP);
	digitalWrite(ROTARY_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(ROTARY_A), handleInterruptRotary, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_B), handleInterruptRotary, CHANGE);

	pinMode(HEAT_PIN, OUTPUT);
	digitalWrite(HEAT_PIN, LOW);


	pinMode(15, INPUT);

	//start and configure hardware SPI
	hspi.begin();

	display.init();
	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_10);

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
	case VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
	default: Serial.println("Unknown"); break;
	}

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
		10000,            /* Stack size in words. */
		NULL,             /* Parameter passed as input of the task */
		1,                /* Priority of the task. */
		NULL);            /* Task handle. */
}

void loop()
{}

volatile uint32_t tempTime = 0;
void taskTipControl(void* params)
{
	Serial.println("Start tip control task");
	Serial.println("Tip temp time " + String(tempReader.GetConversionTime()) + "ms");

	while (1)
	{
		tipTempTarget = rotary.getPosition();
		if (digitalRead(HEAT_PIN))
		{
			digitalWrite(HEAT_PIN, LOW);
			Serial.println("stop temp");
			vTaskDelay((1 * 20) / portTICK_RATE_MS);
		}

		Serial.println("begin temp " + String(!digitalRead(15)));

		if (tipTempTarget > tcTemp)
		{
			// stop cold junction messurment
			tempReader.SetConfiguration0Flags(
				(CR0)(CR0::OC_DETECT_DISABLED
					| CR0::COLD_JUNC_DISABLE
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
		tempReader.SetConversionMode(ConversionMode::OneShot);

		uint32_t t = millis();
		while (digitalRead(15))
		{
			//Serial.println("wait temp " + String(!digitalRead(15)));
			vTaskDelay(1);
		}
		t = millis() - t;
		tempTime = t;
		Serial.println("end temp " + String(t) + " " + String(tempReader.GetConversionMode()));

		//vTaskDelay(tempReader.GetConversionTime() / portTICK_RATE_MS);

		tempReader.SetConversionMode(ConversionMode::Off);

		tcTemp = tempReader.readTCTemperature();
		cjTemp = tempReader.readCJTemperature();

		// Check and print any faults
		uint8_t fault = tempReader.readFault();

		if (fault)
		{
			if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
			if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
			if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
			if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
			if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
			if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
			if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
			if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");

			//if (fault & MAX31856_FAULT_OPEN)
			continue;
		}

		if (tipTempTarget > tcTemp)
		{
			// tip heating require
			Serial.println("tip heating: " + String(tcTemp) + "/" + String(tipTempTarget));

			digitalWrite(HEAT_PIN, HIGH);

			int delta = sqrt(tipTempTarget - tcTemp);

			vTaskDelay(40 + (delta * 200) / portTICK_RATE_MS);
		}
		else
		{
			// no tip heating require
			Serial.println("no tip heating");

			digitalWrite(HEAT_PIN, LOW);

			vTaskDelay((2 * 20) / portTICK_RATE_MS);
		}
	}

	vTaskDelete(NULL);
}

void taskDisplayRender(void* params)
{
	Serial.println("Start render task");

	uint32_t time;
	uint16_t dtime = 0;

	while (1)
	{
		time = micros();

		display.clear();

		display.drawString(0, 10, "dtime: " + String(dtime));
		display.drawString(0, 21, String(tempTime) + "ms");
		display.drawString(0, 34, "Temp: " + String(tcTemp) + " " + String(cjTemp));
		display.drawString(0, 46, "Target Temp: " + String(rotary.getPosition()));

		display.display();


		dtime = micros() - time;

		vTaskDelay(33 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}

void IRAM_ATTR handleInterruptRotary()
{
	rotary.tick(millis());
}