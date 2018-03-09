
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


SPIClass hspi(HSPI);
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI display(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);
//SSD1306 display(0x3c, 5, 4);
RotaryEncoder rotary(ROTARY_A, ROTARY_B);
// Use software SPI: CS
MAX31856 tempReader(MAX31856_CS, hspi);

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

	pinMode(HEAT_PIN, OUTPUT);
	digitalWrite(HEAT_PIN, LOW);


	pinMode(MAX31856_DRDY, INPUT);

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
	case VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
	default: Serial.println("Unknown"); break;
	}

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
}

void loop()
{}

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

	while (1)
	{
		tipTempTarget = rotary.getPosition();
		if (digitalRead(HEAT_PIN))
		{
			digitalWrite(HEAT_PIN, LOW);
			Serial.println("stop temp");
			vTaskDelay(tflast / portTICK_RATE_MS);
		}

		boolean hasTemp = digitalRead(MAX31856_DRDY) == LOW;

		Serial.println("begin temp " + String(hasTemp));

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

			if (tf == 0 && fault & MAX31856_FAULT_OVUV)
				tf = millis();
			//if (fault & MAX31856_FAULT_OPEN)
			continue;
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
					(CR0)(CR0::OC_DETECT_ENABLED_TC_LESS_2ms
						| CR0::COLD_JUNC_ENABLE
						| CR0::FAULT_MODE_COMPARATOR
						| CR0::FAULTCLR_DEFAULT_VAL
						| CR0::FILTER_OUT_50Hz)
				);
			}
			tempReader.SetConversionMode(ConversionMode::OneShot);

			unsigned long t = millis();
			while (!hasTemp)
			{
				//Serial.println("wait temp");
				vTaskDelay(1);
				hasTemp = digitalRead(MAX31856_DRDY) == LOW;

				if (millis() - t > 1000)
				{
					break;
				}
			}
			t = millis() - t;
			tempTime = t;

			//vTaskDelay(tempReader.GetConversionTime() / portTICK_RATE_MS);

			tempReader.SetConversionMode(ConversionMode::Off);

			Serial.println("end temp " + String(t) + " " + String(tempReader.GetConversionMode()));
		}

		tcTemp = tempReader.readTCTemperature();
		cjTemp = tempReader.readCJTemperature();

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
		time = millis();

		display.clearBuffer();

		display.drawStr(0, 10, ("dtime: " + String(dtime)).c_str());
		display.drawStr(0, 21, (String(tempTime) + "ms").c_str());
		display.drawStr(0, 34, ("Temp: " + String(tcTemp) + " " + String(cjTemp)).c_str());
		display.drawStr(0, 46, ("Target Temp: " + String(rotary.getPosition())).c_str());

		display.sendBuffer();


		dtime = millis() - time;

		vTaskDelay(33 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}

