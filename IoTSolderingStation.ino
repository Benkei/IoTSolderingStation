
#include "MAX31856.h"
#include <Wire.h>
#include <SSD1306.h>
#include "RotaryEncoder.h"
//#include <U8g2lib.h>
#include "esp_attr.h"
//#include <Adafruit_MAX31856.h>
#include <SPI.h>

// https://github.com/ThingPulse/esp8266-oled-ssd1306
// https://github.com/adafruit/Adafruit_MAX31856

#define OLED_CLOCK 4
#define OLED_DATA 5
#define ROTARY_A 25
#define ROTARY_B 26

SPIClass hspi(HSPI);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, OLED_CLOCK, OLED_DATA);
SSD1306 display2(0x3c, 5, 4);
RotaryEncoder rotary(ROTARY_A, ROTARY_B, 5, 10, 1000 / 10, 1000 / 5);
// Use software SPI: CS
MAX31856 tempReader(2, &hspi);

unsigned int count = 0;

void setup()
{
	Serial.begin(115200);

	Serial.println("start");

	//display.setI2CAddress(0x3c);
	//display.begin();
	//display.setFont(u8g2_font_9x15_tr);

	display2.init();
	display2.flipScreenVertically();
	display2.setFont(ArialMT_Plain_10);

	pinMode(ROTARY_A, INPUT_PULLUP);
	pinMode(ROTARY_B, INPUT_PULLUP);
	digitalWrite(ROTARY_A, HIGH);
	digitalWrite(ROTARY_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(ROTARY_A), handleInterruptRotary, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_B), handleInterruptRotary, CHANGE);

	Serial.println("GUI");

	xTaskCreate(
		taskLoop,          /* Task function. */
		"taskLoop",        /* String with name of task. */
		10000,            /* Stack size in words. */
		NULL,             /* Parameter passed as input of the task */
		1,                /* Priority of the task. */
		NULL);            /* Task handle. */

	Serial.println("start MAX31856");
	tempReader.Begin();
	//tempReader.setThermocoupleType(MAX31856_TCTYPE_K);

	Serial.print("Thermocouple type: ");
	switch (tempReader.getThermocoupleType()) {
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

}

void loop() {
	Serial.print("Cold Junction Temp: ");
	Serial.println(tempReader.readCJTemperature());

	Serial.print("Thermocouple Temp: ");
	Serial.println(tempReader.readThermocoupleTemperature());
	// Check and print any faults
	uint8_t fault = tempReader.readFault();
	if (fault) {
		if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
		if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
		if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
		if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
		if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
		if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
		if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
		if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
	}
	delay(1000);
}

void taskLoop(void* params) {
	Serial.println("Start render task");

	while (1)
	{
		count++;

		//display.clearBuffer();
		display2.clear();

		String text = "Count: " + String(count);

		displayDisplay(0, 10, &text);

		text = "Rot: " + String(rotary.getPosition());
		displayDisplay(0, 22, &text);
		text = "RotV: " + String(rotary.getPositionV());
		displayDisplay(0, 34, &text);

		//display.sendBuffer();
		display2.display();

		vTaskDelay(33 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}

void displayDisplay(uint16_t x, uint16_t y, const String* text) {
	char muh[16];
	text->toCharArray(muh, 16);
	//display.drawStr(x, y, muh);
	display2.drawString(x, y, muh);
}

void IRAM_ATTR handleInterruptRotary() {
	rotary.tick(millis());
}