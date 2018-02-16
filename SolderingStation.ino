
#include <Wire.h>
#include <esp8266-oled-ssd1306\SSD1306.h>
#include "RotaryEncoder.h"
//#include <U8g2lib.h>
#include "esp_attr.h"

#define OLED_CLOCK 4
#define OLED_DATA 5
#define ROTARY_A 25
#define ROTARY_B 26

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, OLED_CLOCK, OLED_DATA);
SSD1306 display2(0x3c, 5, 4);
RotaryEncoder rotary(ROTARY_A, ROTARY_B, 5, 10, 1000 / 10, 1000 / 5);

unsigned int count = 0;

void setup()
{
	Serial.begin(115200);

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
	

	xTaskCreate(
		taskLoop,          /* Task function. */
		"taskLoop",        /* String with name of task. */
		10000,            /* Stack size in words. */
		NULL,             /* Parameter passed as input of the task */
		1,                /* Priority of the task. */
		NULL);            /* Task handle. */

	Serial.println("Muh");
}

void loop() {}

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