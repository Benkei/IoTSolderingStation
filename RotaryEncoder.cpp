// -----
// RotaryEncoder.cpp - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 17.06.2015 minor updates.
// -----

#include "Arduino.h"
#include "RotaryEncoder.h"



// The array holds the values –1 for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
	0, -1,  1,  0,
	1,  0,  0, -1,
	-1,  0,  0,  1,
	0,  1, -1,  0 };


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

RotaryEncoder::RotaryEncoder(int pin1, int pin2,
	unsigned long stepMid, unsigned long stepHi,
	unsigned long maxSpeedMid, unsigned long maxSpeedHi) {
	// Remember Hardware Setup
	_pin1 = pin1;
	_pin2 = pin2;

	// when not started in motion, the current state of the encoder should be 3
	_oldState = 3;

	// start with position 0;
	_position = 0;
	_positionExt = 0;
	_positionVExt = 0;

	_lastTime = 0;

	_stepMid = stepMid;
	_stepHi = stepHi;

	_maxSpeedMid = maxSpeedMid;
	_maxSpeedHi = maxSpeedHi;
}

void RotaryEncoder::begin() {
	// Setup the input pins
	pinMode(_pin1, INPUT);
	digitalWrite(_pin1, HIGH);   // turn on pullup resistor

	pinMode(_pin2, INPUT);
	digitalWrite(_pin2, HIGH);   // turn on pullup resistor
}

long RotaryEncoder::getPosition() {
	return _positionExt;
}

long RotaryEncoder::getPositionV() {
	return _positionVExt;
}

void RotaryEncoder::setPosition(long newPosition) {
	// only adjust the external part of the position.
	_position = ((newPosition << 2) | (_position & 0x03L));
	_positionExt = newPosition;
	_positionVExt = newPosition;
}

void RotaryEncoder::tick(unsigned long time)
{
	int sig1 = digitalRead(_pin1);
	int sig2 = digitalRead(_pin2);
	int8_t thisState = sig1 | (sig2 << 1);
	unsigned long speed;
	long pos;

	if (_oldState != thisState) {
		_position += KNOBDIR[thisState | (_oldState << 2)];;

		if (thisState == LATCHSTATE) {
			pos = _position >> 2;

			if (_positionExt != pos) {

				speed = time - _lastTime;
				_lastTime = time;

				unsigned long step = 1;
				if (speed < _maxSpeedMid) {
					if (speed < _maxSpeedHi) {
						step = _stepHi;
					}
					else {
						step = _stepMid;
					}
				}
				_positionVExt += _positionExt > pos ? -step : step;
				_positionExt = pos;
			}
		}

		_oldState = thisState;
	}
}

