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
	0,  1, -1,  0
};


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

RotaryEncoder::RotaryEncoder(int pin1, int pin2)
{
	// Remember Hardware Setup
	_pin1 = pin1;
	_pin2 = pin2;

	// when not started in motion, the current state of the encoder should be 3
	_oldState = 3;

	// start with position 0;
	_position = 0;
	_positionExt = 0;

	_lastTime = 0;

	_stepMid = 0;
	_stepHi = 0;
	_velocityMid = 0;
	_velocityHi = 0;
	_minPosition = -100;
	_maxPosition = 100;
}

void RotaryEncoder::begin()
{
	// Setup the input pins
	pinMode(_pin1, INPUT);
	digitalWrite(_pin1, HIGH);   // turn on pullup resistor

	pinMode(_pin2, INPUT);
	digitalWrite(_pin2, HIGH);   // turn on pullup resistor
}

void RotaryEncoder::setup(
	unsigned long stepMid, unsigned long stepHi,
	unsigned long velocityMid, unsigned long velocityHi,
	long minPosition, long maxPosition)
{
	_stepMid = stepMid;
	_stepHi = stepHi;
	_velocityMid = velocityMid;
	_velocityHi = velocityHi;
	_minPosition = minPosition;
	_maxPosition = maxPosition;
}

long RotaryEncoder::getPosition()
{
	return _positionExt;
}

void RotaryEncoder::setPosition(long newPosition)
{
	newPosition = constrain(newPosition, _minPosition, _maxPosition);
	// only adjust the external part of the position.
	_position = ((newPosition << 2) | (_position & 0x03L));
	_positionExt = newPosition;
}

void RotaryEncoder::tick(unsigned long time)
{
	int sig1 = digitalRead(_pin1);
	int sig2 = digitalRead(_pin2);
	int8_t thisState = sig1 | (sig2 << 1);
	unsigned long speed;
	long pos, delta;

	if (_oldState != thisState)
	{
		_position += KNOBDIR[thisState | (_oldState << 2)];;

		if (thisState == LATCHSTATE)
		{
			pos = _position >> 2;
			if (_positionExt != pos)
			{
				speed = time - _lastTime;
				_lastTime = time;

				if (speed < _velocityMid)
				{
					unsigned long step = 0;
					if (speed < _velocityHi)
					{
						step = _stepHi - 1;
					}
					else
					{
						step = _stepMid - 1;
					}
					pos += _positionExt > pos ? -step : step;
				}
			}
			pos = constrain(pos, _minPosition, _maxPosition);
			_positionExt = pos;
			_position = (pos << 2) | (_position & 0b11);
		}

		_oldState = thisState;
	}
}

