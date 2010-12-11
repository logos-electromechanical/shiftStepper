/*
  ShiftStepper.cpp - ShiftStepper library for Arduino
  v0.01a Alpha
  (c) Logos Electromechanical LLC 2010
  Licensed under CC-BY-SA 3.0 

  This library is intended to drive Logos Electromechanical
  shift register boards

  This library is free software; you can redistribute it and/or
  modify it under the terms of the Creative Commons By-Attribution 
  Share-Alike 3.0 Unported license.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include "WProgram.h"
#include "wiring.h"
#include "wiring_private.h"
#include "pins_arduino.h"
#include "HardwareSerial.h"

#include "shiftStepper.h"

// defines for the sixteen channel board
#define __shiftSixteenBytes__ 	2
#define __shiftSixteenFirst__ 	0
#define __shiftSixteenLast__	15

// defines for the six channel board
#define __shiftSixBytes__ 		1
#define __shiftSixFirst__ 		1
#define __shiftSixLast__		6

/////////////////
// shiftDevice //
/////////////////


uint8_t *shiftDevice::getChannels (void) {
	return this->channels;
}

uint8_t shiftDevice::getChanCount (void) {
	return this->chanCount;
}

////////////////////
// shiftStepMotor //
////////////////////

// Constructor //

/*!
  \param seqSteps The number of steps in the sequence of switch positions 
  required to turn the motor. Typically 4 (full steps) or 8 (half steps).
  \param *sequence The array of switch positions for each step in the sequence. 
  Bit 0 of each element of this array controls channels[0], bit 1 controls
  channels[1], etc. This array must have a number of elements equal to 
  seqSteps. This array must be allocated either statically or at global scope.
  \param *channels The array of channel numbers used by the motor. Must have 
  four elements. The order of the channels in this array determines how the
  members of sequence[] are interpreted. This array must be allocated either 
  statically or at global scope.
*/

shiftStepMotor::shiftStepMotor (const uint8_t seqSteps, 
	const uint8_t *sequence, 
	const uint8_t *channels) 
{
	uint8_t i;
	
	this->stepState 	= 0;
	this->stepSpeed		= 0;
	this->lastStep		= 0;
	this->stepsToGo		= 0;
	this->seqSteps 		= seqSteps;
	this->sequence		= sequence;
	this->chanCount		= __channelsPerMotor__;
	for (i=0; i < this->chanCount; i++) 
	{
		this->channels[i] = channels[i];
	}
}

// Private Members //

void shiftStepMotor::incrSeq (void)
{
	if ((this->stepState >= (this->seqSteps - 1)) ||
		(this->stepState < 0)) 			// if we're at the end of the step sequence,
	{										
		this->stepState = 0;					// go back to the beginning
	} else { this->stepState++; }				// if not, just increment
}

void shiftStepMotor::decrSeq	(void)
{
	if ((this->stepState >= this->seqSteps) ||
		(this->stepState <= 0)) 			// just like for incrSeq(), except in reverse
	{
		this->stepState = this->seqSteps - 1;
	} else { this->stepState--; }
}

// Public Members //

/*!
  This function does one of three things, depending on the current state of the motor:
	- If the motor is turning continuously (i.e. stepsToGo < 0), it does nothing
	- If the motor is stopped (stepsToGo = 0), it does one step in the commanded 
	direction
	- If the motor is in the middle of a commanded move, one step is added to the move
	in the commanded direction. If the move is positive and dir is positive, then one 
	step will be added to the current move. If the move is positive and the dir is 
	negative, one step will be subtracted for the current move. 
	
  \param dir This defines the direction in which to perform the step. If dir > 0,
  the step is positive; if dir <= 0, the step is negative.
  \return the error status. This feature has not been implemented, so it always 
  returns 0 (success)
 */

uint8_t shiftStepMotor::incrStep (int8_t dir) 
{
	
	if (this->stepsToGo) 							// if it's currently in the process of 
													// doing some steps, increment the count
	{
		if (this->stepsToGo < 0) { return 0; }		// if motor is currently set for continuous
													// rotation, do nothing
		if (this->stepSpeed > 0) {					// if the motor is currently turning in the
													// positive direction...
			if (dir > 0) { this->stepsToGo++; }	// if dir is positive, increment the count
			else { this->stepsToGo--; }		// if it's non-positive, decrement
		} else if (this->stepSpeed < 0) {			// if the motor is currently turning in the
													// negative direction...
			if (dir > 0) { this->stepsToGo--; }	// if dir is positive, decrement the count
			else { this->stepsToGo++; }		// if non-positive, increment the count
		}
	} else if (dir > 0) 							// if it's not currently doing some steps,
													// and want to do a positive one...
	{
		this->incrSeq();
		this->lastStep = 0;
	} else 
	{
		this->decrSeq();
		this->lastStep = 0;
	}
	return 0;
	
}

/*!
  \param steps The number of steps to move. If this number is negative, then this 
  function commands continuous rotation.
  \param speed The speed at which to make the move. 
  \return the error status. This feature has not been implemented, so it always 
  returns 0 (success)
  \sa shiftStepMotor::setSpeed() for details on how the speed parameter works. 
 */

uint8_t shiftStepMotor::doSteps (int16_t steps, int16_t speed)
{
	this->stepsToGo = steps;
	return this->setSpeed(speed);
}

/*!
  The most convenient way to represent the speed internally is as the number of doDevTick()
  calls between steps. However, this is unwieldy to use, so this function simplifies it.
  __maxMotorSpeed__ defines the maximum speed, and also the number of ticks that can 
  be skipped per step. In order to determine the number of steps to skip, this function
  executes the following steps:
	-# Limit the speed parameter to between -__maxMotorSpeed__ and __maxMotorSpeed__
	-# If speed > 0 (i.e. positive sense), set stepSpeed to __maxMotorSpeed__ - speed + 1
	-# If speed < 0 (i.e. negative sense), set stepSpeed to -__maxMotorSpeed__ - speed - 1
	-# If speed = 0 set stepSpeed to 0, i.e. no rotation
  
  \param speed The desired motor speed; sign indicates direction.
  \return the error status. This feature has not been implemented, so it always 
  returns 0 (success)
 */
uint8_t shiftStepMotor::setSpeed (int16_t speed) 
{
	if (speed > __maxMotorSpeed__) { speed = __maxMotorSpeed__; }
	else if (speed < -__maxMotorSpeed__) { speed = -__maxMotorSpeed__; }
	if (speed) 
	{
		if (speed > 0) { this->stepSpeed = (__maxMotorSpeed__ - speed + 1); }
		else { this->stepSpeed = (-__maxMotorSpeed__ - speed - 1); }
														// since speed is stored internally
														// as the number of ticks to wait before 
														// incrementing the step sequence, we need 
														// to rejigger the entered speed a bit
	} else { this->stepSpeed = 0; }
	return 0;
}

/*!
  \return Speed value is in the same form as the input to shiftStepMotor::setSpeed().
  It derives this from the internal representation by inverting the process described
  for that function.
  \sa shiftStepMotor::setSpeed()
 */

int16_t shiftStepMotor::getSpeed (void) 
{
	if (this->stepSpeed) 
	{
		if (this->stepSpeed > 0) 
		{ 
			return (__maxMotorSpeed__ - this->stepSpeed + 1);
		} else
		{ 
			return (-__maxMotorSpeed__ - this->stepSpeed - 1);
		}
	} else { return 0; }
}

/*!
  \return The number of steps yet to go. A negative value commands continuous rotation.
  \sa shiftStepMotor::doSteps();
 */
 
int16_t shiftStepMotor::getStepsToGo (void)
{
	return this->stepsToGo;
}

/*!
  This function manages the per-tick behavior of a stepper motor device.
  It performs the following tasks:
	- Checks whether or not it's time to make a step, based on the values of stepSpeed, lastStep, and stepsToGo
	- Writes the current switch state to the bits corresponding to the output 
	channels in the boardBytes[] array
*/
 
void shiftStepMotor::doDevTick (uint8_t bytesPerBoard,
	uint8_t *boardBytes)
{
	uint8_t i, j, k, m, n;

	this->lastStep++;
	
	if (this->stepsToGo != 0 ) 
	{
		if (this->stepSpeed > 0) 
		{
			if (this->lastStep >= this->stepSpeed) 
			{ 
				if (this->stepsToGo > 0) { this->stepsToGo--; }
				this->incrSeq();
				this->lastStep = 0;
			}
		} else if (this->stepSpeed < 0)
		{
			if (this->lastStep >= -this->stepSpeed) 
			{  
				if (this->stepsToGo > 0) { this->stepsToGo--; }
				this->decrSeq();
				this->lastStep = 0;
			}
		}
	}
	n = this->sequence[this->stepState];	// grab the current switch state
	for (i = 0; i < __channelsPerMotor__; i++)
	{
		j = this->channels[i];		// get the channel number
		k = j >> 3;					// bytes in which this channel is (equivalent to divide by 8)
		m = j % 8;					// get the bit within that bytes		 
		if (k > (bytesPerBoard - 1)) { break; }	// if we're going to break out of our
													// array, then break out of the loop
		if ((n >> i) & 0x1) 		// if we need to set this bit
		{
			boardBytes[k] |= (0x1 << m);	// set the mth bit of the kth byte
		} else
		{
			boardBytes[k] &= ~(0x1 << m);	// clear the mth bit of the kth byte
		}
	}
}

/*!
  \return a byte describing the state of the four switches that drive the stepper
  motor. 
  \sa shiftStepMotor::shiftStepMotor()
 */

 uint8_t	shiftStepMotor::getSeqStep (void)
{
	return this->sequence[this->stepState];
}

////////////////
// shiftBoard //
////////////////

// Public Members //

/*!
  \return The number of devices associated with this board
 */

uint8_t shiftBoard::getDevCount (void)
{
	return this->devCount;
}

/*!
  \param index The index of the desired device, relative to the board
  \return A pointer to the device at index
  \sa class shiftDevice
 */

shiftDevice* shiftBoard::getDev (uint8_t index)
{
	return this->devices[index];
}

/*!
  \return The number of bytes of shift register on this board
 */
		
uint8_t shiftBoard::getBoardSize (void)
{ 
	return this->byteCount;
}

//////////////////
// shiftSixteen //
//////////////////

// Constructor //

/*!
  \param devCount The number of individual devices attached to this board.
  \param **devices An array of pointers to the device objects. This array and the
  objects it contains must be allocated either statically or at global scope.
 */

shiftSixteen::shiftSixteen (uint8_t devCount, 
	shiftDevice **devices) 
{
	this->byteCount		= __shiftSixteenBytes__;
	//this->byteCount		= 2;
	this->firstChan		= __shiftSixteenFirst__;
	this->lastChan		= __shiftSixteenLast__;
	this->devCount		= devCount;
	this->devices		= devices;
}

// Public Members //

/*!
  \param chainSize The number of bytes in the entire daisy chain of boards
  \param firstByte The index of the first byte assigned to this board
  \param *chainBytes The array of bytes that will be shifted out to the chain.
 */

void shiftSixteen::doBoardTick(uint8_t chainSize, 
	uint8_t firstByte, 
	uint8_t *chainBytes)
{
	uint8_t boardBytes[__shiftSixteenBytes__] = {0};
	uint8_t i, devCnt;
	
	// cache these for some speed
	devCnt = this->devCount;
	
	for (i = 0; i < devCnt; i++)
	{
		devices[i]->doDevTick(__shiftSixteenBytes__, boardBytes);
	}
	
	chainBytes[firstByte + 1] 	= boardBytes[0];	// flip the order of the bytes
	chainBytes[firstByte] 		= boardBytes[1];	// so everything goes out in the right order
	
}

//////////////
// shiftSix //
//////////////

// Constructor //

/*!
  \param devCount The number of individual devices attached to this board.
  \param **devices An array of pointers to the device objects. This array and the
  objects it contains must be allocated either statically or at global scope.
 */

shiftSix::shiftSix (uint8_t devCount, 
	shiftDevice **devices) 
{
	this->byteCount		= __shiftSixBytes__;
	//this->byteCount		= 2;
	this->firstChan		= __shiftSixFirst__;
	this->lastChan		= __shiftSixLast__;
	this->devCount		= devCount;
	this->devices		= devices;
}

// Public Members //

/*!
  \param chainSize The number of bytes in the entire daisy chain of boards
  \param firstByte The index of the first byte assigned to this board
  \param *chainBytes The array of bytes that will be shifted out to the chain.
 */
 
void shiftSix::doBoardTick(uint8_t chainSize, 
	uint8_t firstByte, 
	uint8_t *chainBytes)
{
	uint8_t boardBytes[__shiftSixBytes__] = {0};
	uint8_t i, devCnt;
	
	// cache these for some speed
	devCnt = this->devCount;
	
	for (i = 0; i < devCnt; i++)
	{
		devices[i]->doDevTick(__shiftSixBytes__, boardBytes);
	}
	
	chainBytes[firstByte] 	= boardBytes[0];
	
}

////////////////
// ShiftChain //
////////////////

// Constructor //

/*!
  \param boardCount
  \param **boards
  \param dataPin
  \param clockPin
  \param latchPin
  \param latchPin
  \param resetPin
 */

shiftChain::shiftChain (uint8_t	boardCount, 
	shiftBoard	**boards,
	uint8_t dataPin, 
	uint8_t clockPin,
	uint8_t latchPin,
	uint8_t resetPin)
{
	int i;
	volatile uint8_t *reg;
	
	this->boards 			= boards;
	this->boardCount		= boardCount;			
	this->timerPrescaler	= 0;		
	this->timerVal			= 0;
	this->dataPort			= digitalPinToPort(dataPin);
	this->clockPort			= digitalPinToPort(clockPin);		
	this->latchPort			= digitalPinToPort(latchPin);
	this->resetPort			= digitalPinToPort(resetPin);
	this->dataPinMask		= digitalPinToBitMask(dataPin);	
	this->clockPinMask		= digitalPinToBitMask(clockPin);
	this->latchPinMask		= digitalPinToBitMask(latchPin);
	this->resetPinMask		= digitalPinToBitMask(resetPin);
	this->clk				= portOutputRegister(this->clockPort);
	this->dat				= portOutputRegister(this->dataPort);
	this->lat				= portOutputRegister(this->latchPort);
	this->rst				= portOutputRegister(this->resetPort);
	this->byteCount			= 0;
	this->timer				= 2;
	this->indexFlag			= 0;
	
	// time to set things to output...
	reg = portModeRegister(digitalPinToPort(dataPin));
	*reg |= digitalPinToBitMask(dataPin);
	reg = portModeRegister(digitalPinToPort(clockPin));
	*reg |= digitalPinToBitMask(clockPin);
	reg = portModeRegister(digitalPinToPort(latchPin));
	*reg |= digitalPinToBitMask(latchPin);
	reg = portModeRegister(digitalPinToPort(resetPin));
	*reg |= digitalPinToBitMask(resetPin);
	
	
	for(i = 0; i < this->boardCount; i++)
	{
		this->byteCount += this->boards[i]->getBoardSize();
	}
}

shiftChain::shiftChain (uint8_t	boardCount, 
	shiftBoard	**boards,
	uint8_t dataPin, 
	uint8_t clockPin,
	uint8_t latchPin,
	uint8_t resetPin,
	uint8_t indexPin)
{
	int i;
	volatile uint8_t *reg;
	
	this->boards 			= boards;
	this->boardCount		= boardCount;			
	this->timerPrescaler	= 0;		
	this->timerVal			= 0;
	this->dataPort			= digitalPinToPort(dataPin);
	this->clockPort			= digitalPinToPort(clockPin);		
	this->latchPort			= digitalPinToPort(latchPin);
	this->resetPort			= digitalPinToPort(resetPin);
	this->indexPort			= digitalPinToPort(indexPin);
	this->dataPinMask		= digitalPinToBitMask(dataPin);	
	this->clockPinMask		= digitalPinToBitMask(clockPin);
	this->latchPinMask		= digitalPinToBitMask(latchPin);
	this->resetPinMask		= digitalPinToBitMask(resetPin);
	this->indexPinMask		= digitalPinToBitMask(indexPin);
	this->clk				= portOutputRegister(this->clockPort);
	this->dat				= portOutputRegister(this->dataPort);
	this->lat				= portOutputRegister(this->latchPort);
	this->rst				= portOutputRegister(this->resetPort);
	this->ind				= portOutputRegister(this->indexPort);
	this->byteCount			= 0;
	this->timer				= 2;
	this->indexFlag			= -1;
	
	// time to set things to output...
	reg = portModeRegister(digitalPinToPort(dataPin));
	*reg |= digitalPinToBitMask(dataPin);
	reg = portModeRegister(digitalPinToPort(clockPin));
	*reg |= digitalPinToBitMask(clockPin);
	reg = portModeRegister(digitalPinToPort(latchPin));
	*reg |= digitalPinToBitMask(latchPin);
	reg = portModeRegister(digitalPinToPort(resetPin));
	*reg |= digitalPinToBitMask(resetPin);
	reg = portModeRegister(digitalPinToPort(indexPin));
	*reg |= digitalPinToBitMask(indexPin);
	
	
	for(i = 0; i < this->boardCount; i++)
	{
		this->byteCount += this->boards[i]->getBoardSize();
	}
}

// Private Members //

void shiftChain::fastShiftOut (uint8_t *bytes)
{
	uint8_t i, j;
	//int8_t k;		// if this is unsigned, the inner loop is infinite
	uint8_t clkMask, datMask;
	volatile uint8_t *clkLoc, *datLoc;
	
	// cache the masks to save 20 usec per 16 ch board
	clkMask = this->clockPinMask;
	datMask = this->dataPinMask;
	
	// a little more cache for speed...
	clkLoc = this->clk;
	datLoc = this->dat;
	
	// start the outbound shift
	*rst |= this->resetPinMask;
	*lat &= ~(this->latchPinMask);
	*datLoc &= ~(datMask);
	*clkLoc &= ~(clkMask);
	
	for (i = 0; i < this->byteCount; i++) 
	{
		// unrolling this inner loop saves 30 usec per 16 ch board
		/*for (k = 7; k >= 0; k--) 
		{
			*clk &= ~(this->clockPinMask);
			if (bytes[i] & (1 << k))
			{
				*dat |= (this->dataPinMask);
			} else {
				*dat &= ~(this->dataPinMask);
			}
			*clk |= (this->clockPinMask);
		}*/
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x80)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x40)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x20)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x10)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x8)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x4)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x2)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
		*clkLoc &= ~(clkMask);
		if (bytes[i] & 0x1)
		{
			*datLoc |= (datMask);
		} else {
			*datLoc &= ~(datMask);
		}
		*clkLoc |= (clkMask);
	}
	
	*lat |= this->latchPinMask;
	*clkLoc &= ~(clkMask);
}

// Public Members //

uint8_t shiftChain::startTimer (uint8_t prescaler, uint8_t timerVal, uint8_t timer) 
{
	
	this->timerVal = timerVal;
	this->timer = timer;
	
	if (timer == 2) 
	{
		TIMSK2 &= ~(1<<TOIE2);					// disable the overflow interrupt while we set things up
		TCCR2A &= ~((1<<WGM21) | (1<<WGM20));	// set counter mode to normal, counting up
		TCCR2B &= ~(1<<WGM22);					// set counter mode to normal, counting up
		ASSR &= ~(1<<AS2);						// set clock source to I/O clock 
		TIMSK2 &= ~((1<<OCIE2A) | (1<<OCIE2B));	// disable compare match interrupt
		TCCR2B |= (1<<CS22)  | (1<<CS21) | (1<<CS20);    	// set prescaler to 1024
		switch (prescaler) {
			case __preScaler32__:
				TCCR2B &= ~(1<<CS22);		// set prescaler to 32
				break;
			case __preScaler64__:
				TCCR2B &= ~(1<<CS21);		
				TCCR2B &= ~(1<<CS20);		// set prescaler to 64
				break;
			case __preScaler128__:
				TCCR2B &= ~(1<<CS21);		// set prescaler to 128
				break;
			case __preScaler256__:
				TCCR2B &= ~(1<<CS20);		// set prescaler to 256
				break;
			case __preScaler1024__:			// leave prescaler at 256
			default:
				break;
		}
		TCNT2 = timerVal;			
		TIMSK2 |= (1<<TOIE2);				// enable the overflow interrupt 
		return 0;
	} else { return -1; }
}

uint8_t shiftChain::stopTimer (void)
{
	if (this->timer == 2)
	{
		TIMSK2 &= ~(1<<TOIE2);
		return 0;
	} else { return -1; }
}

uint8_t shiftChain::getBoardCount (void)
{
	return (this->boardCount);
}

shiftBoard* shiftChain::getBoard (uint8_t index)
{
	return this->boards[index];
}

void shiftChain::doTick (void)
{
	uint8_t i, byteCnt, boardCnt, bytesSoFar = 0;
	uint8_t outBytes[this->byteCount];
	
	// make local copies of commonly accessed members to speed things up
	byteCnt = this->byteCount;
	boardCnt = this->boardCount;
	
	// set indicator pin to allow us to time the execution
	if (this->indexFlag) 
	{
		*ind |= (this->indexPinMask);
	}
	
	switch (this->timer) {
		case (2):
			TCNT2 = this->timerVal;
			break;
	}	
	
	/*for(i = 0; i < byteCnt; i++)
	{
		outBytes[i] = 0;
	}*/
	for(i = 0; i < boardCnt; i++)
	{
		this->boards[i]->doBoardTick(byteCnt, bytesSoFar, outBytes);
		bytesSoFar += this->boards[i]->getBoardSize();
	}
	
	this->fastShiftOut(outBytes);
	
	// clear indicator pin to allow us to time the execution
	if (this->indexFlag) 
	{
		*ind &= ~(this->indexPinMask);
	}
	
}

// This is some strange linker food required to make it all work
// see http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);}; 
void __cxa_guard_release (__guard *g) {*(char *)g = 1;}; 
void __cxa_guard_abort (__guard *) {}; 