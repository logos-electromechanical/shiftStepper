/*
  ShiftStepper.cpp - ShiftStepper library for Arduino
  v0.02a Alpha
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

/*!
  \mainpage
  \section intro Introduction
  This library is written for the line of shift register controlled output boards 
  produced by Logos Electromechanical, LLC. It is intended to be flexible enough to 
  support all of the shift register interface boards current in production and 
  planned for the future. Therefore, it has pieces that are not strictly necessary 
  for those currently on the market.

  The highest level object in this library is shiftChain, generally referred to as a
  chain. A chain is a set of one or more shift register boards. The farthest from 
  the host (along the daisy chain cable) is board 0 and the one closest to the host
  has the highest index. This little endian structure mirrors the operation of the 
  shift register chain, i.e. the first bit shifted ends up the farthest from the 
  host at the end of the shifting operation. Each board, represented by a shiftBoard
  object, contains one or more devices, each represented by a shiftDevice object. 
  A device is a group of one or more channels that operate togeter -- for example, 
  the four switches that drive a unipolar stepper motor. The only device types currently implemented are the shiftStepMotor and 
  shiftSwitchBlock. 
  
  A channel is the smallest logical element on the board. On all 
  of the shift register boards so far, a channel is a single switch controlled by a 
  single bit. Future boards may have channels that involve many more bits; hence the
  term should be taken as generic.

  New data is shifted out to the chain by calling shiftChain::doTick(). This can be called 
  from inside the loop() function, but for the best stepper performance, it should be 
  called by a timer interrupt service routine. The shiftChain object has member functions 
  for setting up the timer correctly, but the user must supply the interrupt service 
  routine. Only Timer 2 is implemented so far; adding more is also on the TODO list. 

  shiftChain::doTick() calls the shiftBoard::doBoardTick() for each board in the chain. In 
  turn, each example of shiftBoard::doBoardTick() calls shiftDevice::doDeviceTick() for 
  each device in that board. These function are defined as virtual functions in the 
  shiftBoard and shiftDevice classes; individual boards and devices are implemented 
  subclasses of these two classes, respectively, and must implement 
  shiftBoard::doBoardTick() and shiftDevice::doDeviceTick().

  The aim of all of this is to allow each board and each applicaiton of that board to be 
  separately defined in a way that still plays nice together in a (potentially) long chain 
  of such device and allows the addition of new boards and applications without disrupting 
  source code based on this version of the library. 
  
  \section install Installation
  
  In order to install this library, copy this directory into the arduino libraries 
  directory.

  Then restart the Arduino IDE and it should appear in the Tools/Import Library 
  pulldown and in the File/Examples pulldown.
  
  \section using Using the Library
  
  The first step is declaring the devices that make up each board, each board, and 
  the chain, in that order. There are clever ways to do this, but the simplest and 
  most fool-proof way to do it is to declare all the objects at global scope. Here's
  how I do it in the included examples:

  \code
  static const uint8_t stepSequence[4] = {0x2, 0x4, 0x1, 0x8}; 

  static const uint8_t motChans0[__channelsPerMotor__] = {0,2,3,1}; 
  static const uint8_t motChans1[__channelsPerMotor__] = {4,5,6,7}; 
  static const uint8_t motChans2[__channelsPerMotor__] = {11,9,10,8}; 
  static const uint8_t motChans3[__channelsPerMotor__] = {15,14,13,12}; 

  shiftChain *myChain = 0; 
  shiftStepMotor motor0(__channelsPerMotor__, stepSequence, motChans0); 
  shiftStepMotor motor1(__channelsPerMotor__, stepSequence, motChans1); 
  shiftStepMotor motor2(__channelsPerMotor__, stepSequence, motChans2); 
  shiftStepMotor motor3(__channelsPerMotor__, stepSequence, motChans3); 
  shiftDevice *motors[4] = {&motor0, &motor1, &motor2, &motor3}; 
  shiftSixteen board0(4, motors); shiftBoard *boards[1] = {&board0}; 
  shiftChain storeChain(1, boards, DATPIN, SCLPIN, LATPIN, MRPIN, INDPIN);
  \endcode

  All of these declarations could alternately be made static in the setup() 
  function, with the exception of the declaration of *myChain, which has to be 
  global in order to be useful. There are no blank constructors defined for any of 
  these objects, so they must be fully initialized at declaration; that drives the 
  order of the declaration in the above snippet. The various arrays are declared as 
  global variables here due to the lack of proper array literals in C++.

  Once you've declared all the members of the chain, you define the interrupt 
  routine as shown above, assign the myChain pointer to point at storeChain, and 
  start the timer:

  \code
  myChain = &storeChain; 
  myChain->startTimer(__preScaler32__, 0, 2);
  \endcode
  
  After that, the interrupt will execute every time the selected timer interrupt 
  trips. What it does depends on what each device has been commanded to do. See the
  subclasses of shiftDevice for documentation of the command functions.
  
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
  members of sequence[] are interpreted. This array need not be permanently 
  allocated.
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
  This function manages the per-tick behavior of a stepper motor device. It is 
  expected to be called from the doBoardTick() function of the board that contains
  this device.
  
  It performs the following tasks:
	- Checks whether or not it's time to make a step, based on the values of 
	stepSpeed, lastStep, and stepsToGo
	- Writes the current switch state to the bits corresponding to the output 
	channels in the boardBytes[] array
	
  \param bytesPerBoard This tells the function how many bytes of shift register are
  in the output board.
  
  \param *boardBytes This is a pointer to the array of bytes that the doBoardTick() 
  function this is called from uses to write to the array of bytes that is then
  shifted out to the boards themselves.
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

//////////////////////
// shiftSwitchBlock //
//////////////////////

// Constructor //

/*!
  The default number of channels is set by __channelsPerSwitchBlockDefault__, 
  defined in shiftStepper.h. 
  \param *channels The array of channel numbers. This must have at least
  __channelsPerSwitchBlockDefault__ entries. It must be declared static or at 
  global scope, or otherwise permanently allocated. 
 */

shiftSwitchBlock::shiftSwitchBlock (uint8_t *chans)
{
	this->channels 		= chans;
	this->chanCount 	= __channelsPerSwitchBlockDefault__;
	this->switches		= 0;
}

/*!
  \param chanCount The number of channels that this device controls. 
  \param *channels The array of channel numbers. This must have at least chanCount 
  entries. It must be declared static or at global scope, or otherwise permanently 
  allocated. 
 */

shiftSwitchBlock::shiftSwitchBlock (uint8_t chanCnt, uint8_t *chans)
{
	if (chanCount > __channelsPerSwitchBlockMax__) 
	{
		this->chanCount = __channelsPerSwitchBlockMax__;
	} else
	{
		this->chanCount = chanCnt;
	}
	this->channels 		= chans;
	this->switches		= 0;
}

// Public Members //

/*!
  \param positions This is a bit string representing the position of the switches
  The LSB controls channels[0], the next controls channels[1], etc.
  \return Attempt status. This is not implemented yet; it always returns 0.
 */

uint8_t shiftSwitchBlock::setSwitches (uint8_t positions) 
{
	this->switches = positions;
	return 0;
}
	
/*!
  \return The bit string representing the current switch positions
 */
 
uint8_t shiftSwitchBlock::getSwitches (void)
{
	return this->switches;
}

/*!
  This function manages the per-tick behavior of a block of switches. It is 
  expected to be called from the doBoardTick() function of the board that contains
  this device.
  
  This function writes the switches specified in the channels array with the states
  specified in the bit string written with setSwitches().
  
  \param bytesPerBoard This tells the function how many bytes of shift register are
  in the output board.
  
  \param *boardBytes This is a pointer to the array of bytes that the doBoardTick() 
  function this is called from uses to write to the array of bytes that is then
  shifted out to the boards themselves.
 */

void shiftSwitchBlock::doDevTick (uint8_t bytesPerBoard, uint8_t *boardBytes)
{
	int i, j, k, m, n, x;
	n = this->chanCount;
	x = this->switches;
	for (i = 0; i < n; i++)
	{
		j = this->channels[i];		// get the channel number
		k = j >> 3;					// bytes in which this channel is (equivalent to divide by 8)
		m = j % 8;					// get the bit within that bytes		 
		if (k > (bytesPerBoard - 1)) { break; }	// if we're going to break out of our
												// array, then break out of the loop
		if ((x >> i) & 0x1) 		// if we need to set this bit
		{
			boardBytes[k] |= (0x1 << m);	// set the mth bit of the kth byte
		} else
		{
			boardBytes[k] &= ~(0x1 << m);	// clear the mth bit of the kth byte
		}
	}
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

//////////////////////
// shiftBoardDirect //
//////////////////////

// Public Members //

/*!
  \return The number of bytes of shift register on this board
 */
		
uint8_t shiftBoardDirect::getBoardSize (void)
{ 
	return this->byteCount;
}

/*!
  \param bytes An array of bytes representing the desired output state of the board. 
  Must have at least as many members as specified by the output of 
  shiftBoardDirect::getBoardSize()
  \return Exit status. Always returns 0 (success) now
 */

uint8_t	shiftBoardDirect::setBoardOutput (uint8_t *bytes)
{
	int i, j;
	j = this->byteCount;
	for (i = 0; i < j; i++)
	{
		this->bytes[i] = bytes[i];
	}
	return 0;
}

////////////////////////
// shiftSixteenDirect //
////////////////////////

// Constructors //

/*!
  \param *bytes An array of bytes defining the initial state of the output switches.
 */

shiftSixteenDirect::shiftSixteenDirect (uint8_t *bytes) 
{
	this->byteCount		= __shiftSixteenBytes__;
	this->firstChan		= __shiftSixteenFirst__;
	this->lastChan		= __shiftSixteenLast__;
	this->bytes[0]		= bytes[0];
	this->bytes[1]		= bytes[1];
}

/*!
  
 */

shiftSixteenDirect::shiftSixteenDirect (void) 
{
	this->byteCount		= __shiftSixteenBytes__;
	this->firstChan		= __shiftSixteenFirst__;
	this->lastChan		= __shiftSixteenLast__;
	this->bytes[0]		= 0;
	this->bytes[1]		= 0;
}

// Public Members //

/*!
  \param chainSize The number of bytes in the entire daisy chain of boards
  \param firstByte The index of the first byte assigned to this board
  \param *chainBytes The array of bytes that will be shifted out to the chain.
 */

void shiftSixteenDirect::doBoardTick(uint8_t chainSize, 
	uint8_t firstByte, 
	uint8_t *chainBytes)
{
	chainBytes[firstByte + 1] 	= this->bytes[0];	// flip the order of the bytes
	chainBytes[firstByte] 		= this->bytes[1];	// so everything goes out in the right order
}

////////////////////
// shiftSixDirect //
////////////////////

// Constructors //

/*!
  \param *bytes An array of bytes defining the initial state of the output switches.
 */

shiftSixDirect::shiftSixDirect (uint8_t outByte) 
{
	this->byteCount		= __shiftSixBytes__;
	this->firstChan		= __shiftSixFirst__;
	this->lastChan		= __shiftSixLast__;
	this->bytes[0]		= outByte;
}

/*!
  
 */

shiftSixDirect::shiftSixDirect (void) 
{
	this->byteCount		= __shiftSixBytes__;
	this->firstChan		= __shiftSixFirst__;
	this->lastChan		= __shiftSixLast__;
	this->bytes[0]		= 0;
}

// Public Members //

/*!
  \param chainSize The number of bytes in the entire daisy chain of boards
  \param firstByte The index of the first byte assigned to this board
  \param *chainBytes The array of bytes that will be shifted out to the chain.
 */

void shiftSixDirect::doBoardTick(uint8_t chainSize, 
	uint8_t firstByte, 
	uint8_t *chainBytes)
{
	chainBytes[firstByte] 	= this->bytes[0];	// flip the order of the bytes
}

////////////////
// ShiftChain //
////////////////

// Constructor //

/*!
  \param boardCount The number of boards attached to the chain
  \param **boards An array of pointers to the board objects. Storage for the array 
  and the board objects must be allocated before calling the constructor, such as 
  by declaring them statically or globally.
  \param dataPin The Arduino pin used to transmit data to the shift register(s). On 
  the sixteen channel board, this is pin 13.
  \param clockPin The Arduino pin used to clock the shift register(s). On the sixteen
  channel board, this is pin 12.
  \param latchPin The Arduino pin used to latch the shifted data into the shift 
  register output. On the sixteen channel board, this is pin 7.
  \param resetPin The Arduino pin that drives the master reset for the shift register
  chain. On the sixteen channel board, this is pin 8.
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

/*!
  \param boardCount The number of boards attached to the chain
  \param **boards An array of pointers to the board objects. Storage for the array 
  and the board objects must be allocated before calling the constructor, such as 
  by declaring them statically or globally.
  \param dataPin The Arduino pin used to transmit data to the shift register(s). On 
  the sixteen channel board, this is pin 13.
  \param clockPin The Arduino pin used to clock the shift register(s). On the sixteen
  channel board, this is pin 12.
  \param latchPin The Arduino pin used to latch the shifted data into the shift 
  register output. On the sixteen channel board, this is pin 7.
  \param resetPin The Arduino pin that drives the master reset for the shift register
  chain. On the sixteen channel board, this is pin 8.
  \param indexPin This pin turns on while shiftChain::doTick() is running. This allows
  precise timing measurement.
 */

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

/*!
  This starts the timer directed by the timer parameter with the given pre-scaler 
  and reset value. It also activates the overflow interrupt for the given timer. 
  \param prescaler Select the timer pre-scaler. This is the value that the main 
  processor clock is divided by to clock the timer. #define values are provide 
  for all the supported pre-scaler values:
	- __preScaler32__
	- __preScaler64__
	- __preScaler128__
	- __preScaler256__
	- __preScaler1024__
  If an un-supported value is passed in here, the pre scaler defaults to 1024.
  \param timerVal This is the value that the timer resets to after each time it 
  overflows. The number of ticks of the timer clock between overflows is equal to
  maxTimerValue - timerVal. Since only 8-bit timers are supported so far, this is
  an 8-bit value.
  \param timer The timer to start. The only supported timer right now is 2; selecting
  any other timer will result in an error.
  \return Returns a 0 if timer 2 is selected; otherwise -1.
 */
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
			case __preScaler1024__:			// leave prescaler at 1024
			default:
				break;
		}
		TCNT2 = timerVal;			
		TIMSK2 |= (1<<TOIE2);				// enable the overflow interrupt 
		return 0;
	} else { return -1; }
}

/*!
  \return This returns status. However, the error checking/status functionality is
  not implemented, so it always returns 0 (success).
 */

uint8_t shiftChain::stopTimer (void)
{
	if (this->timer == 2)
	{
		TIMSK2 &= ~(1<<TOIE2);
		return 0;
	} else { return -1; }
}

/*!
  \return The number of boards in this chain
 */

uint8_t shiftChain::getBoardCount (void)
{
	return (this->boardCount);
}

/*!
  \param index The array index of the desired board object
  \return A pointer to the selected board object
 */

shiftBoard* shiftChain::getBoard (uint8_t index)
{
	return this->boards[index];
}

/*!
  This can be called from a loop, but you get the most consistent results from calling
  it from an ISR, such as follows:
  \code
  ISR (TIMER2_OVF_vect)
      {
	      myChain->doTick();
      }
  \endcode
  Calling it in this fashion gives consistent speed from stepper motors and the like.
 */
 
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

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */
int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);}; 

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */
void __cxa_guard_release (__guard *g) {*(char *)g = 1;}; 

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */
void __cxa_guard_abort (__guard *) {}; 