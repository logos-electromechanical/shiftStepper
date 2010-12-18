/*
  ShiftStepper.h - ShiftStepper library for Arduino
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

#ifndef ShiftStepper_h
#define ShiftStepper_h

#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <inttypes.h>

	/// Default number of channels in each shiftSwitchBlock device
#define __channelsPerSwitchBlockDefault__		4
	/// Max number of channels in each shiftSwitchBlock device
#define __channelsPerSwitchBlockMax__			8
	/// Number of channels each stepper motor consumes
#define __channelsPerMotor__				4 		
	/// Maximum speed of each motor. 
#define __maxMotorSpeed__				255		
	/// Sets timer pre-scaler to 32
#define __preScaler32__					3		
	/// Sets timer pre-scaler to 64
#define __preScaler64__					4		
	/// Sets timer pre-scaler to 128
#define __preScaler128__				5		
	/// Sets timer pre-scaler to 256
#define __preScaler256__				6		
	/// Sets timer pre-scaler to 1024
#define __preScaler1024__				7		

// defines for the sixteen channel board
	/// The number of bytes on a sixteen channel board
#define __shiftSixteenBytes__ 	2
	/// The index of the lowest channel on a six channel board
#define __shiftSixteenFirst__ 	0
	/// The index of the highest channel on a six channel board
#define __shiftSixteenLast__	15

// defines for the six channel board
	/// The number of bytes on a six channel board
#define __shiftSixBytes__ 		1
	/// The index of the lowest channel on a six channel board
#define __shiftSixFirst__ 		1
	/// The index of the highest channel on a six channel board
#define __shiftSixLast__		6


	/// Default step sequence for full stepping
	/*!
	  See http://www.cs.uiowa.edu/~jones/step/types.html#unipolar for more details
	 */
static const uint8_t default4StepSequence[4] = {0x1, 0x2, 0x4, 0x8};					
	///	Default step sequence for half stepping
	/*!
	  See http://www.cs.uiowa.edu/~jones/step/types.html#unipolar for more details
	 */
static const uint8_t default8StepSequence[8] = {0x1,0x3,0x2,0x6,0x4,0xc,0x8,0x9};		

	/// Abstract base class for creating devices such as motors that attach to a board
	/*!
	  A shiftDevice object represents an object controlled by a shift register board. 
	 */
class shiftDevice
{
	protected:	
			/// Array of channels in the object relative to the board
			/*! 
			  This array holds the channel numbers. For example, a unipolar step motor
			  might consume channel 1, 2, 3, and 4. This array would then be equal to 
			  {1, 2, 3, 4}.
			 */
		uint8_t 	*channels;		
			/// Number of channels per device, in device units
			/*!
			  For example, a unipolar step motor consumes four switch channels.
			 */
		uint8_t 	chanCount;		
	public:
			/// Handle for the per timer tick behavior of each device.
			/*!
			  This function is intended to be called from shiftBoard::doBoardTick(), 
			  which passes it the array of bytes that it expects it to write the data
			  to be shifted out to.
			  \param bytesPerBoard is the number of outgoing bytes for the board this device is part of.
			  \param *boardBytes is the array of outgoing bytes for the board this device is part of.
			  
			  Must be re-implemented by each sub-class.
			*/
		virtual void 	doDevTick (uint8_t bytesPerBoard, 
								   uint8_t *boardBytes);
			/// Get the byte array that holds the channels that the device occupies
		uint8_t 		*getChannels (void);	
			/// Get the number of channels the device occupies
		uint8_t 		getChanCount (void);	/// gets the number of channels, in object units 
};	

	/// Implementation of a 4-phase unipolar stepper motor device
	/*!
	
	 */
class shiftStepMotor : public shiftDevice
{
	private:
		uint8_t		stepState;		// Current index in the step sequence
		int16_t		stepSpeed;		// Sign gives directions, __maxMotorSpeed - magnitude + 1 is the number of timer ticks per step
		uint8_t		lastStep;		// Number of timer ticks since last step
		int16_t		stepsToGo;		// Number of steps to go until we finish the current set; set to -1 for continuous rotation
		const uint8_t		*sequence;		// An array of pin configurations that correspond to the steps
		uint8_t		seqSteps;		// Number of steps in the sequence of switch positions
		uint8_t		channels[__channelsPerMotor__];		// The array of channels in the board that correspond to the drive channels for this motor, measured in bits
		uint8_t		chanCount;		// Number of channels in the array 
		void 		incrSeq (void); // Increment the step sequence by 1
		void 		decrSeq	(void); // Decrement the step sequence by 1
	public:
			/// Creates a shiftStepMotor object
		shiftStepMotor (
							// the number of steps in the sequence 
						const uint8_t seqSteps,		
							// an array of switch position, expressed as integers
						const uint8_t *sequence, 	
							// the channel numbers on the board occupied by this device (always 4 elements long)
						const uint8_t *channels);	
			/// Go one step in the commanded direction
		uint8_t		incrStep (int8_t dir);						
			/// Command a move of steps steps at speed speed.
		uint8_t		doSteps (int16_t steps, int16_t speed);	
			/// Set motor speed and direction
		uint8_t		setSpeed (int16_t speed);				
			/// Get the current speed
		int16_t 	getSpeed (void);							
			/// Get the number of steps to go in the current move
		int16_t 	getStepsToGo (void);						
			/// Returns the current output switch configuration
		uint8_t		getSeqStep (void);							
			/// Local implementation of per timer tick behavior
		void 		doDevTick (uint8_t bytesPerBoard, 
							   uint8_t *boardBytes);
};
	/// A block of plain switches
	/*!
	
	 */
class shiftSwitchBlock : public shiftDevice
{
	private:
			/// Switch positions
		uint8_t switches;
	public:
			/// Create a new shiftSwichBlock object, with the default number of channels.
		shiftSwitchBlock (uint8_t *channels);
			/// Create a new shiftSwitchBlock object, with a variable number of channels.
		shiftSwitchBlock (uint8_t chanCount, 
						  uint8_t *channels);
			/// Set switch positions
		uint8_t		setSwitches (uint8_t positions);
			/// Get switch positions
		uint8_t		getSwitches (void);				
			/// Local implementation of per timer tick behavior
		void 		doDevTick (uint8_t bytesPerBoard, 
							   uint8_t *boardBytes);
};

	/// Abstract base class for individual boards with shiftDevice.
	/*!
	  This base class covers boards which contain one or more shiftDevice objects.
	 */
class shiftBoard
{
	protected:
			/// Number of bytes of shift register on the board 
		uint8_t			byteCount;	
			/// First active channel/byte/bit on the board
		uint8_t			firstChan;	
			/// Last active channel/byte/bit on the board
		uint8_t 		lastChan;	
			/// An array of pointers to all of the objects associated with this board
		shiftDevice		**devices;	
			/// Number of entries in the device array
		uint8_t			devCount;	
	public:
			/// Handle for per timer tick behavior of each board
		virtual void 	doBoardTick (uint8_t chainSize, 
									 uint8_t firstByte, 
									 uint8_t *chainBytes);
			/// Get the number of objects associated with the board
		uint8_t			getDevCount (void);			
			/// Get pointer to device at index
		shiftDevice 	*getDev (uint8_t index);	
			/// Get the size of the board's datastream, in bytes
		uint8_t			getBoardSize (void);		
};

	/// Implementation of shiftBoard for a sixteen channel high current Arduino shield
	/*!
	
	 */
class shiftSixteen : public shiftBoard
{
	public:
			/// Create a ShiftSixteen board object
		shiftSixteen (
					uint8_t devCount, 
					shiftDevice **devices);
			/// Do a timer tick for a sixteen channel board
		void doBoardTick(
						uint8_t chainSize, 
						uint8_t firstByte, 
						uint8_t *chainBytes);
};

	/// Implementation of shiftBoard for a six channel high current Arduino shield
	/*!
	
	 */
class shiftSix : public shiftBoard
{
	public:
			/// Create a new shiftSix board object
		shiftSix (
					uint8_t devCount, 
					shiftDevice **devices);
			/// Do a timer tick for a six channel board
		void doBoardTick(
						uint8_t chainSize, 
						uint8_t firstByte, 
						uint8_t *chainBytes);
};

	/// Abstract base class for individual boards, without shiftDevice objects.
	/*!
	  This base class differs from shiftBoard in that it is designed to enable the user
	  to write directly to the bits shifted to the board, without the intervention of
	  any devices. This will be faster in applications that do not require the 
	  bookkeeping of stepper applications or similar. 
	  
	  
	 */
class shiftBoardDirect
{
	protected:
			/// Number of bytes of shift register on the board 
		uint8_t			byteCount;	
			/// First active channel/byte/bit on the board
		uint8_t			firstChan;	
			/// Last active channel/byte/bit on the board
		uint8_t 		lastChan;	
			/// Array of bytes in the board
		uint8_t			*bytes;
	public:
			/// Handle for per timer tick behavior of each board
		virtual void 	doBoardTick (uint8_t chainSize, 
									 uint8_t firstByte, 
									 uint8_t *chainBytes);
			/// Get the size of the board's datastream, in bytes
		uint8_t			getBoardSize (void);	
			/// Set the board's output
		uint8_t			setBoardOutput (uint8_t *bytes);
};

	/// Implementation of shiftBoardDirect for a sixteen channel high current Arduino shield
	/*!
	
	 */
class shiftSixteenDirect : public shiftBoardDirect
{
	private:
		uint8_t bytes[__shiftSixteenBytes__];
	public:
			/// Create a ShiftSixteenDirect board object with the state defined in *bytes.
		shiftSixteenDirect (uint8_t *bytes);
			/// Create a ShiftSixteenDirect board object with a zero state.
		shiftSixteenDirect (void);
			/// Do a timer tick for a sixteen channel board
		void doBoardTick(
						uint8_t chainSize, 
						uint8_t firstByte, 
						uint8_t *chainBytes);
};

	/// Implementation of shiftBoardDirect for a six channel high current board
	/*!
	
	 */
class shiftSixDirect : public shiftBoardDirect
{
	private:
		uint8_t bytes[__shiftSixBytes__];
	public:
			/// Create a new shiftSixDirect board object with the state defined in *bytes.
		shiftSixDirect (uint8_t outByte);
			/// Create a new shiftSixDirect board object with a zero state.
		shiftSixDirect (void);
			/// Do a timer tick for a six channel board
		void doBoardTick(
						uint8_t chainSize, 
						uint8_t firstByte, 
						uint8_t *chainBytes);
};

	/// Class to represent a daisy chain of boards
	/*!
	  Each shiftChain object stores all of the data required to create and use a daisy
	  chained group of shift register controlled modules, devices, or boards. 
	 */
class shiftChain
{
	private:
		shiftBoard	**boards;			// array of pointers to the board objects
		uint8_t		boardCount;			// number of boards in the chain
		uint8_t		byteCount;			// nuber of bytes in the chain
		uint8_t		timerPrescaler;		// prescaler value to set timer frequency
		uint8_t		timerVal;			// the value the timer starts off at after each tick
										// together, these two values control the tick frequency
		uint8_t		timer;				// the timer number we're using
		uint8_t		dataPort;			// shift register data pin
		uint8_t		clockPort;			// shift register clock pin
		uint8_t		latchPort;			// shift register latch pin
		uint8_t		resetPort;			// shift register master reset pin
		uint8_t		indexPort;			// index pin to indicate when the interrupt is running
		uint8_t		dataPinMask;		// shift register data pin
		uint8_t		clockPinMask;		// shift register clock pin
		uint8_t		latchPinMask;		// shift register latch pin
		uint8_t		resetPinMask;		// shift register master reset pin
		uint8_t		indexPinMask;		// index pin
		uint8_t		indexFlag;			// flag to determine whether or not index pin is enabled
		volatile uint8_t 	*clk;		// port register for clock pin
		volatile uint8_t 	*dat;		// port register for data pin
		volatile uint8_t 	*lat;		// port register for latch pin
		volatile uint8_t 	*rst;		// port register for master reset pin
		volatile uint8_t	*ind;		// port register for index pin
		void 		fastShiftOut (uint8_t *bytes);
										// shift them bits out quick!
	public: 
			/// Create a new daisy chain object, without an index output
		shiftChain (
					uint8_t	boardCount, 
					shiftBoard	**boards, 
					uint8_t dataPin, 
					uint8_t clockPin,
					uint8_t latchPin,
					uint8_t resetPin);
			/// Create a new daisy chain object, with an index output
		shiftChain (
					uint8_t	boardCount, 
					shiftBoard	**boards, 
					uint8_t dataPin, 
					uint8_t clockPin,
					uint8_t latchPin,
					uint8_t resetPin,
					uint8_t indexPin);				
			/// Get the number of boards in the chain
		uint8_t		getBoardCount (void);
			/// Get a pointer to a particular board in the chain
		shiftBoard	*getBoard (uint8_t index);
			/// Start the timer
		uint8_t 	startTimer (
					uint8_t prescaler, 
					uint8_t timerVal, 
					uint8_t timer);
			/// Shut down the current timer
		uint8_t		stopTimer (void);
			/// Do a timer tick -- call this from an ISR.
		void 		doTick (void);
};

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */

__extension__ typedef int __guard __attribute__((mode (__DI__))); 

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */

extern "C" int __cxa_guard_acquire(__guard *); 

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */

extern "C" void __cxa_guard_release (__guard *); 

/*!
  This is some strange linker food required to make it all work.
  
  See http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 */

extern "C" void __cxa_guard_abort (__guard *); 

#endif