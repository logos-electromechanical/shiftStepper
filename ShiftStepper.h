/*
  ShiftStepper.h - ShiftStepper library for Arduino
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

#ifndef ShiftStepper_h
#define ShiftStepper_h

#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <inttypes.h>

	/// Number of channels in each shiftSwitchBlock device
#define __channelsPerSwitchBlock__		4
	/// Number of channels each stepper motor consumes
#define __channelsPerMotor__			4 		
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

	/// Default step sequence for full stepping
static const uint8_t default4StepSequence[4] = {0x1, 0x2, 0x4, 0x8};					
	///	Default step sequence for half stepping
static const uint8_t default8StepSequence[8] = {0x1,0x3,0x2,0x6,0x4,0xc,0x8,0x9};		

	/// Abstract base class for creating devices such as motors that attach to a board
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
class shiftStepMotor : public shiftDevice
{
	private:
		uint8_t		stepState;		// Current index in the step sequence
		int16_t		stepSpeed;		// Sign gives directions, __maxMotorSpeed - magnitude + 1 is the number of timer ticks per step
		uint8_t		lastStep;		// Number of timer ticks since last step
		int16_t	stepsToGo;		// Number of steps to go until we finish the current set; set to -1 for continuous rotation
		const uint8_t		*sequence;		// An array of pin configurations that correspond to the steps
		uint8_t		seqSteps;		// Number of steps in the sequence of switch positions
		uint8_t		channels[__channelsPerMotor__];		/// The array of channels in the board that correspond to the drive channels for this motor, measured in bits
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
class shiftSwitchBlock : public shiftDevice
{
	private:
		uint8_t		channels[__channelsPerSwitchBlock__];		
		uint8_t		chanCount;		
	public:
		shiftStepMotor (const uint8_t *channels);
			/// Set switch positions
		uint8_t		setSwitches (uint8_t positions);
			/// Get switch positions
		uint8_t		getSwitches (void);				
			/// Local implementation of per timer tick behavior
		void 		doDevTick (uint8_t bytesPerBoard, 
								uint8_t *boardBytes);
}
	/// Abstract base class for individual boards
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
class shiftSixteen : public shiftBoard
{
	public:
			/// Create a ShiftSixteen object
		shiftSixteen (
						/// Number of devices 
					uint8_t devCount, 
						/// Array of pointers to device objects
					shiftDevice **devices);
			/// Do a timer tick for a sixteen channel board
		void doBoardTick(
							/// Total number of bytes in the chain
						uint8_t chainSize, 
							/// First byte in chainBytes that this function writes to
						uint8_t firstByte, 
							/// Array of bytes to be shifted out
						uint8_t *chainBytes);
};

	/// Implementation of shiftBoard for a six channel high current Arduino shield
class shiftSix : public shiftBoard
{
	public:
			/// Create a ShiftSix object
		shiftSix (
						/// Number of devices 
					uint8_t devCount, 
						/// Array of pointers to device objects
					shiftDevice **devices);
			/// Do a timer tick for a sixteen channel board
		void doBoardTick(
							/// Total number of bytes in the chain
						uint8_t chainSize, 
							/// First byte in chainBytes that this function writes to
						uint8_t firstByte, 
							/// Array of bytes to be shifted out
						uint8_t *chainBytes);
};

	/// Class to represent a daisy chain of boards
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
						/// The number of boards in the chain
					uint8_t	boardCount, 
						/// Array of pointers to ShiftBoard objects
					shiftBoard	**boards, 
						/// Arduino pin number to be used for the data line
					uint8_t dataPin, 
						/// Arduino pin number to be used for the clock line
					uint8_t clockPin,
						/// Arduino pin number to be used for the latch line
					uint8_t latchPin,
						///  Arduino pin number to be used for the master reset line
					uint8_t resetPin);
			/// Create a new daisy chain object, with an index output
		shiftChain (
						/// The number of boards in the chain
					uint8_t	boardCount, 
						/// Array of pointers to ShiftBoard objects
					shiftBoard	**boards, 
						/// Arduino pin number to be used for the data line
					uint8_t dataPin, 
						/// Arduino pin number to be used for the clock line
					uint8_t clockPin,
						/// Arduino pin number to be used for the latch line
					uint8_t latchPin,
						///  Arduino pin number to be used for the master reset line
					uint8_t resetPin,
						/// Arduino pin number to be used for an index output
					uint8_t indexPin);				
			/// Get the number of boards in the chain
		uint8_t		getBoardCount (void);
			/// Get a pointer to a particular board in the chain
		shiftBoard	*getBoard (uint8_t index);
			/// Start the timer
		uint8_t 	startTimer (
						/// Select the timer pre-scaler. Defaults to 1024
					uint8_t prescaler, 
						/// Select a value for the timer to reset to, for precise control of frequency
					uint8_t timerVal, 
						/// Select the timer to use; only Timer 2 works currently
					uint8_t timer);
			/// Shut down the current timer
		uint8_t		stopTimer (void);
			/// Do a timer tick -- call this from an ISR.
		void 		doTick (void);
};

// This is some strange linker food required to make it all work
// see http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870

__extension__ typedef int __guard __attribute__((mode (__DI__))); 

extern "C" int __cxa_guard_acquire(__guard *); 
extern "C" void __cxa_guard_release (__guard *); 
extern "C" void __cxa_guard_abort (__guard *); 

#endif