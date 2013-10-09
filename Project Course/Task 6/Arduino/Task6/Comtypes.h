/* ComTypes.h - Communication classes header file
*  MRSD Team B
*  Project Class Task 6
*/

#ifndef COMTYPES_H
#define COMTYPES_H

#import <Arduino.h>
#import "pinout.h"
#import "Servo.h"

//16 bit midpoint value
#define MID_16 0x8000

class InputChannel {
	private:
	uint16_t value;
	
	public:
	void setValue(uint16_t val);
	void set10BitValue(uint16_t val);
	uint16_t getValue();
};

class OutputChannel {
	protected:
	InputChannel * source;
	
    public:
	virtual ~OutputChannel() {}
	virtual void setValue(uint16_t val) = 0;
	void attachInput(InputChannel * src);
	void updateChannel();
};

class LEDChannel : public OutputChannel {
	public:
	void setValue(uint16_t val);
};

//Wrap the built in servo class to use the Channel Schema
class ServoChannel : public OutputChannel, public Servo  {
	public:
	void setValue(uint16_t val);
};

class Packet {
	public:
	
	Packet();

	union {
		struct {
			uint16_t header;
			uint8_t inChannel;
			uint8_t outChannel;
			uint16_t channel[8];
			uint16_t checksum;
			uint16_t footer;
		};
		uint8_t chars[24];
	};

	bool isValid();
	bool isError();
	void pack();
	void transmit();
};



#endif