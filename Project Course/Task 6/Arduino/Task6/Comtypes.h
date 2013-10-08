/* ComTypes.h - Communication classes header file
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "pinout.h"

#ifndef COMTYPES_H
#define COMTYPES_H

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
	private:
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

class Packet {
	public:
	
	Packet();

	union {
		struct {
			uint16_t header;
			uint8_t in_chan;
			uint8_t out_chan;
			uint16_t channel[8];
			uint16_t checksum;
			uint16_t footer;
		};
		uint8_t chars[24];
	};

	bool is_valid();
	bool is_error();
	void pack();
	void transmit();
};



#endif