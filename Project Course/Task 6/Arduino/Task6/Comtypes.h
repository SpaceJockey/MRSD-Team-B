/* ComTypes.h - Communication classes header file
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>

#ifndef COMTYPES_H
#define COMTYPES_H

class Packet {
	public:
	
	Packet();

	union {
		struct {
			uint16_t header;
			uint8_t in_chan;
			uint8_t out_chan;
			int16_t channel[8];
			uint16_t checksum;
			uint16_t footer;
		} data;
		uint8_t chars[24];
	};

	bool is_valid();
	void pack();
	void transmit();
};



#endif