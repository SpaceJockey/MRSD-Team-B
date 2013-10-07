/* ComTypes.cpp - Communication classes implementation
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Comtypes.h"

Packet::Packet() {
	//Init our packet frame
	this->header = 0xDEAD;
	this->footer = 0xBEEF;
}

//Check a packet to verify no transmission errors
bool Packet::is_valid(){
	//Check headers and footers
	if ((this->header != 0xDEAD) || (this->footer != 0xBEEF)) return false;
	
	//calculate checksum
	uint16_t checksum = this->in_chan + this->out_chan;
	for(int i = 0; i < 8; i++) checksum += this->channel[i];
	if(checksum != this->checksum) return false;

	//Looks ok!
	return true;
}

//pack for transmission
void Packet::pack(){
	//re-init our packet frame
	this->header = 0xDEAD;
	this->footer = 0xBEEF;
	
	//calculate checksum
	this->checksum = this->in_chan + this->out_chan;
	for(int i = 0; i < 8; i++) this->checksum += this->channel[i];
}

void Packet::transmit(){
	Serial.write(this->chars, sizeof(Packet));
}
