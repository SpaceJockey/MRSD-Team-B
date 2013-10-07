/* ComTypes.cpp - Communication classes implementation
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Comtypes.h"

Packet::Packet() {
	//Init our packet frame
	this->data.header = 0xDEAD;
	this-> data.footer = 0xBEEF;
}

//Check a packet to verify no transmission errors
bool Packet::is_valid(){
	//Check headers and footers
	if ((this->data.header != 0xDEAD) || (this->data.footer != 0xBEEF)) return false;
	
	//calculate checksum
	uint16_t checksum = this->data.in_chan + this->data.out_chan;
	for(int i = 0; i < 8; i++) checksum += this->data.channel[i];
	if(checksum != this->data.checksum) return false;

	//Looks ok!
	return true;
}

//pack for transmission
void Packet::pack(){
	//re-init our packet frame
	this->data.header = 0xDEAD;
	this-> data.footer = 0xBEEF;
	
	//calculate checksum
	this->data.checksum = this->data.in_chan + this->data.out_chan;
	for(int i = 0; i < 8; i++) this->data.checksum += this->data.channel[i];
}

void Packet::transmit(){
	Serial.write(this->chars, sizeof(Packet));
}
