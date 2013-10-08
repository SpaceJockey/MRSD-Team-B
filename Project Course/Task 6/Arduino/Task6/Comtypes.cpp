/* ComTypes.cpp - Communication classes implementation
*  MRSD Team B
*  Project Class Task 6
*/

#import "Comtypes.h"


void InputChannel::setValue(uint16_t val){
	value = val;
}
void InputChannel::set10BitValue(uint16_t val){
	value = (val << 6);
}

uint16_t InputChannel::getValue(){
	return value;
}

void OutputChannel::attachInput(InputChannel * src){
	source = src;
}

void OutputChannel::updateChannel(){
	setValue(source->getValue());
}

void LEDChannel::setValue(uint16_t val) {
	analogWrite(STATUS_LED, val >> 8);
}

void ServoChannel::setValue(uint16_t val){
	this->writeMicroseconds(map(val, 0, 0xFFFF, 1000, 2000));
}

Packet::Packet() {
	//Init our packet frame
	header = 0xDEAD;
	footer = 0xBEEF;
}

//Check a packet to verify no transmission errors
bool Packet::is_valid(){
	//Check headers and footers
	if ((header != 0xDEAD) || (footer != 0xBEEF)) return false;
	
	//calculate checksum
	uint16_t chk = in_chan + out_chan;
	for(int i = 0; i < 8; i++) chk += channel[i];
	if(chk != checksum) return false;

	//Looks ok!
	return true;
}

//Check to see if this is an error packet
bool Packet::is_error(){
	return (in_chan == 0xFF) && (out_chan == 0xFF); 
}

//pack for transmission
void Packet::pack(){
	//re-init our packet frame
	header = 0xDEAD;
	footer = 0xBEEF;
	
	//calculate checksum
	checksum = in_chan + out_chan;
	for(int i = 0; i < 8; i++) checksum += channel[i];
}

void Packet::transmit(){
	Serial.write(chars, sizeof(Packet));
}
